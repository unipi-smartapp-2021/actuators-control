#!/usr/bin/env python

import math
import rospy
import numpy as np
import carla_msgs.msg
from execution import topics
from execution.PIDController import PIDController
from execution.plotter import LivePlotter
from tf.transformations import euler_from_quaternion
from planning.msg import STP_Data
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

class Dispatcher():
    def __init__(self):
        # Initialize Dispatcher Node
        rospy.init_node('dispatcher', anonymous=True)

        self.current_velocity = 0.0
        self.target_velocity = 0.0
        self.current_zorient = 0.0
        self.target_zorient = 0.0
        self.last_cmd = None
        self.enable_actuators = False

        self.plot_pids = rospy.get_param('plot_pids', False)

        self._init_plotters()

        # Controllers
        self.velocity_control = PIDController(0.0,
                Kp=3.0, Ki=1.5, Kd=0.0, minv=0, maxv=1)

        self.steering_control = PIDController(0.0,
                Kp=1.0, Ki=0.1, Kd=0.1, minv=-1.0, maxv=1.0,
                guard=0.1,
                verbose=True)

        self.braking_control = PIDController(0.0,
                Kp=0.1, Ki=0.5, Kd=0.0, minv=0.0, maxv=1.0,
                guard=3.0,
                verbose=False)

        # CARLA vehicle control commands
        self.cmd_pub = rospy.Publisher(
                '/carla/ego_vehicle/vehicle_control_cmd',
                carla_msgs.msg.CarlaEgoVehicleControl,
                queue_size = 6
                )

        # CARLA vehicle information
        self.status_sub = rospy.Subscriber(
                '/carla/ego_vehicle/vehicle_status',
                carla_msgs.msg.CarlaEgoVehicleStatus,
                self.update_status
                )
        # CARLA vehicle information
        self.orientation_sub = rospy.Subscriber(
                '/pose_stamped',
                PoseStamped,
                self.update_orientation
                )

        # Kinematics broker subscription
        self.desired_kinematics_sub = rospy.Subscriber(
                topics.DESIRED_KINEMATICS, STP_Data,
                self.update_command
                )

        self.actuators_enable_sub = rospy.Subscriber(
                topics.ACTUATORS_ENABLE, Bool,
                self.enable_callback
                )

    def _init_plotters(self):
        self.steering_plotter = LivePlotter(
                title='Steering Controller',
                xlabel='Time (s)',
                ylabel='Yaw (rad)',
                xlim=(0, 1), ylim=(-0.3, 0.3),
                blit=True)
        self.velocity_plotter = LivePlotter(
                title='Velocity Controller',
                xlabel='Time (s)',
                ylabel='Velocity (km/h)',
                xlim=(0, 1), ylim=(0.0, 55.0),
                blit=True)


    def enable_callback(self, data):
        if self.enable_actuators and not data.data:
            self.safe_stop()
            self.last_cmd = None
            self.target_velocity = 0.0
            self.target_zorient = 0.0
       
        self.enable_actuators = data.data

    def safe_stop(self):
        """ Stop the car in a safe way """
        rospy.loginfo('Safe stopping...')
        cmd = carla_msgs.msg.CarlaEgoVehicleControl()
        cmd.brake = 0.1
        cmd.throttle = 0.0
        cmd.steer = 0.0
        self.cmd_pub.publish(cmd)
    
    def update_command(self, data):
        self.last_cmd = data
        # Get target velocity
        self.target_velocity = self.current_velocity + data.dv

        # This fixes the buggy behaviour of the STP at the start
        if self.current_velocity <= 1e-2:
            data.dt = 0.0

        self.target_zorient = -data.psi + data.dt
    
    def update_orientation(self, data):
        q = data.pose.orientation
        (x, y, z) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_zorient = z

        if self.last_cmd and self.enable_actuators:
            self.update_control(self.last_cmd)

    def update_status(self, data):
        rospy.loginfo('updating status')
        # get current linear (forward) velocity
        self.current_velocity = data.velocity
        if self.last_cmd and self.enable_actuators:
            self.update_control(self.last_cmd)

    def update_control(self, data):
        rospy.loginfo('current dv: {:.3f}'.format(data.dv))
        rospy.loginfo('current psi: {:.3f}\t target dt: {:.3}'.format(data.psi, data.dt))
        rospy.loginfo('current z: {:.3f}\t target z: {:.3}'.format(self.current_zorient, self.target_zorient))
        rospy.loginfo('current v: {:.3f}\t target v: {:.3}'.format(self.current_velocity, self.target_velocity))
        # Vehicle Control message
        cmd = carla_msgs.msg.CarlaEgoVehicleControl()

        # Control throttle
        cmd.throttle = self.velocity_control.pid_step(self.current_velocity, self.target_velocity)
        # Control steering
        cmd.steer = -self.steering_control.pid_step(self.current_zorient, self.target_zorient)
        # Control braking (if not activating the throttle)
        brake = 1 - self.braking_control.pid_step(self.current_velocity, self.target_velocity)
        if data.dv < 0 and cmd.throttle <= 1e-2:
            cmd.brake = brake

        # Deactivate steering when braking
        # if cmd.brake > 0.0 or self.target_velocity < 1.0:
        #     cmd.steer = 0.0

        rospy.loginfo('current brake: {:.3f}'.format(cmd.brake))
        rospy.loginfo('current steer: {:.3f}'.format(cmd.steer))
        rospy.loginfo('current throttle: {:.3f}'.format(cmd.throttle))
        rospy.loginfo(10*'-')
        self.cmd_pub.publish(cmd)

        if self.plot_pids:
            self.plot_pid(self.steering_control, self.steering_plotter)
            self.plot_pid(self.velocity_control, self.velocity_plotter)
    
    def plot_pid(self, pid, plotter):
        x = pid.get_instants()
        y = pid.get_measurements()

        plotter.set_x(x)
        plotter.set_y(y)
        plotter.plot(reference=pid.get_targets())
    
    def spin(self):
        rospy.spin()

def main():
    dispatcher = Dispatcher()
    dispatcher.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

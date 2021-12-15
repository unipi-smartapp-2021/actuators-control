#!/usr/bin/env python

import rospy
import numpy as np
from execution import topics
from planning.msg import STP_Data
from std_msgs.msg import Float32, String, Int32, Bool
from execution.state import VehicleState

class KinematicsBroker():
    def __init__(self):
        #Initialize Broekr Node
        rospy.init_node('broker', anonymous=True)
        self.vehicle_state = VehicleState(started = False, stopped = False)

        # Receive
        self.stp_sub = rospy.Subscriber(
                topics.STP_DATA, STP_Data,
                self.kinematics_callback
                )
        self.interaction_sub = rospy.Subscriber(
                topics.HUMAN_INTERACTION, String,
                self.interaction_callback
                )

        # Publish
        self.desired_kinematics_pub = rospy.Publisher(
                topics.DESIRED_KINEMATICS, STP_Data,
                queue_size=5)

        self.actuators_enable_pub = rospy.Publisher(
                topics.ACTUATORS_ENABLE, Bool,
                queue_size=5)

    def kinematics_callback(self, data):
        if self.vehicle_state.is_started() and not self.vehicle_state.is_stopped():
            rospy.loginfo('[KINEMATICS BROKER] Sending kinematics data: {}'.format(data))
            self.desired_kinematics_pub.publish(data)

    def interaction_callback(self, data):
        rospy.loginfo('[KINEMATICS BROKER] Received new human_interaction command: {}'.format(data))
        if data.data == 'start':
            rospy.loginfo('[KINEMATICS BROKER] Registered starting command')
            self.vehicle_state.set_started(True)
            self.vehicle_state.set_stopped(False)
            self.actuators_enable_pub.publish(True)
        elif data.data == 'stop':
            rospy.loginfo('[KINEMATICS BROKER] Registered stopping command')
            self.vehicle_state.set_stopped(True)
            self.vehicle_state.set_started(False)
            self.actuators_enable_pub.publish(False)

    def spin(self):
        rospy.spin()

def main():
    broker = KinematicsBroker()
    broker.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


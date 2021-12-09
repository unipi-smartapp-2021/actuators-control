#!/usr/bin/env python

import rospy
import numpy as np
from execution import topics
from std_msgs.msg import Float32, String, Int32

class Broker():
	def __init__(self, buffer_size):
		#Initialize Broekr Node
		rospy.init_node('broker', anonymous=True)

		self.buffer = []
		self.buffer_size = buffer_size

		# Receive
		self.get_data_kinem = rospy.Subscriber(topics.KINEMATICS, Float32, lambda data: self.get_data(data))
		#self.get_data_sens 	= rospy.Subscriber(topics.SENSOR_FEED, Int32, lamda data: self.callback(data))

		self.get_imp_command = rospy.Subscriber(topics.SENSORY, String, lambda data:self.get_command(data))

		# Publish
		self.send_data_disp = rospy.Publisher(topics.SIGNAL, Float32, queue_size=4)

	def get_command(data):
		rospy.loginfo(data)

	def get_data(self, data): 
		self.buffer.append(data)
		if len(self.buffer) > self.buffer_size:
			self.buffer = self.buffer[1:]
		rospy.loginfo(self.buffer)

	def spin(self):
		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			#print(self.buffer)
			random_number = np.random.rand()
			print(random_number)
			self.send_data_disp.publish(random_number)
			rate.sleep()

def main():
    broker = Broker(5)
    broker.spin()

if __name__ == '__main__':
	try:
	    main()
	except rospy.ROSInterruptException:
	    pass
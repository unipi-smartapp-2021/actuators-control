#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

import rospy
import numpy as np
from execution import topics
from std_msgs.msg import Float32, String
#from TO_DO_MESS.msg import TO_DO

class Broker():
	def __init__(self, buffer_size):
		# Initialize Broker Node
		rospy.init_node('broker', anonymous=True)

		self.buffer = []
		self.buffer_size = buffer_size

		# Receive
		self.get_data_kinem = rospy.Subscriber(topics.KINEMATICS, Float32, lambda data: self.callback(data))

		# Publish
		self.send_data_transl = rospy.Publisher(topics.SIGNAL, Float32, queue_size=4)

	def callback(self, data): 
		# how to manage the buffer size?
		self.buffer.append(data)
		if len(self.buffer) > self.buffer_size:
			self.buffer = self.buffer[1:]
		rospy.loginfo(self.buffer)

	def spin(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			print(self.buffer)
			# random_number = np.random.rand()
			# send_data_transl.publish(random_number)
			rate.sleep()

def main():
    broker = Broker(5)
    broker.spin()

if __name__ == '__main__':
	try:
	    main()
	except rospy.ROSInterruptException:
	    pass
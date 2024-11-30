#!/usr/bin/env python3
'''

Remote control of a differential wheeled robot using Bluetooth HC-06 device
Author: Aleksandar Haber 
Date: January-May 2023

Here is the complete description:

The Linux ROS computer sends the control commands (motor velocities) remotely by using 
the bluetooth device to the Arduino microcontroller. 
Arduino microcontroller controlls two motors. It receives control commands and executes them. 
Then, the Arduino microcontroller sends back encoder readings back to the computer. 
We print the received encoder readings to the screen. 

Topics for sending control actions: "left_motor_velocity" and "right_motor_velocity"

READ THE LICENSE!

'''
import rospy
# we are sending Int32 message
from std_msgs.msg import Int32

# this is our node name
nodeName='messagepublisher'

# these are the topic names- Make sure that in the Arduino code, the subcriber nodes are subscribed to these exact topics
topicNameLeftMotor='left_motor_velocity'
topicNameRightMotor='right_motor_velocity'

# here, we initialize our publisher node 
# we set "anonymous=True" to make sure that the node has a unique name 
# this parameter will add random numbers to the end of the node name 
rospy.init_node(nodeName,anonymous=True)

# here, we are saying that our node is publishing messages to the specified topic names
# we specify the type of messages we want to publish (Int32)
# queue_size=5 simply means that we limit the number of queued messages if the subscriber 
# cannot receive the messages fast enough
publisherLeftMotor=rospy.Publisher(topicNameLeftMotor, Int32, queue_size=5)
publisherRightMotor=rospy.Publisher(topicNameRightMotor, Int32, queue_size=5)

# here we specify the frequency of publishing the messages
# that is, we publish the messages with 1 [Hz]
ratePublisher=rospy.Rate(1)

# here we initialize the integers that we plan to send
# The values are from 0 - 255 
# velocity of the left motor 
leftMotor=0
# velocity of the right motor
rightMotor=0

while not rospy.is_shutdown():
	# these lines are for debugging
	rospy.loginfo(leftMotor)
	rospy.loginfo(rightMotor)
	# ask the user to enter the left and right motor velocities
	leftMotor = int(input("Enter left motor velocity (0-255): \n "))
	rightMotor =int(input("Enter right motor velocity (0-255): \n "))
	# here, we publish the message
	publisherLeftMotor.publish(leftMotor)
	publisherRightMotor.publish(rightMotor)

	# pause for the previously defined rate
	ratePublisher.sleep()
	
	
	

#!/usr/bin/env python3 
'''

Author: Aleksandar Haber 
Date: January -May 2024

This is the subscriber node that receives encoder readings from Arduino 
and that calculates position x and y of the center of the robot as well as the
robot orientation angle (angle theta). These values are calculated from the encoder readings
by first calculating the velocity, and then x,y, and theta. That is, this code performs 
dead reckoning. After calculating x,y, and theta, we create a quaternion and create an Odometry ROS object. 
The odometry is then published. RViz uses the published odometry to graph and track the position and orientation of the robot in space

For more details on dead reckoning, see


https://aleksandarhaber.com/what-is-dead-reckoning-clear-explanation-with-python-simulation-part-i/

READ THE LICENSE! 

'''
###############################################################################
# START: import the necessary libraries, modules and packages
###############################################################################

# ROS imports
import rospy

# we expect to receive Int32 from the left and right encoders
from std_msgs.msg import Int32

# this is the odometry data structure that this code 
# will continiously update
from nav_msgs.msg import Odometry

# we need quaternions
from geometry_msgs.msg import Quaternion

# see this later on
from tf.broadcaster import TransformBroadcaster

# Python imports 
from math import cos, sin, pi

###############################################################################
# END: import the necessary libraries, modules and packages
###############################################################################

###############################################################################
# START: dead reckoning class 
###############################################################################
      
class DeadReckoningOdom():
    
    ###############################################################################
    # START: Constructor function
    ###############################################################################
      
    def __init__(self):
        ###############################################################################
        # START: Definition of the variables, constants, parameters, names, etc.
        ###############################################################################
        # this is the node name that corresponds to this class 
        # note that ROS might add a random number at the end of this node
        self.nodeName='DeadReckoningOdom'
    
        # these are the topic names
        # these topic names have to match the topic names specified in the Arduino code
        self.topicNameLeftEncoder='left_encoder_pulses'
        self.topicNameRightEncoder='right_encoder_pulses'
        
        #frame names for broadcasting odometry 
        self.baseFrameName=rospy.get_param('~base_frame_id','base_frame')
        self.odomFrameName=rospy.get_param('~odom_frame_id','odom_frame')
        
        # wheel radius in meters
        self.wheelRadius=0.0326 
        # distance between the wheels in meters 
        self.distanceWheels=0.28
        
        # encoder LM393 specs 
        # this encoder will read 20 pulses per one full revolution 
        self.encoderPulsesConstant=20
        
        # updateFrequency for publishing odometry in Hz
        self.updateFrequencyPublish=10
        
        # flag for initial reading - these variables are set to zero after 
        # the first reading from the encoder topics
        self.flagInitialLeftEncoder=1
        self.flagInitialRightEncoder=1
        
        # these variables are used to store the initial readings from the encoder topics
        # namely, when you start the system, you might have non-zero encoder readings
        # we need to store these readings in these variables
        # here, they are initialized by zeros, however, they are updated in the first encoder reading
        self.initialValueLeftEncoder=0
        self.initialValueRightEncoder=0
        
        # left encoder readings
        # current time step k
        self.currentValueLeftEncoder=0
        # previous time step k-1
        self.pastValueLeftEncoder=0
        
        # right encoder readings
        # current time step k
        self.currentValueRightEncoder=0
        # previous time step k-1
        self.pastValueRightEncoder=0
                
        # time 
        # callback functions for the right and left encoders will update 
        # these time stampt
        # current time stamp for left encoder
        self.currentTimeLeftEncoder=0
        # current time stamp for right encoder
        self.currentTimeRightEncoder=0
        
        # the function self.calculateUpdate will use these time veriables and
        # it will update them on the basis of 
        # self.currentTimeLeftEncoder and self.currentTimeRightEncoder
        # current and past time for the complete code
        self.currentTime=0
        # this time is initialized with the current ROS time below 
        # after calling the init_ndoe
        self.pastTime=0
        
         # kinematic states 
        # x and y coordinates of the mobile robot
        self.x=0
        self.y=0
        # orientation angle
        self.theta=0
        
        ###############################################################################
        # END: Definition of the constants, parameters, names, etc.
        ###############################################################################    
        
        ###############################################################################
        # START: Start the node
        ###############################################################################
        # here, we initialize our node 
        # we set "anonymous=True" to make sure that the node has a unique name 
        # this parameter will add random numbers to the end of the node name 	
        rospy.init_node(self.nodeName,anonymous=True)
        
        # get the past time
        self.pastTime=rospy.get_time()
        # update the node name 
        self.nodeName = rospy.get_name()
        # print the message
        rospy.loginfo("The node - %s has started" % self.nodeName)
        ###############################################################################
        # END: Start the node
        ###############################################################################

        ###############################################################################
        # START: Subscribers and publishers
        ###############################################################################
        # NOTE HERE THAT THE TWO CALLBACK FUNCTIONS ARE DEFINED LATER IN THE CODE
        
        # subscribe to the messages from the left encoder
        rospy.Subscriber(self.topicNameLeftEncoder, Int32, self.callBackFunctionLeftEncoder)
        # subscribe to the messages from the right encoder
        rospy.Subscriber(self.topicNameRightEncoder, Int32, self.callBackFunctionRightEncoder)
        
        
        # we publish an odometry             
        self.odometryPublisher = rospy.Publisher("odom", Odometry,queue_size=5)
        self.odometryBroadcaster = TransformBroadcaster()
    
        ###############################################################################
        # END: Subscribers and publishers
        ###############################################################################
    
    ###############################################################################
    # END: Constructor function
    ###############################################################################
    

    ###############################################################################
    # START: callback function
    ###############################################################################
    # These are the callback functions, that are called when the messages are received
    # These functions will print the encoder readings to the screen
    # NOTE THAT THESE FUNCTIONS ARE RUNNING IN THE BACKGROUND BY THE ROS SYSTEM
    # left encoder callback function
    def callBackFunctionLeftEncoder(self,message1):
        if self.flagInitialLeftEncoder==1:
            self.initialValueLeftEncoder=message1.data
            self.flagInitialLeftEncoder=0
        else:
            self.currentValueLeftEncoder=message1.data-self.initialValueLeftEncoder
            self.currentTimeLeftEncoder=rospy.get_time()            
        #print("Left encoder pulses: %s" %(message1.data))
        #print("Time stamp left encoder %s" %(rospy.get_time()))

    # right encoder callback function
    def callBackFunctionRightEncoder(self,message2):
        if self.flagInitialRightEncoder==1:
            self.initialValueRightEncoder=message2.data
            self.flagInitialRightEncoder=0
        else:
            self.currentValueRightEncoder=message2.data-self.initialValueRightEncoder
            self.currentTimeRightEncoder=rospy.get_time()
        
        #print("Right encoder pulses: %s" %message2.data)
        #print("Time stamp right encoder %s" %(rospy.get_time()))
    
    ###############################################################################
    # END: callback function
    ###############################################################################  
    
    
    ###############################################################################
    # START: Update function
    ###############################################################################    
    # this function performs calculations of x,y, and theta on the basis of the encoder 
    # readings and after that it creates a quaternion and publishes odometry 
    # https://aleksandarhaber.com/what-is-dead-reckoning-clear-explanation-with-python-simulation-part-i/
    
    def calculateUpdate(self):

        # get the current time 
        self.currentTime=rospy.get_time() 
        
        
        if (self.currentTime > self.pastTime):
           
            # change in time
            # this is the time step
            deltaT=self.currentTime-self.pastTime
            
            # this is for test
            #self.currentValueLeftEncoder=self.pastValueLeftEncoder+1
            #self.currentValueRightEncoder=self.pastValueRightEncoder-1
            
            # calculate left and right encoder angle changes
            # (2*pi/self.encoderPulsesConstant) is the angle for 1 encoder tick
            leftEncoderAngleChange=(2*pi/self.encoderPulsesConstant)*(self.currentValueLeftEncoder-self.pastValueLeftEncoder)
            rightEncoderAngleChange=(2*pi/self.encoderPulsesConstant)*(self.currentValueRightEncoder-self.pastValueRightEncoder)
            
            # calculate left and right encoder angular velocities
            leftWheelAngularVelocity=leftEncoderAngleChange/deltaT
            rightWheelAngularVelocity=rightEncoderAngleChange/deltaT
            
            # calculate left and right wheel velocities
            leftWheelVelocity=self.wheelRadius*leftWheelAngularVelocity
            rightWheelVelocity=self.wheelRadius*rightWheelAngularVelocity
            
            # calculate the velocity of the robot center
            velocity=(leftWheelVelocity+rightWheelVelocity)/2
            
            # calculate the angular velocity of the center of the robot
            angularVelocity=(rightWheelVelocity-leftWheelVelocity)/self.distanceWheels
            
            
            # dead reckoning equations
            distanceTraveled=velocity*deltaT
            angleChange=angularVelocity*deltaT
            
            # for test
            #distanceTraveled=0.01
            #angleChange=0
            
            # this is the set of equations given in the equation 8 of
            # https://aleksandarhaber.com/what-is-dead-reckoning-clear-explanation-with-python-simulation-part-i/
            
            self.x=self.x+distanceTraveled*cos(self.theta)    
            self.y=self.y+distanceTraveled*sin(self.theta)
            self.theta=self.theta+angleChange
            
            #if (self.theta>2*pi):
            #    self.theta=self.theta-2*pi
            
        # here, we print the results for inspection
        print("x,y,theta: (%s,%s,%s)"%(self.x,self.y,self.theta))
        #print("Current value left encoder %s"%self.currentValueLeftEncoder)
        print("Time difference %s"%(self.currentTime-self.pastTime))
        #print("Current time %s"%(self.currentTime))
        #print("Angular velocities (%s,%s)"%(leftWheelAngularVelocity,rightWheelAngularVelocity))
        #print("Encoder: (%s,%s) "%(self.currentValueLeftEncoder,self.currentValueRightEncoder))
        
        
        # create a quaternion object and compute the values
        quaternion = Quaternion()
        # create a data structure for the quaternion
        quaternion1=Quaternion()
        quaternion1.x=0
        quaternion1.y=0
        quaternion1.z=sin(self.theta/2)
        quaternion1.w=cos(self.theta/2)
        quaternionTuple=(quaternion1.x,quaternion1.y,quaternion1.z,quaternion1.w)
        
        # publish odometry
        self.odometryBroadcaster.sendTransform((self.x, self.y, 0),
               quaternionTuple,rospy.Time.now(),
               self.baseFrameName,self.odomFrameName)
        
        odometry1 = Odometry()
        odometry1.header.stamp = rospy.Time.now()
        odometry1.header.frame_id = self.odomFrameName
        odometry1.pose.pose.position.x = self.x
        odometry1.pose.pose.position.y = self.y
        odometry1.pose.pose.position.z = 0
        odometry1.pose.pose.orientation = quaternion1
        odometry1.child_frame_id = self.baseFrameName
        odometry1.twist.twist.linear.x = velocity
        odometry1.twist.twist.linear.y = 0
        odometry1.twist.twist.angular.z = angularVelocity
        
        self.odometryPublisher.publish(odometry1)
        
                
        # for the next iteration, update the time step and encoder values
        self.pastTime=self.currentTime
        self.pastValueLeftEncoder=self.currentValueLeftEncoder
        self.pastValueRightEncoder=self.currentValueRightEncoder

   ###############################################################################
   # END: Update function
   ###############################################################################   

   ###############################################################################
   # START: main function
   ###############################################################################   


    def mainLoop(self):
        
        ROSRate= rospy.Rate(self.updateFrequencyPublish)
        
        while not rospy.is_shutdown():
            self.calculateUpdate()
            ROSRate.sleep()
   ###############################################################################
   # START: end function
   ###############################################################################   
###############################################################################
# END: dead reckoning class 
###############################################################################      

if __name__ == "__main__":
    """ main """
    objectDR = DeadReckoningOdom()
    objectDR.mainLoop()


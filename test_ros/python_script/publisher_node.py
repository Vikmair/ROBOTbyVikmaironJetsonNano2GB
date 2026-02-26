#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO  # Cambié a RPi.GPIO
import threading

class MotorEncoderController:
    def __init__(self):
        
        rospy.init_node("motor_encoder_controller", anonymous=True)
        rospy.loginfo("Motor Encoder Controller Node Initialized")

        GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering

        self.encoder_pin_left = 7  # Pin 7 (GPIO 4)
        self.encoder_pin_right = 11  # Pin 11 (GPIO 17)

        self.total_pulses_left = 0
        self.total_pulses_right = 0

        GPIO.setup(self.encoder_pin_left, GPIO.IN)
        GPIO.setup(self.encoder_pin_right, GPIO.IN)

        GPIO.add_event_detect(self.encoder_pin_left, GPIO.RISING, callback=self.encoder_callback_left)
        GPIO.add_event_detect(self.encoder_pin_right, GPIO.RISING, callback=self.encoder_callback_right)

        self.left_encoder_publisher = rospy.Publisher("left_encoder_pulses", Int32, queue_size=10)
        self.right_encoder_publisher = rospy.Publisher("right_encoder_pulses", Int32, queue_size=10)

        self.encoder_thread = threading.Thread(target=self.publish_encoder_data)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()

    def encoder_callback_left(self, channel):
        self.total_pulses_left += 1

    def encoder_callback_right(self, channel):
        self.total_pulses_right += 1

    def publish_encoder_data(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Publish encoder pulse data to ROS topics
            self.left_encoder_publisher.publish(self.total_pulses_left)
            self.right_encoder_publisher.publish(self.total_pulses_right)
            rate.sleep()

    def cleanup(self):
        GPIO.cleanup()
        rospy.loginfo("GPIO cleanup complete")

if __name__ == "__main__":
    try:
        motor_encoder_controller = MotorEncoderController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        motor_encoder_controller.cleanup()


#!/usr/bin/env python3

import smbus
import rospy
import numpy as np
from sensor_msgs.msg import Temperature, Imu
from tf.transformations import quaternion_from_euler
from mpu_6050_driver.registers import (
    PWR_MGMT_1, ACCEL_XOUT_H, ACCEL_YOUT_H, ACCEL_ZOUT_H, 
    TEMP_H, GYRO_XOUT_H, GYRO_YOUT_H, GYRO_ZOUT_H
)

# Filtro Complementario solo para el eje Z (Yaw)
class ComplementaryFilter:
    def __init__(self, alpha=1):
        self.alpha = alpha
        self.angle_z = 0.0  # Solo guardamos el ángulo Yaw (eje Z)

    def update(self, accel_angles, gyro_rates, dt):
        # Solo actualizamos el ángulo en el eje Z (Yaw)
        self.angle_z += gyro_rates[2] * dt  # Solo usaremos el giro en Z
        return self.angle_z

ADDR = None
bus = None
IMU_FRAME = None
complementary_filter = ComplementaryFilter()
last_time = None

# Calibración de Giroscopio
gyrXoffs = 0.0
gyrYoffs = 0.0
gyrZoffs = 0.0

def calibrate_gyro():
    global gyrXoffs, gyrYoffs, gyrZoffs
    # Inicia la calibración del giroscopio para eliminar el offset
    num_samples = 500
    x_sum, y_sum, z_sum = 0, 0, 0
    for _ in range(num_samples):
        x_sum += read_word_2c(GYRO_XOUT_H)
        y_sum += read_word_2c(GYRO_YOUT_H)
        z_sum += read_word_2c(GYRO_ZOUT_H)
    gyrXoffs = x_sum / num_samples
    gyrYoffs = y_sum / num_samples
    gyrZoffs = z_sum / num_samples

def read_word(adr):
    high = bus.read_byte_data(ADDR, adr)
    low = bus.read_byte_data(ADDR, adr + 1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

def publish_temp(timer_event):
    temp_msg = Temperature()
    temp_msg.header.frame_id = IMU_FRAME
    temp_msg.temperature = read_word_2c(TEMP_H) / 340.0 + 36.53  # Calculando la temperatura en °C
    temp_msg.header.stamp = rospy.Time.now()
    temp_pub.publish(temp_msg)

def publish_imu(timer_event):
    global last_time
    imu_msg = Imu()
    imu_msg.header.frame_id = IMU_FRAME

    accel_x = read_word_2c(ACCEL_XOUT_H) / 16384.0
    accel_y = read_word_2c(ACCEL_YOUT_H) / 16384.0
    accel_z = read_word_2c(ACCEL_ZOUT_H) / 16384.0

    gyro_x = (read_word_2c(GYRO_XOUT_H) - gyrXoffs) / 131.0
    gyro_y = (read_word_2c(GYRO_YOUT_H) - gyrYoffs) / 131.0
    gyro_z = (read_word_2c(GYRO_ZOUT_H) - gyrZoffs) / 131.0

    # Solo calculamos el ángulo Yaw (eje Z)
    current_time = rospy.Time.now().to_sec()
    dt = current_time - last_time
    last_time = current_time

    # Actualizamos solo el ángulo en Z (Yaw)
    yaw = complementary_filter.update(
        (0, 0),  # No usamos los ángulos de acelerómetro para X y Y
        (gyro_x, gyro_y, gyro_z),  # Usamos el giro solo en Z
        dt
    )
    
    

    # Usamos el ángulo Yaw para generar el cuaternión
    orientation = quaternion_from_euler(0, 0, np.radians(yaw))
    imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = orientation

    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z

    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z

    imu_msg.header.stamp = rospy.Time.now()
    imu_pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('imu_node')

    bus = smbus.SMBus(rospy.get_param('~bus', 1))
    ADDR = rospy.get_param('~device_address', 0x68)
    if type(ADDR) == str:
        ADDR = int(ADDR, 16)

    IMU_FRAME = rospy.get_param('~imu_frame', 'imu_link')

    bus.write_byte_data(ADDR, PWR_MGMT_1, 0)  # Despertamos el MPU-6050
    calibrate_gyro()  # Realizamos la calibración del giroscopio

    temp_pub = rospy.Publisher('temperature', Temperature, queue_size=10)
    imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
    
    last_time = rospy.Time.now().to_sec()  # Inicialización después de init_node
    imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)  # Publicar IMU cada 20ms
    temp_timer = rospy.Timer(rospy.Duration(10), publish_temp)  # Publicar temperatura cada 10s
    rospy.spin()

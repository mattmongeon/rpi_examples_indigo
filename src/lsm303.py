#!/usr/bin/env python

import rospy
import smbus
import sys
import time
import math
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Temperature

# Portions of code borrowed from http://forums.adafruit.com/viewtopic.php?f=19&t=37342

LIN_ACCEL_ADDR = 0x19
COMPASS_ADDR = 0x1E

G_RESOLUTION = 32768 / 4
G_TO_M_S2 = 9.80665
X_ACCEL_OFFSET = 0
Y_ACCEL_OFFSET = 0
Z_ACCEL_OFFSET = 450

# Accelerometer registers

LIN_ACCEL_REG_CTRL_REG1_A = 0x20
LIN_ACCEL_REG_CTRL_REG4_A = 0x23
LIN_ACCEL_REG_OUT_X_L_A = 0x28
LIN_ACCEL_REG_OUT_X_H_A = 0x29
LIN_ACCEL_REG_OUT_Y_L_A = 0x2A
LIN_ACCEL_REG_OUT_Y_H_A = 0x2B
LIN_ACCEL_REG_OUT_Z_L_A = 0x2C
LIN_ACCEL_REG_OUT_Z_H_A = 0x2D

# Magnetometer registers

MAG_REG_CRA_REG_M = 0x00
MAG_REG_MR_REG_M = 0x02
MAG_REG_OUT_X_H_M = 0x03
MAG_REG_OUT_X_L_M = 0x04
MAG_REG_OUT_Z_H_M = 0x05
MAG_REG_OUT_Z_L_M = 0x06
MAG_REG_OUT_Y_H_M = 0x07
MAG_REG_OUT_Y_L_M = 0x08
MAG_REG_TEMP_OUT_H_M = 0x31
MAG_REG_TEMP_OUT_L_M = 0x32

bus = smbus.SMBus(1)

imu_pub = rospy.Publisher("lin_accel", Imu, queue_size=10)
temp_pub = rospy.Publisher("temperature_degf", Temperature, queue_size=10)

def write_accel_reg(register, value):
	global bus
	bus.write_byte_data(LIN_ACCEL_ADDR, register, value)

def write_mag_reg(register, value):
	global bus
   	bus.write_byte_data(COMPASS_ADDR, register, value)

def read_accel_reg(register):
	global bus
	return bus.read_byte_data(LIN_ACCEL_ADDR, register)

def read_mag_reg(register):
	global bus
	return bus.read_byte_data(COMPASS_ADDR, register)

def convert_twos_complement(input):
	ret = input
	if input >= 0x8000:
		ret = ret ^ 0xFFFF
		ret += 1
		ret *= -1
	return ret

def get_x_accel():
	lo = read_accel_reg(LIN_ACCEL_REG_OUT_X_L_A)
	hi = read_accel_reg(LIN_ACCEL_REG_OUT_X_H_A)
	total = (hi << 8) + lo
	total += X_ACCEL_OFFSET
	total = convert_twos_complement(total)
	return (float(total) / float(G_RESOLUTION)) * G_TO_M_S2

def get_y_accel():
	lo = read_accel_reg(LIN_ACCEL_REG_OUT_Y_L_A)
	hi = read_accel_reg(LIN_ACCEL_REG_OUT_Y_H_A)
	total = (hi << 8) + lo
	total += Y_ACCEL_OFFSET
	total = convert_twos_complement(total)
	return (float(total) / float(G_RESOLUTION)) * G_TO_M_S2

def get_z_accel():
	lo = read_accel_reg(LIN_ACCEL_REG_OUT_Z_L_A)
	hi = read_accel_reg(LIN_ACCEL_REG_OUT_Z_H_A)
	total = (hi << 8) + lo
	total += Z_ACCEL_OFFSET
	total = convert_twos_complement(total)
	return (float(total) / float(G_RESOLUTION)) * G_TO_M_S2

def get_compass_heading():
	# x value
	lo = read_mag_reg(MAG_REG_OUT_X_L_M)
	hi = read_mag_reg(MAG_REG_OUT_X_H_M)
	x_total = convert_twos_complement((hi << 8) + lo)

	# y value
	lo = read_mag_reg(MAG_REG_OUT_Y_L_M)
	hi = read_mag_reg(MAG_REG_OUT_Y_H_M)
	y_total = convert_twos_complement((hi << 8) + lo)

	# z value
	lo = read_mag_reg(MAG_REG_OUT_Z_L_M)
	hi = read_mag_reg(MAG_REG_OUT_Z_H_M)
	z_total = convert_twos_complement((hi << 8) + lo)

	return x_total, y_total, z_total

def get_temperature_degf():
	lo = read_mag_reg(MAG_REG_TEMP_OUT_L_M)
	lo = lo >> 4
	hi = read_mag_reg(MAG_REG_TEMP_OUT_H_M)
	total = (hi << 4) + lo
	total = convert_twos_complement(total)
	total /= 8
	
	return total
	

print_debug = False

def run():

	global print_debug

	# First initialize the module.
	write_accel_reg(LIN_ACCEL_REG_CTRL_REG1_A, 0x47)
	write_accel_reg(LIN_ACCEL_REG_CTRL_REG4_A, 0x90)

	write_mag_reg(MAG_REG_CRA_REG_M, 0x94)
	write_mag_reg(MAG_REG_MR_REG_M, 0x00)

	# Now get everything setup to run in ROS.
	rospy.init_node("lsm303", anonymous=True)

	print_debug = rospy.get_param("~print_debug", False)

	rospy.Timer(rospy.Duration(0.1), timer_callback)

	rospy.spin()


def timer_callback(event):
	global imu_pub, print_debug

	x_accel = get_x_accel()
	y_accel = get_y_accel()
	z_accel = get_z_accel()

	x_mag, y_mag, z_mag = get_compass_heading()

	temp_deg_f = get_temperature_degf()

	imu_msg = Imu()
	imu_msg.header.stamp = rospy.Time.now()
	imu_msg.linear_acceleration.x = x_accel
	imu_msg.linear_acceleration.y = y_accel
	imu_msg.linear_acceleration.z = z_accel

	imu_pub.publish(imu_msg)

	temp_msg = Temperature()
	temp_msg.header.stamp = rospy.Time.now()
	temp_msg.temperature = float(temp_deg_f)
	temp_msg.variance = 0

	temp_pub.publish(temp_msg)

	if print_debug:
		s = "X Accel = " + str(x_accel) + "\r\n"
		s += "Y Accel = " + str(y_accel) + "\r\n"
		s += "Z Accel = " + str(z_accel) + "\r\n"
		s += "X Mag = " + str(x_mag) + "\r\n"
		s += "Y Mag = " + str(y_mag) + "\r\n"
		s += "Z Mag = " + str(z_mag) + "\r\n"
		s += "Temp = " + str(temp_deg_f) + "\r\n"
		print s


def main(args):
	run()

if __name__ == "__main__":
	main(sys.argv)


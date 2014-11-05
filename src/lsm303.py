#!/usr/bin/env python

import rospy
import smbus
import sys
import time
import math
from sensor_msgs.msg import Imu

# Lots of code borrowed from http://forums.adafruit.com/viewtopic.php?f=19&t=37342

# The I2C addresses of the accelerometer and magnetometer.
LIN_ACCEL_ADDR = 0x19
COMPASS_ADDR = 0x1E

# Convenience variables used down below.
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

# Writes a value to a particular register in the accelerometer.
def write_accel_reg(register, value):
	global bus
	bus.write_byte_data(LIN_ACCEL_ADDR, register, value)

# Writes a value to a particular register in the magnetometer.
def write_mag_reg(register, value):
	global bus
   	bus.write_byte_data(COMPASS_ADDR, register, value)

# Reads the value from a particular register in the accelerometer.
def read_accel_reg(register):
	global bus
	return bus.read_byte_data(LIN_ACCEL_ADDR, register)

# Reads the value from a particular register in the magnetometer.
def read_mag_reg(register):
	global bus
	return bus.read_byte_data(COMPASS_ADDR, register)

# The values come in as 16-bit twos-complement values, although the
# Python code will initially interpret it as an unsigned value.  This
# function corrects that so that the value now becomes a signed integer.
def convert_twos_complement(input):
	ret = input
	if input >= 0x8000:
		ret = ret ^ 0xFFFF
		ret += 1
		ret *= -1
	return ret

# Gets the x-acceleration data from the accelerometer and returns a
# value in m/sec^2.
def get_x_accel():
	lo = read_accel_reg(LIN_ACCEL_REG_OUT_X_L_A)
	hi = read_accel_reg(LIN_ACCEL_REG_OUT_X_H_A)
	total = (hi << 8) + lo
	total += X_ACCEL_OFFSET
	total = convert_twos_complement(total)
	return (float(total) / float(G_RESOLUTION)) * G_TO_M_S2

# Gets the y-acceleration data from the accelerometer and returns a
# value in m/sec^2.
def get_y_accel():
	lo = read_accel_reg(LIN_ACCEL_REG_OUT_Y_L_A)
	hi = read_accel_reg(LIN_ACCEL_REG_OUT_Y_H_A)
	total = (hi << 8) + lo
	total += Y_ACCEL_OFFSET
	total = convert_twos_complement(total)
	return (float(total) / float(G_RESOLUTION)) * G_TO_M_S2

# Gets the z-acceleration data from the accelerometer and returns a
# value in m/sec^2.
def get_z_accel():
	lo = read_accel_reg(LIN_ACCEL_REG_OUT_Z_L_A)
	hi = read_accel_reg(LIN_ACCEL_REG_OUT_Z_H_A)
	total = (hi << 8) + lo
	total += Z_ACCEL_OFFSET
	total = convert_twos_complement(total)
	return (float(total) / float(G_RESOLUTION)) * G_TO_M_S2

# Reads magnetometer values for x-, y-, and z-axes and returns the
# unscaled value as a signed integer.
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

# Set this to True through the private parameter to enable printing
# accelerometer and magnetometer data to the terminal in real-time.
print_debug = False

def run():

	global print_debug

	# First initialize the accelerometer to update all three axes at
        # 50 Hz and to use a full scale of +/-4G.
	write_accel_reg(LIN_ACCEL_REG_CTRL_REG1_A, 0x47)
	write_accel_reg(LIN_ACCEL_REG_CTRL_REG4_A, 0x90)

        # Initialize the magnetometer to update all three axes at 30 Hz.
        # Also enables the temperature sensor, although we are not going
        # to do anything with it.
	write_mag_reg(MAG_REG_CRA_REG_M, 0x94)
	write_mag_reg(MAG_REG_MR_REG_M, 0x00)

	# Now get everything setup to run in ROS.
	rospy.init_node("lsm303", anonymous=True)

	print_debug = rospy.get_param("~print_debug", False)

        # We will read from the module at 20 Hz.
	rospy.Timer(rospy.Duration(0.05), timer_callback)

	rospy.spin()


def timer_callback(event):
	global imu_pub, print_debug

        # Read our acceleration values from the accelerometer.
	x_accel = get_x_accel()
	y_accel = get_y_accel()
	z_accel = get_z_accel()

        # Read our magnetometer values.
	x_mag, y_mag, z_mag = get_compass_heading()

        # Put together an Imu message that contains our
        # updated acceleration values.
	imu_msg = Imu()
	imu_msg.header.stamp = rospy.Time.now()
	imu_msg.linear_acceleration.x = x_accel
	imu_msg.linear_acceleration.y = y_accel
	imu_msg.linear_acceleration.z = z_accel

	imu_pub.publish(imu_msg)

        # This can be enabled using the private parameter.  Just provides
        # an easy debugging interface.
	if print_debug:
                s = "X Accel = " + str(x_accel) + "\r\n"
                s += "Y Accel = " + str(y_accel) + "\r\n"
                s += "Z Accel = " + str(z_accel) + "\r\n"
                s += "X Mag = " + str(x_mag) + "\r\n"
                s += "Y Mag = " + str(y_mag) + "\r\n"
                s += "Z Mag = " + str(z_mag) + "\r\n"
                print s


def main(args):
	run()

if __name__ == "__main__":
	main(sys.argv)


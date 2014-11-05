#!/usr/bin/env python

import roslib
import rospy
import tf
import sys
from sensor_msgs.msg import Imu

# Some variables for holding position and velocity.
x_pos = 0.0
y_pos = 0.0
z_pos = 0.0

v_x = 0.0
v_y = 0.0
v_z = 0.0

prev_time_s = 0.0

# This is the object that will be used for transmitting tf data.
br = tf.TransformBroadcaster()

x_accel_offset = 0.0
y_accel_offset = 0.0
z_accel_offset = 0.0
print_debug = False

def run():
    global x_accel_offset, y_accel_offset, z_accel_offset, print_debug

    rospy.init_node("pi_position")

    # These parameters can be used to offset the acceleration values from the accelerometer
    # in case it drifts or there are any offsets that need to be accounted for, such as
    # the accelerometer being at an angle.
    x_accel_offset = rospy.get_param("~x_accel_offset", 0.0)
    y_accel_offset = rospy.get_param("~y_accel_offset", 0.0)
    z_accel_offset = rospy.get_param("~z_accel_offset", 9.80665)

    # This variable will allow printing of acceleration and velocity data to the terminal.
    print_debug = rospy.get_param("~print_debug", False)

    # This will pick up linear acceleration data from the accelerometer.
    imu_sub = rospy.Subscriber("/lin_accel", Imu, received_callback)

    rospy.spin()
        

def received_callback(data):
    global x_pos, y_pos, z_pos, v_x, v_y, v_z, prev_time_s
    global br, x_accel_offset, y_accel_offset, z_accel_offset, print_debug

    current_time_s = rospy.get_time()
    if prev_time_s != 0.0:
        delta_t = current_time_s - prev_time_s

        x_pos += v_x * delta_t
        y_pos += v_y * delta_t
        z_pos += v_z * delta_t

        if print_debug:
            s = "x_accel = " + str(data.linear_acceleration.x + x_accel_offset) + "\r\n"
            s += "y_accel = " + str(data.linear_acceleration.y + y_accel_offset) + "\r\n"
            s += "z_accel = " + str(data.linear_acceleration.z + z_accel_offset) + "\r\n"
            s += "x_vel = " + str(v_x) + "\r\n"
            s += "y_vel = " + str(v_y) + "\r\n"
            s += "z_vel = " + str(v_z) + "\r\n"
            print s

        # Use the acceleration data to update our velocity values.
        v_x += (data.linear_acceleration.x + x_accel_offset) * delta_t
        v_y += (data.linear_acceleration.y + y_accel_offset) * delta_t
        v_z += (data.linear_acceleration.z + z_accel_offset) * delta_t

        # Transmit new tf data for the position in space.  Set all rotations to 0 since
        # we are not receiving any kind of orientation data from anything.
        br.sendTransform((x_pos, y_pos, z_pos),
                         tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                         rospy.Time.now(),
                         "base_link",
                         "world")

    prev_time_s = current_time_s


def main(args):
    run()


if __name__=="__main__":
    main(sys.argv)


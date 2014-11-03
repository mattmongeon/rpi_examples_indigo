# ROS Indigo on Raspbery Pi

## Introduction

This document explains the steps necessary for installing ROS Indigo on the Raspberry PI and takes it a step farther to give an introduction of how to incorporate some of the Raspberry Pi's GPIO capabilities so that they can be utilized as part of a ROS package.  This documentation is by no means a fully complete resource of all of the Raspberry Pi's capabilities.  However, the hope is that it can provide a good introduction and starting point so that the time spent in the small details can be minimized as much as possible.

The code associated with this document is intended as a launch point and a means of demonstrating how to work with the GPIO on the Raspberry Pi. It is by no means meant to be a complete solution, nor is it guaranteed to be bug free.  Rather it is intended to demonstrate concepts so that it can be expanded to other specific applications.  Each of the code files is written in Python.

## Installing ROS Indigo

To install ROS Indigo, visit this [link](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi).  It will guide the installation process step by step.

There are a few items of note regarding these instructions.  First of all, beware that the installation process will take 8 or more hours to complete.  The installation process will involve compiling many libraries and packages from source, which takes a long time on the Raspberry Pi's hardware.  Secondly, during the installation process you will likely encounter errors as you go.  Ignore these errors and continue on.  When you are finished with all of the steps you will be able to run roscore and other applications.

## Testing with turtlesim

### Running turtlesim for the First Time

### Controlling turtlesim Over SSH

## GPIO Programming

###Overview

The Raspberry Pi has 40 GPIO pins that provide numerous capabilities, including I2C, SPI, and general digital I/O.  A schematic for pins can be found [here](http://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/README.md).

### Controlling an LED



### I2C Interface

####1. Enabling I2C

The Raspberry Pi has a single I2C channel on pins 3 (SDA) and 5 (SCL).  By default the I2C channel is disabled.  To enable it, follow steps 1-4 in the instructions on [this page](http://www.instructables.com/id/Raspberry-Pi-I2C-Python/) with only one item to note.  Once the files have been edited and it is time to run the i2cdetect tool, the identifying number needs to be correct.  The instructions say to use 0, but it could be it needs to be 1.  To verify run the following commands in the terminal

    $ cd /dev
    $ ls | grep i2c

This will list the I2C interfaces on the Raspberry Pi.  The number at the end is the argument to pass to ````i2cdetect````.  For example, if running the command returned

    i2c-1

the ````i2cdetect```` tool should be run with

    i2cdetect -y 1

####2. Writing the Code

Now to the fun part of writing code.  The code provided is written in Python and works with a [LSM303DLHC 3D compass and accelerometer](http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF251940).  The code utilizing it is by no means meant to be a complete solution for this device.  It simply provides an example to demonstrate how to program the I2C interface.  It will only be making full use of the accelerometer capabilities.  The magnetometer can be utilized in a similar fashion.  The file ```lsm303.py``` contains the complete source code for this example.

#####2.1 Importing smbus

The ````smbus```` package contains the classes used for I2C communication.  At the top of the Python script add the line

    import smbus

#####2.2 Creating the SMBus Object

The communication with the I2C bus happens using the SMBus object.  Its constructor requires an index identifying which I2C bus to use.  The following line of code will instantiate a SMBus object that will communicate over bus 1.

    bus = smbus.SMBus(1)

#####2.3 About the SMBus Object

For this example we will be using only two functions from the SMBus class:  ```write_byte_data()``` and ```read_byte_data()```.  Documentation for these and other functions can be found [here](http://wiki.erazor-zone.de/wiki:linux:python:smbus:doc).  The function signature for ```write_byte_data()``` is

    write_byte_data(addr, cmd, val)

where ```addr``` is the address of the I2C component, ```cmd``` is the command to be run or the register to write to, and ```val``` is the value to be written.  The function signature for ```read_byte_data()``` is

   read_byte_data(addr, cmd)

where ```addr``` is the address of the I2C component and ```cmd``` is the command to be run or the register to be read.

#####2.4 Initializing the Accelerometer

After creating the SMBus object, we can use it to configure our accelerometer.  The following code snippet configures the accelerometer to update acceleration data at a rate of 50 Hz and enables all three axes.  It also sets our linear acceleration full scale to +/- 4G.  

    LIN_ACCEL_ADDR = 0x19
    LIN_ACCEL_REG_CTRL_REG1_A = 0x20
    LIN_ACCEL_REG_CTRL_REG4_A = 0x23
    
    bus.write_byte_data(LIN_ACCEL_ADDR, LIN_ACCEL_REG_CTRL_REG1_A, 0x47)
    bus.write_byte_data(LIN_ACCEL_ADDR, LIN_ACCEL_REG_CTRL_REG4_A, 0x90)

#####2.5 Reading the Accelerometer

Once the accelerometer has been initialized, the SMBus object can be used to read data from its registers.  The following code snippet is a helper function that reads data from a given register.

    def read_accel_reg(register):
        return bus.read_byte_data(LIN_ACCEL_ADDR, register)

The next code snippet uses this function to read the raw x-acceleration data and turns it into a signed integer where 0 acceleration corresponds to a 0 value.

    LIN_ACCEL_REG_OUT_X_L_A = 0x28
    LIN_ACCEL_REG_OUT_X_H_A = 0x29
    
    def read_x_accel():
        lo = read_accel_reg(LIN_ACCEL_REG_OUT_X_L_A)
        hi = read_accel_reg(LIN_ACCEL_REG_OUT_X_H_A)
        total = (hi << 8) + lo
        total = convert_twos_complement(total)
        return total

After initially constructing the value total, it is used by the code as an unsigned integer value.  It needs to be turned into a signed integer centered at 0.  This is done by the ```convert_twos_complement()``` function.  Its definition can be found in the source code.

#####2.6 Making Acceleration Value Useful

The ```read_x_accel()``` function can be modified to convert the acceleration value into an acceleration value in meters/second^2.  The following code snippet modifies the previous ```read_x_accel()``` function to convert to m/s^2

    G_RESOLUTION = 32768 / 4
    G_TO_M_S2 = 9.80665
    
    def read_x_accel():
        lo = read_accel_reg(LIN_ACCEL_REG_OUT_X_L_A)
        hi = read_accel_reg(LIN_ACCEL_REG_OUT_X_H_A)
        total = (hi << 8) + lo
        total = convert_twos_complement(total)

        return (float(total) / float(G_RESOLUTION)) * G_TO_M_S2

Here ```G_RESOLUTION``` is a constant that specifies an integer value per g.  In this case, if the accelerometer gives a value of 8192 it corresponds to 1 g, 16384 corresponds to 2 g, etc.  Likewise, -8192 corresponds to -1 g, -16384 corresponds to -2 g, etc.

#####2.7 Using it in ROS

Now that the acceleration values are available they can start being used in ROS.  The following code snippet hows how this can be done.

    import rospy
    from sensor_msgs.msg import Imu
    
    G_RESOLUTION = 32768 / 4
    G_TO_M_S2 = 9.80665
    
    LIN_ACCEL_ADDR = 0x19
    LIN_ACCEL_REG_CTRL_REG1_A = 0x20
    LIN_ACCEL_REG_CTRL_REG4_A = 0x23
    LIN_ACCEL_REG_OUT_X_L_A = 0x28
    LIN_ACCEL_REG_OUT_X_H_A = 0x29
    LIN_ACCEL_REG_OUT_Y_L_A = 0x2A
    LIN_ACCEL_REG_OUT_Y_H_A = 0x2B
    LIN_ACCEL_REG_OUT_Z_L_A = 0x2C
    LIN_ACCEL_REG_OUT_Z_H_A = 0x2D

    bus = smbus.SMBus(1)

    imu_pub = rospy.Publisher("lin_accel", Imu, queue_size=10)

    def run():
        # Initialize/configure the accelerometer
        bus.write_byte_data(LIN_ACCEL_ADDR, LIN_ACCEL_REG_CTRL_REG1_A, 0x47)
        bus.write_byte_data(LIN_ACCEL_ADDR, LIN_ACCEL_REG_CTRL_REG4_A, 0x90)

        # Get the node up and running so it can publish acceleration data.
        rospy.init_node("lsm303", anonymous=True)
        rospy.Timer(rospy.Duration(0.1), timer_callback)
        rospy.spin()

    def timer_callback(event):
        global imu_pub

        # Read our acceleration values.
        x_accel = get_x_accel()
        y_accel = get_y_accel()
        z_accel = get_z_accel()

        # Stuff the acceleration values into an Imu message and send it out.
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.linear_acceleration.x = x_accel
        imu_msg.linear_acceleration.y = y_accel
        imu_msg.linear_acceleration.z = z_accel
        imu_pub.publish(imu_msg)

##Taking it Farther

The capabilities of the Raspberry Pi go further still than what is outlined in this document.  The following pages provide some information on looking into the possibilities:

* SPI - [Intro](http://www.100randomtasks.com/simple-spi-on-raspberry-pi) and [Documentation](http://tightdev.net/SpiDev_Doc.pdf)
* [PWM and TCP interrupts and more](http://pythonhosted.org/RPIO/)

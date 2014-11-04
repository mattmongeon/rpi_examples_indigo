# ROS Indigo on Raspbery Pi

## Introduction

This document explains the steps necessary for installing ROS Indigo on the Raspberry PI and takes it a step farther to give an introduction of how to incorporate some of the Raspberry Pi's GPIO capabilities so that they can be utilized as part of a ROS package.  This documentation is by no means a fully complete resource of all of the Raspberry Pi's capabilities.  However, the hope is that it can provide a good introduction and starting point so that the time spent in the small details can be minimized as much as possible.

The code associated with this document is intended as a launch point and a means of demonstrating how to work with the GPIO on the Raspberry Pi. It is by no means meant to be a complete solution, nor is it guaranteed to be bug free.  Rather it is intended to demonstrate concepts so that it can be expanded to other specific applications.  Each of the code files is written in Python.

## Installing ROS Indigo

To install ROS Indigo, visit this [link](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi).  It will guide the installation process step by step.

There are a few items of note regarding these instructions.  First of all, beware that the installation process will take 8 or more hours to complete.  The installation process will involve compiling many libraries and packages from source, which takes a long time on the Raspberry Pi's hardware.  Secondly, during the installation process you will likely encounter errors as you go.  Ignore these errors and continue on.  When you are finished with all of the steps you will be able to run roscore and other applications.

## Testing with turtlesim

### Running turtlesim for the First Time

<img src="https://raw.githubusercontent.com/mattmongeon/raspberry_pi/master/image/turtle.jpg" alt="alt text" style="width:300px">

### Controlling turtlesim Over SSH

## GPIO Programming

###Overview

The Raspberry Pi has 40 GPIO pins that provide numerous capabilities, including I2C, SPI, and general digital I/O.  Information about the pins can be found [here](http://pi.gadgetoid.com/pinout).

### Blinking an LED

The following example will demonstrate how to make an LED blink at a periodic rate.  The file [led_blink.py](https://github.com/mattmongeon/raspberry_pi/blob/master/src/led_blink.py) contains the source code.

####1. Wiring It Up

The schematic below shows the wiring used for this particular example.

<img src="https://raw.githubusercontent.com/mattmongeon/raspberry_pi/master/image/blink_led.png" alt="alt text" style="width:400px">

####2. Writing the Code

The first thing to do is to initialize the GPIO.  The following code snippet shows how this is done for this particular example.

    import RPi.GPIO as GPIO
    
    def run():
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)

The call to ```GPIO.setmode()``` tells the Raspberry Pi how the pins are going to be referenced.  It will either be set to ```GPIO.BCM``` (for BCM numbering) or ```GPIO.BOARD``` (for Board numbering).  The ```GPIO.setup()``` function initializes a numbered pin as either input (```GPIO.IN```) or output (```GPIO.OUT```).  It also provides allows an initial value for output pins to be set to either ```GPIO.LOW``` or ```GPIO.HIGH```.

After the pin has been initialized it is ready to be used.  The following code snippet expands on the previous one and demonstrates how this is done.

    import rospy
    import RPi.GPIO as GPIO
        
    lit = False
    
    def run():
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)

        rospy.init_node("led_blink", anonymous=False)
        rospy.Timer(rospy.Duration(1.0), timer_callback)
        rospy.spin()
	GPIO.cleanup()

    def timer_callback(event):
        global lit

        # Toggle the light state.
        lit = not lit
        if lit:
            GPIO.output(18, 1)
        else:
            GPIO.output(18, 0)

The ```rospy.Timer``` object is set to fire the ```timer_callback``` once per second.  In this function the on/off state of the LED is toggled, which is accomplished with ```GPIO.output()```.  This function takes in a pin number and a ```1``` or ```0``` to indicate whether the pin is powered or not.  The ```GPIO.cleanup()``` releases the setup for the pins.

###Toggling an LED

This example demonstrates how to take in a button press and use it to toggle the state of an LED.  The full source code can be found in [button_toggle_led.py](https://github.com/mattmongeon/raspberry_pi/blob/master/src/button_toggle_led.py).

####1. Wiring It Up

The schematic below shows the wiring used for this particular example.

<img src="https://raw.githubusercontent.com/mattmongeon/raspberry_pi/master/image/toggle_led.png" alt="alt text" style="width:400px">

####2. Writing the Code

The first thing to do is to initialize the GPIO.  The following code snippet shows how this is done for this particular example.

    import RPi.GPIO as GPIO
    
    def run():
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(23, GPIO.IN)
        
        GPIO.add_event_detect(23, GPIO.RISING, callback=rising_edge_callback, bouncetime=400)

The initialization code in this example adds two additional lines of code compared to the prevoius example.  The call

    GPIO.setup(23, GPIO.IN)

initializes pin 23 to be an input.  This is the pin that will take in the state of the pushbutton.  The last line

    GPIO.add_event_detect(23, GPIO.RISING, callback=rising_edge_callback, bouncetime=400)

sets up event detection based on the Raspberry Pi detecting a rising edge on pin 23, which will be generated by the pushbutton.  The entry

    callback=rising_edge_callback

specifies a callback function that will be fired when a rising edge occurs.  The entry

    bouncetime=400

provides some debouncing time to the entry to eliminate a flood of rising edge triggers.

Now that the code is initializing the pins and watching for a rising edge from the button it is time to make use of it.  The following code snippt it puts it all together.

    import rospy
    import RPi.GPIO as GPIO

    lit = False

    def run():
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(18, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(23, GPIO.IN)

        rospy.init_node("button_toggle_led", anonymous=False)
        
        GPIO.add_event_detect(23, GPIO.RISING, callback=rising_edge_callback, bouncetime=400)

        rospy.spin()
        GPIO.cleanup()

    def rising_edge_callback(channel):
        global lit
        
        lit = not lit
        if lit:
            GPIO.output(18, 1)
        else:
            GPIO.output(18, 0)

## I2C Interface

####1. Enabling I2C

The Raspberry Pi has a single I2C channel on pins 3 (SDA) and 5 (SCL).  By default the I2C channel is disabled.  To enable it, follow steps 1-4 in the instructions on [this page](http://www.instructables.com/id/Raspberry-Pi-I2C-Python/) with only one item to note.  Once the files have been edited and it is time to run the i2cdetect tool, the identifying number needs to be correct.  The instructions say to use 0, but it could be it needs to be 1.  To verify run the following commands in the terminal

    $ cd /dev
    $ ls | grep i2c

This will list the I2C interfaces on the Raspberry Pi.  The number at the end is the argument to pass to ````i2cdetect````.  For example, if running the command returned

    i2c-1

the ````i2cdetect```` tool should be run with

    $ i2cdetect -y 1

The following diagram shows the schematic used for this example

<img src="https://raw.githubusercontent.com/mattmongeon/raspberry_pi/master/image/lsm303.png" alt="alt text" style="width:500px">

####2. Writing the Code

Now to the fun part of writing code.  The code provided is written in Python and works with a [LSM303DLHC 3D compass and accelerometer](http://www.st.com/web/catalog/sense_power/FM89/SC1449/PF251940).  The code utilizing it is by no means meant to be a complete solution for this device.  It simply provides an example to demonstrate how to program the I2C interface.  It will only be making full use of the accelerometer capabilities.  The magnetometer can be utilized in a similar fashion.  The file [lsm303.py](https://github.com/mattmongeon/raspberry_pi/blob/master/src/lsm303.py) contains the complete source code for this example.

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

#####2.6 Making Acceleration Values Useful

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

#####2.8 Publishing the tf Data

Now that the acceleration data is being published it can be viewed in rviz on another machine.  This can be done by subscribing to the ```/lin_accel``` topic, updating the velocity and position of the Raspberry Pi, and publishing new tf data.  The source code for this example can be found in [pi_position.py](https://github.com/mattmongeon/raspberry_pi/blob/master/src/pi_position.py).  Information on tf can be found in [this documentation](http://wiki.ros.org/tf) and [these tutorials](http://wiki.ros.org/tf/Tutorials).

First a subscriber needs to be created.

    rospy.Subscriber("/lin_accel", Imu, received_callback)

Next a ```tf.TransformBroadcaster``` needs to be created.  Once these are finished the acceleration data can be used to generate a new position for the Raspberry Pi.

    def received_callback(data):
        current_time_s = rospy.get_time()
        if prev_time_s != 0.0:
            delta_t = current_time_s - prev_time_s
            
            x_pos += v_x * delta_t
            y_pos += v_y * delta_t
            z_pos += v_z * delta_t

            v_x += data.linear_acceleration.x * delta_t
            v_y += data.linear_acceleration.y * delta_t
            v_z += data.linear_acceleration.z * delta_t

            br.sendTransform((x_pos, y_pos, z_pos),
                tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                rospy.Time.now(),
                "base_link",
                "world")

        prev_time_s = current_time_s

The call to ```sendTransform()``` will transmit the tf data to be used by the rest of ROS, and specifically rviz.  The function ```received_callback()``` will be called any time linear acceleration data is received.

As an additional note, the full code contains offsets that can be applied to the x-, y-, and z-acceleration values to try to get them closer to 0.0 when at rest.  These values naively assume that the accelerometer never rotates or changes orientation.  If this happens, gravity causes the values to shift, which will mess up all of the calculations.  Compensating for gravity requires knowing the orientation (roll, pitch, yaw) of the accelerometer, which is not available in this code.

#####2.9 Preparing to View in rviz

The data is waiting to be viewed in rviz.  Before doing that, the network needs to be setup.  Make sure the Raspberry Pi and remote machine have their network settings set as mentioned above in this documentation so that the remote machine can received acceleration data from the Raspberry Pi.

#####2.10 Viewing it in rviz

Now that the tf data is being published, the movement of the Raspberry Pi can be viewed in rviz once a URDF file has been created.  For the purposes of this example a simple URDF will suffice, such as the one shown in the following snippet (this snippet shows the contents of [accels.urdf](https://github.com/mattmongeon/raspberry_pi/blob/master/accels.urdf)).

    <?xml version="1.0"?>
    <robot name="pi_accels">
      <link name="base_link">
        <visual>
          <geometry>
	    <box size="0.6 0.2 0.1"/>
          </geometry>
        </visual>
      </link>
    </robot>

This file defines a single box link, which is sufficient for showing the Raspberry Pi's position in space.  For more information regarding URDF files see [these tutorials](http://wiki.ros.org/urdf/Tutorials) and [this information](http://wiki.ros.org/urdf/XML) on the XML tags.

Now it is time to launch the node as well as rviz to view the URDF file.  To help with loading the URDF file and launching rviz, the file [pi_in_rviz.launch](https://github.com/mattmongeon/raspberry_pi/blob/master/launch/pi_in_rviz.launch) has been created.  It launches the pi_position.py file and also launches the display.launch file from the urdf_tutorial.  This code snippet displays the contents of pi_in_rviz.launch

    <?xml version="1.0"?>
    <launch>
      <node name="pi_position" pkg="raspberry_pi" type="pi_position.py">
          <param name="_x_offset" value="-0.641"/>
          <param name="_y_offset" value="0.14"/>
          <param name="_z_offset" value="9.74"/>
          <param name="_print_debug" value="False"/>
      </node>
    
      <include file="$(find urdf_tutorial)/launch/display.launch">
          <arg name="model" value="accels.urdf"/>
      </include>
    </launch>

The display.launch file from urdf_tutorial loads a URDF file and launches rviz.  The urdf file is specified by the entry

    <arg name="model" value="accels.urdf"/>

Notice that there are several offset parameters with values set.  These correspond to the acceleration offset values to try to zero the values as much as possible.  These should be tuned for each accelerometer.  The ```_print_debug``` parameter can be used to print accelerations and velocities to the console so that the offsets can be tuned more easily.

To launch everything run the command

    roslaunch raspberry_pi pi_in_rviz.launch

and move the Raspberry Pi around to see it move in rviz.

##Taking the Raspberry Pi Farther

The capabilities of the Raspberry Pi go further still than what is outlined in this document.  The following pages provide some information on looking into the possibilities:

* SPI - [Intro](http://www.100randomtasks.com/simple-spi-on-raspberry-pi) and [Documentation](http://tightdev.net/SpiDev_Doc.pdf)
* [PWM and TCP interrupts and more](http://pythonhosted.org/RPIO/)

# ROS Indigo on Raspbery Pi

## Introduction

This document explains the steps necessary for installing ROS Indigo on the Raspberry PI and takes it a step farther to give an introduction of how to incorporate some of the Raspberry Pi's GPIO capabilities so that they can be utilized as part of a ROS package.  This documentation is by no means a fully complete resource of all of the Raspberry Pi's capabilities.  However, the hope is that it can provide a good introduction and starting point so that the time spent in the small details can be minimized as much as possible.

The code associated with this document is intended as a launch point and a means of demonstrating how to work with the GPIO on the Raspberry Pi. It is by no means meant to be a complete solution, nor is it guaranteed to be bug free.  Rather it is intended to demonstrate concepts so that it can be expanded to other specific applications.  Each of the code files is written in Python.

For this project we have devices including a [Raspberry Pi B+ board](http://www.raspberrypi.org/products/model-b-plus/), a keyboard, a mouse, a HDMI monitor, a LSM303DLHC Ultra-compact high-performance eCompass module: 3D accelerometer and 3D magnetometer [datasheet](http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00027543.pdf) a Raspberry Pi B+ GPIO Breakout Board, and some LEDs, bush buttons, resistors and wire.

The Raspberry Pi B+ board has a SD card slot. With a 8G SD card installed with [Raspbian OS](http://raspbian.org/) and HDMI cable connected with HDMI monitor, you could set up a micro-computer. The Raspberry will start as long as the power is pluged in.

Hint: 
1. When using the terminal in Raspbian OS, some command lines are different with ubuntu 14.04 and the process duration is significantly longer than on intel Core i5 processor, please be more patient.
2. [Screenshots On The Raspbian OS](http://www.raspberrypi-spy.co.uk/2013/10/how-to-take-screenshots-on-the-raspberry-pi/).


## Installing ROS Indigo

To install ROS Indigo, visit this [link](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Indigo%20on%20Raspberry%20Pi).  It will guide the installation process step by step.

There are a few items of note regarding these instructions.  First of all, beware that the installation process will take 8 or more hours to complete.  The installation process will involve compiling many libraries and packages from source, which takes a long time on the Raspberry Pi's hardware.  Secondly, during the installation process you will likely encounter errors as you go.  Ignore these errors and continue on.  When you are finished with all of the steps you will be able to run roscore and other applications.

## Testing with turtlesim

### Running turtlesim for the First Time

When finish the installation of ROS indigo on the Raspberry Pi board, first try to creat a new workspace just like on our computer [Creat a ROS Workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). Then you could try our cute turtlesim node as a test whether our ROS is installed successfully or not [turtlesim](http://wiki.ros.org/turtlesim).

Hint: Here you might encounter some error like "turtlesim_node could not be found". First of all, you should check your ROS environment variables. If the ROS environment is working appropriately, that means your turtlesim package is not installed appropriately. You should try to clone the package [ros tutrial](https://github.com/ros/ros_tutorials) to your workspace.

### Controlling turtlesim Over Network

Then we could try to use turtle_teleop_key to control the motion of the turtle.

Hint: The Raspberry Pi B+ board have 4 USB port could be used with periphal devices like keyboard, usb-key, mouse, web camera and so on. All the devices listed above requires no drivers, they will automatically work with just plug-in.

Furthermore, you could try to control the turtle through the Network with Raspberry Pi. First of all, establish a local network (a router) and connect the Raspberry Pi and your computer with it. First of all, in order to resolve the name of the Raspberry Pi and our own computer, we should install avhi-daemon on Raspberry Pi:

    sudo apt-get install avahi-daemon

Then follow the tutorial of running ROS over a Network, export the Master URI and Hostname on both Raspberry Pi terminal and your own computer:

    $ export ROS_HOSTNAME=localhost
    $ export ROS_MASTER_URI=http://localhost:11311

Then try to control it via the keyboard on your machine (use turtle_teleop_key), you could get the turtle running like following:

<img src="https://raw.githubusercontent.com/mattmongeon/raspberry_pi/master/image/turtle.jpg" alt="alt text" style="width:300px">

### How to Program the Raspberry Pi Hardware

Now that ROS Indigo has been installed on the Raspberry Pi, it is time to start making use of its features.  The following pages demonstrate how to utilize its hardware in the ROS environment.

* The [GPIO](https://github.com/mattmongeon/raspberry_pi/blob/master/gpio_programming.md) page demonstrates how to program the GPIO pins by controlling an LED and reading a pushbutton.
* The [I2C](https://github.com/mattmongeon/raspberry_pi/blob/master/i2c_programming.md) page demonstrates how to read and write over I2C using an accelerometer.  It also demonstrates how to display the Raspberry Pi in rviz as it uses the acceleration data to show its position.

###Taking the Raspberry Pi Farther

The capabilities of the Raspberry Pi go further still than what is outlined in this document.  The following pages provide some information on looking into the possibilities:

* SPI - [Intro](http://www.100randomtasks.com/simple-spi-on-raspberry-pi) and [Documentation](http://tightdev.net/SpiDev_Doc.pdf)
* [PWM and TCP interrupts and more](http://pythonhosted.org/RPIO/)


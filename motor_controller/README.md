# Perceptbot Motor Controller

This directory contains the source code for the Perceptbot motor controller. 

A refactoring of this code is located in the `refactored/` directory.

## How to use these files

Anything you do depends on `roscore`. Unless you are launching the Perceptbot using the `main.launch` file, `roscore` will not be running by default. So open up a terminal and enter: `$ roscore`

## teleop and motor controller

It is possible to use a `teleop` script to control the Perceptbot manually,
using keyboard input. If you'd like to do this, make sure that `roscore` is
running (see above), and then run the motor controller script, using
`roslaunch`.

The motor controller listens for messages over `cmd_vel`. As the `teleop`
script broadcasts over `cmd_vel`, all you now need to do is run the `teleop`
script. To do this, open up another terminal, switch to the
`teleop_twist_keyboard` directory (the location of `teleop_twist_keyboard.py`)
and type

`$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## wheel odometry

Perceptbot wheel odometry is managed by two nodes which broadcast three topics;
 * `wheelencoder.py` broadcasts a separate topic for left and right wheel encoders, `\lwheel` and `\rwheel`
 * `odom_calculator.py` subscribes to both of these topics and uses this information to perform wheel odometry calculations. It broadcasts the transform over tf and also a topic called `wheel_odom`

You can launch all of these nodes from the `main.launch` file.

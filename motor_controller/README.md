# How to use these files

Anything you do depends on `roscore`. So open up a terminal and enter: `$ roscore`

## teleop and motor controller

In a terminal: 

`$ cd ~/overlay_ws/src/teleop_twist_keyboard/` `$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

In another terminal: `$ cd ~/overlay_ws/src/motor_controller/src` `$ python commander.py`

The `teleop_twist_keyboard.py` script is what we are using to pass keyboard
input to cmd\_vel. Output is represented as a Twist object; each Twist object
has two Vector3 fields.  You can instead use the navigation stack to pass
cmd\_vel Twists, the controller.py script will listen to this too.

The Listener object in move.py listens to cmd\_vel and then maps these Twist
values into motor controls

## wheel odometry

Perceptbot wheel odometry is managed by two nodes which broadcast three topics;
 * `wheelencoder.py` broadcasts a separate topic for left and right wheel encoders, `\lwheel` and `\rwheel`
 * `odom_calculator.py` subscribes to both of these topics and uses this information to perform wheel odometry calculations. It broadcasts the transform over tf and also a topic called `wheel_odom`

To run both of these nodes, make sure that `roscore` is first running (see above). Then open a terminal and run `$ python wheelencoder.py`. When the `\lwheel` and `\rwheel` topics are broadcasting, open another terminal and run `$ python odom_calculator.py`.

Of course, you must be in `~/overlay_ws/src/motor_controller/src` in order to be able to run these scripts.

TODO: incorporate launching of these nodes into main.launch (once debugged!). Also incorporate slippage correction data from laser.

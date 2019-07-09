# Refactored motor controller :sparkles:

These directories contain source code and tests for an *untested* refactoring
of the motor controller code (the code contained in the top-level `src`
directory, of which the `refactored` directory is a child. 

The code is *untested* in the sense that it has not yet been used to control a Perceptbot.

## What are these files? Why do they exist?

To understand why we attempted to refactor this code, it is useful to first
know about how the original source code was developed.

The original motor controller was developed in an organic fashion. The first
scripts that were written exercised a basic level of control over the motors;
they were really there so that we could understand how our hardware worked, and
get an idea as to how we could translate between high-level movement commands
and low-level hardware commands. All the first scripts could do was enable and
disable motors, and (after a bit of tinkering) modulate the speed of our
Perceptbot using Pulse Width Modulation signals (PWM).

When we found that the Perceptbot hardware was not sensitive  enough to
respond accurately to cmd\_vel signals, we decided to add in a PID controller. 
This PID model itself then went through several significant iterations e.g. 
we moved from having a single PID controller for the entire motor controller to having models for each wheels, as
each wheel of our Perceptbot implementation exhibited different aberrant
behaviour. We ultimately ended up with PID controllers for each axis of movement.

By the time we were done with the project and ready to show it off, our code
was quite messy. You can see this by taking a look at the two UML diagrams
below: the first shows interaction of key classes
that make up the Perceptbot's motor controller *prior* to refactoring. The
second diagram shows the state of play *post*-refactor.

![Pre-refactor UML](https://github.com/maf2418/Perceptbot/tree/master/images/uml_0.jpeg)
![Post-refactor UML](https://github.com/maf2418/Perceptbot/tree/master/images/uml_1.jpeg)

As you can see from the first diagram, out expressed a number of important principles, but because it
had been designed in an incremental and somewhat meandering fashion, the
important things that it expressed were not expressed in the clearest ways. A
refactor was in order!

In particular, the refactor attempted to address several issues with the original source code;

1. Lack of extensibility. Components abstracted from the hardware
   e.g. the PIDController class, still know a lot about the physical layout of
   the machine. What if somebody wanted to use the code for a Perceptbot with a
   different drive configuration? With the old code, this wouldn't be possible.
   Our code was *fragile*.
2. Long message chains. Partly as a result of (1), if classes at the 'top' of
   the hierarchy wanted information from lower down e.g. odometry data or an
   updated PID model since the last time interval, they had to pass calls via a
   number of other objects before they were able to extract the information
   they required. This further hindered extensibility.
3. Redundancy. The original code demarcated class boundaries in an unnecessary
   fashion: if the encapsulation is designed in such a way that classes at the top
   of the hierarchy need to know low-level information obtained at the bottom
   e.g. from odometry sensors or GPIO output pins, why do they need to be
   separate at all? A more fitting encapsulation of the problem should result
   in classes which are concerned with distinct logical entities. In an ideal
   world, some of these logical entities should map roughly to hardware
   components.

## The design of these files

The new code located in the `src/` directory here reflects a series of design decisions;

1. PIDModel and PIDController classes are merged into a single class which
   manages control and keeps a model for a single motor only (in the original
   code, each PIDController manages a 'left' and a 'right' model).
2. Create an abstract DriveChain interface. Subclass this interface to create
   concrete instances which reflect the chosen hardware. For instance, with our two-wheel
   drive hardware, we created a TwoWheelDriveChain class. The
   TwoWheelDriveChain broadcasts PID-related messages over ROS and knows about
   the physical layout of the robot e.g. the number of wheels. It loads encoder settings,
   exerts control by updating the PID model, and polls each PIDController for
   data it needs to exert control.
3. Apply a *strategy pattern* to the OdometryPublisher class, extracting an
   OdometryCalculator interface which allows us to defer specialisation of the
   Odometry until runtime. An OdometryPublisher is now built with a calculator
   which corresponds to the Perceptbot hardware; future users can define
   calculators in an *extensible* and *modular* fashion which correspond to
   the hardware they are using.

## Improvements

As mentioned above, this code is *untested*. It has not been used to
actually control a Perceptbot, and as such it represents an ideal version of the code that
we'd like to see in production. We're including it here because we believe that
a future user may well be interested in it.

There are several places where we can recommend changes or improvements;

 * Explore further abstraction away from the TwoWheelDriveChain into the
   DriveChain superclass.
 * The MotionController still knows about the layout of the robot in its
   `load_drivechains()` method. This is not ideal as it breaks one of the goals
   of the refactor (stated above). A good way to solve this problem would be to
   pass an *interface* to the `load_drivechains()` method, deferring
   specialisation to the `drivechain` module.
 * Improve the code in the `odometry2` module.
 * Users still need to create their own Mover class to match their hardware
   (minimally this involves modifying the global constants in `src/mover.py` to
   reflect which GPIO pins are hooked up -- if using a Raspberry Pi). Is there
   a way we could do better?
 * Try and actually run the code! :wink:

# Refactored motor controller :sparkles:

These directories contain source code and tests for an *untested* refactoring
of the motor controller code. It is *untested* in the sense that it has not
been used to control a Perceptbot; but perhaps it could be, in future, given a
little bit more love and care... :sparkling_heart:

## What are these files? Why do they exist?

Here are two rough UML diagrams. The first shows interaction of key classes
that make up the Perceptbot's motor controller *prior* to refactoring. The
second diagram shows the state of play *post*-refactor.

![Pre-refactor UML](https://github.com/maf2418/Perceptbot/images/uml_0.jpeg)

![Post-refactor UML](https://github.com/maf2418/Perceptbot/images/uml_1.jpeg)

The original motor controller was designed in an organic fashion. The first
scripts that were written exercised a basic level of control over the motors.
PWM control was then added, followed shortly thereafter by PID controler (when
it was found that the Perceptbot hardware was not sensitive/expensive enough to
respond accurately to cmd\_vel signals). The PID model itself went through
several significant refinements e.g. we moved from having a single PID
controller for the entire motor controller to having models for each wheels --
each wheel of our Perceptbot implementation exhibited different aberrant
behaviour -- and having controllers for each axis of movement.

By the time we were done with the project and ready to show it off, our code
was quite messy. It expressed a number of important principles, but because it
had been designed in an incremental and somewhat meandering fashion, the
important things that it expressed were not expressed in the best ways. A
refactor was in order!

In particular, the refactor attempted to address several issues with the original source code;

1. Lack of extensibility. Components abstracted from the hardware
   e.g. the PIDController class, still know a lot about the physical layout of
   the machine. What if somebody wanted to use the code for a Perceptbot with a
   different drive configuration? With the old code, this wouldn't be possible.
2. Long message chains. Partly as a result of (1), if classes at the 'top' of
   the hierarchy wanted information from lower down e.g. odometry data or an
   updated PID model since the last time interval, they had to pass calls via a
   number of other objects before they were able to extract the information
   they required.
3. Redundancy. The original code demarcated class boundaries in an unnecessary
   fashion: if the encapsulation is designed in such a way that classes at the top
   of the hierarchy need to know low-level information obtained at the bottom
   e.g. from odometry sensors or GPIO output pins, why do they need to be
   separate at all? A more fitting encapsulation of the problem should result
   in classes which are concerned with distinct logical entities.

## The design of these files

The new code located in the `src/` directory reflects a series of design decisions;

1. PIDModel and PIDController classes are merged into a single class which
   manages control and keeps a model for a single motor only (in the original
   code, each PIDController manages a 'left' and a 'right' model).
2. Create an abstract DriveChain interface. Subclass this interface to create
   concrete instances which reflect the chosen hardware e.g. with our two-wheel
   drive hardware, we created a TwoWheelDriveChain class. The
   TwoWheelDriveChain broadcasts PID-related messages over ROS and knows about
   the layout of the robot e.g. number of wheels. It loads encoder settings,
   exerts control by updating the PID model, and polls each PIDController for
   data it needs to exert control.
3. Apply a *strategy pattern* to the OdometryPublisher class, extracting an
   OdometryCalculator interface which allows us to defer specialisation of the
   Odometry until runtime. An OdometryPublisher is now built with a claculator
   which corresponds to the Perceptbot hardware; future users can define
   calculators in an *extensible* and *modular* fashion for their own brand of
   hardware (see the UML diagrams above for a clear representation).

## The catch; or, possible improvements

As mentioned above, this code is *untested*. It has not (yet) been used to
actually control a Perceptbot; it represents an ideal version of the code that
we'd like to see in production. 

There are several places we can recommend changes or improvements;

 * Explore further abstraction away from the TwoWheelDriveChain into the
   abstract superclass
 * Try and actually run the code! :wink:

## TODO

 * re-add tests.sh script from the Gitlab repo

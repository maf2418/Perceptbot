#!/usr/bin/env python

import nodecheckers
nodecheckers.setenvironment_vars()

from actionlib import SimpleActionServer
from actionlib.msg import TestAction
import rospy
import smach
from rospy import sleep
from sm import sm
import smach_ros

# state1: NAV in navigation mode, has a home and has a destination
# outcomes: Pending, Succeeded, Not_active

# state2: IDLE in static state, waiting for a command, is in this state before Nav and after succeeding its journey
# outcomes: Pending, Succeeded, Not_active

# Scenario:
# Idle_state (listening for useful input = destination)
# Given destination
# Nav_state (navigating and calculating distance from its current_pos and dest)
# Idle_state (switch upon arrival)

# graph:
# IDLE -> MOVING
# MOVING -> MOVING_TURNING
# MOVING_TURNING -> MOVING_TURNING
# MOVING_TURNING -> IDLE

# define state NAV
class NAV(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['static'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Nav')
        sleep(2)
        if self.counter < 3:
            self.counter += 1
            return 'static' # pending
        else:
            return 'static' # succeeded


# define state IDLE
class IDLE(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['active'])

    def execute(self, userdata):
        sleep(2)

        # if userdata -> destination): then set tp pending
        rospy.loginfo('Executing state Idle')
        return 'active'

# action server
# Create a trivial action server
class TestServer:
    def __init__(self,name):
        self._sas = SimpleActionServer(name,
                TestAction,
                execute_cb=self.execute_cb)

    def execute_cb(self, msg):
        if msg.goal == 0:
            self._sas.set_succeeded()
        elif msg.goal == 1:
            self._sas.set_aborted()

def create_StMachine():

    rospy.init_node('sm_template')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['static', 'active'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Nav', NAV(),
                               transitions={'static': 'Idle'})

        smach.StateMachine.add('Idle', IDLE(),
                               transitions={'active': 'Nav'})

    return sm

# main
def main():
    # Execute SMACH plan
    sm = create_StMachine()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

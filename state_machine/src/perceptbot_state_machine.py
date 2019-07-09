import time
import subprocess
import os
import signal

# ROS libs
import actionlib
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslaunch
import rospy

# State Machine
import smach
from smach import State, StateMachine


def launch_from_file(launchfile):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launchfile])

    launch.start()

    return launch

# Idle state
class Idle(smach.State):
    def __init__(self, outcomes=['explore', 'stop']):
        self._outcomes = outcomes
        self._input_keys = []
        self._output_keys = ['class_label']
        self._target_class_label = 'laptop'

    def execute(self, userdata):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        rospy.init_node('statemachine_node', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        for i in range(2):
            twist_msg = Twist()
            rospy.loginfo(twist_msg)
            pub.publish(twist_msg)
            rate.sleep()

        message = rospy.wait_for_message("statemachine_message", String, timeout=None)

        if message.data == "e":
            return 'explore'

        else:
            return 'stop'

# Exploration state
class Explore(smach.State):
    def __init__(self, outcomes=['stop']):
        self._outcomes = outcomes
        self._input_keys = []
        self._output_keys = []

    def execute(self, userdata):
         # launch exploration
         exp = launch_from_file("/home/pi/overlay_ws/src/m-explore/explore/launch/explore.launch")

         time.sleep(5)
         message = rospy.wait_for_message("statemachine_message", String, timeout=None)

         action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
         action_client.wait_for_server()
         action_client.cancel_all_goals()

         exp.shutdown()

         return 'stop'

def main():
    sm = StateMachine(outcomes=['kill'])

    with sm:
        StateMachine.add('IDLE', Idle(),
                            transitions={'explore': 'EXPLORE', \
                                         'stop': 'kill'})

        StateMachine.add('EXPLORE', Explore(), \
                            transitions={'stop': 'IDLE'})

    StateMachine.set_initial_state(sm, ['IDLE'])

    result = sm.execute()
    rospy.loginfo(result)

if __name__ == "__main__":
    main()

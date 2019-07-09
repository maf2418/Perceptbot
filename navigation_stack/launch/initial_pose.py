import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
def setInitialPose():
	pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size =10)
	rospy.init_node('setInitialPose', anonymous = True)
	rate = ropspy.Rate(10)
	while not rospy.is_shutdown():		
		initpose = PoseWithCovarianceStamped()
                initpose.pose.pose.position.x = 100.0
                initpose.pose.pose.position.y = 100.0 
                initpose.pose.pose.position.z = 0.0 
                rospy.loginfo(initpose)
                pub.publish(initpose)
                rate.sleep()


def main():
	pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size =10)
        rospy.init_node('setInitialPose', anonymous = True)
	rate = rospy.Rate(10)
	for i in range(100):
		initpose = PoseWithCovarianceStamped()
                initpose.pose.pose.position.x = 100.0
                initpose.pose.pose.position.y = 100.0
                initpose.pose.pose.position.z = 0.0
                rospy.loginfo(initpose)
                pub.publish(initpose)
                rate.sleep()

if __name__  == "__main__":
	main()

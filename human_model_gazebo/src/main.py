import rospy

# transformations for extended frame
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, Point, TransformStamped

# get moveit to move arm
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from tf.transformations import quaternion_from_euler
import moveit_utils as mv

if __name__=="__main__":

	rospy.loginfo("initializing..")

	# getting moveit
	robot = moveit_commander.RobotCommander() 
	scene = moveit_commander.PlanningSceneInterface()
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
	                                           moveit_msgs.msg.DisplayTrajectory,
	                                           queue_size=20)

    
	## Calling the transform listeners and broadcaster to project wrt robot base
	#br = tf.TransformBroadcaster()
	#t = tf.TransformListener(True, rospy.Duration(10.0))

	while not rospy.is_shutdown(): 
		try:
			goto_final_object_pos = PoseStamped()
			goto_final_object_pos.header.frame_id = "/base_human_link"
			goto_final_object_pos.pose.position.x = 0.395
			goto_final_object_pos.pose.position.y = -0.125
			goto_final_object_pos.pose.position.z = 0.129#-0.10
			goto_final_object_pos.pose.orientation.x = -0.683
			goto_final_object_pos.pose.orientation.y = 0.183
			goto_final_object_pos.pose.orientation.z = 0.182
			goto_final_object_pos.pose.orientation.w = 0.684


	# goto_final_object_pos.pose.position.x = 0.295
	# goto_final_object_pos.pose.position.y = -0.125
	# goto_final_object_pos.pose.position.z = 0.329#-0.10
	# goto_final_object_pos.pose.orientation.x = -0.683
	# goto_final_object_pos.pose.orientation.y = 0.183
	# goto_final_object_pos.pose.orientation.z = 0.182
	# goto_final_object_pos.pose.orientation.w = 0.684


	# mv.move_right_arm([-0.44977258103904316, 0.02834543673116714, 0.056921676051430406, -0.20318100829739133])
			mv.move_right_arm(goto_final_object_pos)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

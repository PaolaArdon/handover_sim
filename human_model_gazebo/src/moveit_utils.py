# get moveit to move arm
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker



from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from geometry_msgs.msg import PoseStamped

#import rospy
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK

def move_right_arm(pose_3d):
        
    group = moveit_commander.MoveGroupCommander("sim_human_arm_r")
    group.set_max_velocity_scaling_factor(1.0) #0.03
    group.set_max_acceleration_scaling_factor(1.0)

    # group.set_workspace([-20,-20,-20,20,20,20])
    group.set_workspace([0.39,-0.12,0.12,0.4,-0.13,0.13])

    #group.set_start_state_to_current_state()


    joint_state = JointState()
    joint_state.header.frame_id = '/base_human_link' #Header()
    #joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['bodyright_shoulderright_joint', 'shoulderright_armright1_joint', 'armright1_armright2_joint', 'armrighht3_handright_joint']
    joint_state.position = [0.0,0.0,0.0,0.0]
    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = joint_state
    group.set_start_state(moveit_robot_state)


    group.set_num_planning_attempts(10)


    group.set_goal_joint_tolerance(1.0)
    group.set_goal_orientation_tolerance(1.0)
    group.set_goal_position_tolerance(1.0)
    group.set_goal_tolerance(1.0)

    group.set_random_target()
    group.get_current_pose(end_effector_link = 'handright_link')
    #traj = group.plan()




    # robot_pose = PoseStamped()
    # robot_pose = group.get_current_joint_values()

    # print pose_3d.pose
    # waypoints = []
    # waypoints.append(pose_3d.pose)

    # eef_step = 0.01
    # jump_threshold = 0
    # trajectory = group.compute_cartesian_path(waypoints, eef_step, jump_threshold, avoid_collisions = False)
    # print trajectory[0].joint_trajectory.points[0].positions


    # #group.get_random_pose("sim_hand_palm")
    # traj = group.plan(pose_3d.pose)

    # plotTrajectory(traj.joint_trajectory,"handover_trajectory_pub",1,0,0)

    #group.execute(traj)
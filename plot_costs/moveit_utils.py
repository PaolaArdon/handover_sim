# get moveit to move arm
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
#import pr2_controllers_msgs.msg

def move_torso(robot):
    tor = moveit_commander.MoveGroupCommander("torso")
    tor.set_pose_reference_frame("base_link")
    tor.go([0.2])

def home_left_arm(robot):

    mgc = moveit_commander.MoveGroupCommander("left_arm")
    mgc.get_current_joint_values()
    jv = mgc.get_current_joint_values()
    jv[0]=1.0

    mgc.set_joint_value_target(jv)
    mgc.plan()
    p = mgc.plan()
    mgc.execute(p)

def move_right_arm(robot, pose_3d):
        
    group = moveit_commander.MoveGroupCommander("right_arm")
    group.set_max_velocity_scaling_factor(0.1) #0.03
    group.set_max_acceleration_scaling_factor(0.1)

    print pose_3d.pose

    traj = group.plan(pose_3d.pose)

    print traj

    # plotTrajectory(traj.joint_trajectory,"handover_trajectory_pub",1,0,0)

    group.execute(traj)

    print 'after execute'


def home_head():
    mgc = moveit_commander.MoveGroupCommander("head")
    jv = mgc.get_current_joint_values()
    jv[1] = 0.5
    mgc.set_joint_value_target(jv)
    p = mgc.plan()
    mgc.execute(p)

def home_right_arm(robot):
	
    mgc = moveit_commander.MoveGroupCommander("right_arm")
    jv = mgc.get_current_joint_values()
    jv[0] = -1

    mgc.set_joint_value_target(jv)
    p = mgc.plan()
    mgc.execute(p)

# def open_right_grip(pub):
#     msg = pr2_controllers_msgs.msg.Pr2GripperCommand()
#     msg.max_effort = -1
#     msg.position = .06
#     pub.publish(msg)
#     time.sleep(4)
#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIK

def main():
    print "hello"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("mwe")

    compute_ik_srv = rospy.ServiceProxy("/compute_ik", GetPositionIK)
    rospy.wait_for_service("/compute_ik")

    arm_move_group = moveit_commander.MoveGroupCommander("sim_human_arm_r")
    robot_commander = moveit_commander.RobotCommander()

    pose = PoseStamped()
    pose.header.frame_id = "handright_link"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = 0.295
    pose.pose.position.y = -0.125
    pose.pose.position.z = 0.329#-0.10
    pose.pose.orientation.x = -0.683
    pose.pose.orientation.y = 0.183
    pose.pose.orientation.z = 0.182
    pose.pose.orientation.w = 0.684

    pose.pose.position.x = 0.389
    pose.pose.position.y = -0.187
    pose.pose.position.z = 0.310#-0.10
    pose.pose.orientation.x = 0.433
    pose.pose.orientation.y = 0.567
    pose.pose.orientation.z = 0.257
    pose.pose.orientation.w = 0.652

    pose.pose.position.x = 0.325
    pose.pose.position.y = -0.125
    pose.pose.position.z = 0.169#-0.10
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.866
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 0.5

    req = GetPositionIKRequest()
    req.ik_request.group_name = "sim_human_arm_r"
    req.ik_request.robot_state = robot_commander.get_current_state()
    req.ik_request.avoid_collisions = True
    #req.ik_request.ik_link_name = arm_move_group.get_end_effector_link()
    req.ik_request.pose_stamped = pose
    k = compute_ik_srv(req)
    
    print k
    #print k.error_code.val == 1

    #arm_move_group.set_pose_target(pose)
    #print arm_move_group.go()

main()
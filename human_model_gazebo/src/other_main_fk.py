#!/usr/bin/env python
import sys
import rospy
import moveit_commander
# from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFKRequest, GetPositionFK

#extract data to plot outise
import pickle
import random

def main():
    print "hello"

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("mwe")

    compute_fk_srv = rospy.ServiceProxy("/compute_fk", GetPositionFK)
    rospy.wait_for_service("/compute_fk")

    arm_move_group = moveit_commander.MoveGroupCommander("sim_human_arm_r")
    robot_commander = moveit_commander.RobotCommander()

    fk_request = GetPositionFKRequest()
    fk_request.header.frame_id = "base_human_link"
    fk_request.header.stamp = rospy.Time.now()

    fk_request.fk_link_names = ['sim_hand_palm']
    fk_request.robot_state.joint_state.name = ['bodyright_shoulderright_joint', 'shoulderright_armright1_joint', 'armright1_armright2_joint', 'armright2_armright3_joint', 'armrighht3_handright_joint']
    # fk_request.robot_state.joint_state.header = fk_request.header
    
    

    with open("sampling_poses.txt", "w") as txt_file:

        for i in range(15*15*15*2):
            bodyright = random.uniform(-0.60, 0.60)
            shoulderright = random.uniform(-0.60, 0.60)
            armright1 = random.uniform(-0.60, 0.60)
            armright2 = random.uniform(-0.60, 0.60)
            armright3 = random.uniform(-3.14, 3.14)

            fk_request.robot_state.joint_state.position = [bodyright, shoulderright, armright1, armright2, armright3] 

            k = compute_fk_srv(fk_request)

            x = k.pose_stamped[0].pose.position.x
            y = k.pose_stamped[0].pose.position.y
            z = k.pose_stamped[0].pose.position.z

            alpha = k.pose_stamped[0].pose.orientation.x
            beta = k.pose_stamped[0].pose.orientation.y
            gamma = k.pose_stamped[0].pose.orientation.z
            w = k.pose_stamped[0].pose.orientation.w
            
            txt_file.write(str(fk_request.robot_state.joint_state.position)+" "+
                str([x, y, z])+" "+
                str([alpha, beta, gamma, w])+ "\n")
            #print (k.pose_stamped[0].pose.position)
    
main()



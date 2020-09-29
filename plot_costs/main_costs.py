#Plotting the set of cubes in RVIZ
import rospy

from interactive_markers.interactive_marker_server import *

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
import plot_utils as plt
import moveit_utils as mv


if __name__=="__main__":
    rospy.init_node("cube")
    server = InteractiveMarkerServer("cube")
    rospy.loginfo("initializing..")
    object_positions = plt.makeCube(server)
    #print(object_positions)   
    server.applyChanges()

    # getting moveit
    robot = moveit_commander.RobotCommander() 
    scene = moveit_commander.PlanningSceneInterface()
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    #mv.open_right_grip(display_trajectory_publisher)
    mv.move_torso(robot)
    mv.home_left_arm(robot)
    # mv.home_right_arm(robot)
    mv.home_head()

    ## Calling the transform listeners and broadcaster to project wrt robot base
    br = tf.TransformBroadcaster()
    t = tf.TransformListener(True, rospy.Duration(10.0))
    goto_final_object_pos =  PoseStamped()
    while not rospy.is_shutdown(): 
        try:
            final_object_pose_3D=plt.plot_object_pose(object_positions)

            br.sendTransform((final_object_pose_3D.pose.position.x, final_object_pose_3D.pose.position.y, final_object_pose_3D.pose.position.z),
                        (final_object_pose_3D.pose.orientation.x, final_object_pose_3D.pose.orientation.y, final_object_pose_3D.pose.orientation.z, final_object_pose_3D.pose.orientation.w),
                        rospy.Time.now(),
                        "final_object_pose",
                        "sim_hand_palm") #/sim_hand_palm when dealing with simulation #hand_pose_link when dealing with the real human

            ## getting a transform to locate the arm wrt the new object pose
            (trans,rot) = t.lookupTransform('/object','/r_gripper_r_finger_tip_link', rospy.Time(0)) #r_gripper_r_finger_tip_link     #r_wrist_flex_link      
                                  
            goto_final_object_pos.header.frame_id = "/final_object_pose"
            goto_final_object_pos.pose.position.x = trans[0]#-0.10
            goto_final_object_pos.pose.position.y = trans[1]+0.25
            goto_final_object_pos.pose.position.z = trans[2]#-0.10

            q = quaternion_from_euler(0,  -3.00, 1.5707) # 0 -3.14, 1.5707
            goto_final_object_pos.pose.orientation.x  = q[0]
            goto_final_object_pos.pose.orientation.y  = q[1]
            goto_final_object_pos.pose.orientation.z  = q[2]
            goto_final_object_pos.pose.orientation.z  = q[3]


            print 'found where to handle over the object'

            goto_final_object_pos = t.transformPose('base_link', goto_final_object_pos) 
            print 'after transform'           
            mv.move_right_arm(robot, goto_final_object_pos)
            
            break
            

            consider_cost = raw_input("Consider cost? (yes/no): ")
            if consider_cost == 'yes':

                goto_final_object_pos = t.transformPose('base_link', goto_final_object_pos)            
                mv.move_right_arm(robot, goto_final_object_pos)            
                g = raw_input("Arrived to pose, should I go closer or farther away?: ") 

                if g == 'closer':
                    goto_final_object_pos.pose.position.x = trans[0]#-0.10
                    goto_final_object_pos.pose.position.y = trans[1]+0.20
                    goto_final_object_pos.pose.position.z = trans[2]-0.10
                    mv.move_right_arm(robot, goto_final_object_pos)

                elif g== 'farther':
                    goto_final_object_pos.pose.position.x = trans[0]#-0.10
                    goto_final_object_pos.pose.position.y = trans[1]
                    goto_final_object_pos.pose.position.z = trans[2]
                    mv.move_right_arm(robot, goto_final_object_pos)
                else:
                    ## OPEN GRIPPER
                    print('open gripper')
                    mv.home_right_arm(robot)
            else:
                no_cost_pose =  PoseStamped()
                no_cost_pose.header.frame_id = "/base_link"
                no_cost_pose.pose.position.x = 0.4
                no_cost_pose.pose.position.y = -0.2
                no_cost_pose.pose.position.z = 0.8
                
                q = quaternion_from_euler(3.14,  0, 0)
                no_cost_pose.pose.orientation.x  = q[0]
                no_cost_pose.pose.orientation.y  = q[1]
                no_cost_pose.pose.orientation.z  = q[2]
                no_cost_pose.pose.orientation.z  = q[3]
                mv.move_right_arm(robot, no_cost_pose)
                print('open gripper')
                      
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

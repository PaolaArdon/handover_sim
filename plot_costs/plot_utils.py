#Plotting the set of cubes in RVIZ
import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from math import sqrt
import numpy as np
import math
import costs as c

from geometry_msgs.msg import PoseStamped, Pose, Point, TransformStamped

# get input
import argparse

#extract data to plot outise
import pickle


from tf.transformations import quaternion_from_euler

positions = list()

pub_object_pose_3D = rospy.Publisher("/desired_object_pose_3D", PoseStamped,queue_size=10)
    
def processFeedback( feedback ):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        #compute difference vector for this cube
        x = feedback.pose.position.x
        y = feedback.pose.position.y
        z = feedback.pose.position.z
        index = int(feedback.marker_name)
        #print("feedbackPose", feedback.pose)

        if index > len(positions):
            return
    
        dx = x - positions[index][0]
        dy = y - positions[index][1]
        dz = z - positions[index][2]

        # move all markers in that direction
        for i in range(len(positions)):
            (mx, my, mz) = positions[i]
            d = sqrt(sqrt((x - mx)**2 + (y - my)**2)**2 + (z-mz)**2)
            positions[i][0] += dx
            positions[i][1] += dy
            positions[i][2] += dz

            if i == index:
              rospy.loginfo( d )
              positions[i][0] = x
              positions[i][1] = y
              positions[i][2] = z

            pose = geometry_msgs.msg.Pose()
            pose.position.x = positions[i][0]
            pose.position.y = positions[i][1]
            pose.position.z = positions[i][2]

            server.setPose( str(i), pose )
        server.applyChanges()

def object_pose_rviz(marker_position):
    object_pose_3D = PoseStamped()
    object_pose_3D.header.frame_id = "/sim_hand_palm" #/sim_hand_palm when dealing with simulated human #/hand_pose_link when dealing with real human hand
    object_pose_3D.header.stamp = rospy.Time.now()

   
    ## Getting object orientation towards hand palm
    y_angle = math.atan2(marker_position[0],marker_position[2]) # x,z
    z_angle = math.atan2(marker_position[0],-1*marker_position[1])
    x_angle = math.atan2(marker_position[1],-1*marker_position[2])
   
    q = quaternion_from_euler(x_angle-0.9853, y_angle, z_angle) #3.14159,  0, 0 # -0.7853, 0, -1.57 ##0.9853 CONSIDERING GRIPPER POSE
    
    object_pose_3D.pose.position.x = marker_position[0]
    object_pose_3D.pose.position.y = marker_position[1]
    object_pose_3D.pose.position.z = marker_position[2]
    object_pose_3D.pose.orientation.x = q[0]
    object_pose_3D.pose.orientation.y = q[1]
    object_pose_3D.pose.orientation.z = q[2]
    object_pose_3D.pose.orientation.w = q[3]

    
    pub_object_pose_3D.publish(object_pose_3D)
    
    return object_pose_3D

def plot_object_pose(marker_position):
    #for i in range(len(marker_position)):  
    final_object_pose_3D = object_pose_rviz(marker_position[7]) #6,7,8 # 20 for no cost plot

    ######################################
    with open("object_safety_poses.txt", "w") as txt_file:
        for i in range(len(marker_position)):
            object_for_effort_pose = object_pose_rviz(marker_position[i])
            #print(object_for_effort_pose.position, object_for_effort_pose.orientation)

            txt_file.write(str([object_for_effort_pose.pose.position.x, object_for_effort_pose.pose.position.y, object_for_effort_pose.pose.position.z,
                            object_for_effort_pose.pose.orientation.x, object_for_effort_pose.pose.orientation.y, object_for_effort_pose.pose.orientation.z, 
                            object_for_effort_pose.pose.orientation.w])+"\n")

    ######################################

    return final_object_pose_3D


def makeBoxControl( msg, dist_marker_hand,cost):
    #print(red, blue, green)
    control = InteractiveMarkerControl()
    control.always_visible = True
    
    control.orientation_mode = InteractiveMarkerControl.FIXED
    control.interaction_mode = InteractiveMarkerControl.MOVE_3D

    control.independent_marker_orientation = True

    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale
    marker.scale.y = msg.scale
    marker.scale.z = msg.scale

    if (cost == '-r'):
        ##Plot colours for reachability:  
        if (dist_marker_hand < 0.15):
            marker.color.r = 0.35
            marker.color.g = 0.25
            marker.color.b = 0.8-dist_marker_hand

        elif (dist_marker_hand > 0.15 and  dist_marker_hand < 0.25):
            marker.color.r = dist_marker_hand
            marker.color.g = 0.35+dist_marker_hand
            marker.color.b = 0.15

        else:     
            marker.color.r = 0.35+dist_marker_hand
            marker.color.g = 0.25
            marker.color.b = 0.15
        marker.color.a = 0.25
    elif (cost == '-s'):
        ##Plot colours for safety
        if (dist_marker_hand < 0.15):
            marker.color.r = 0.85
            #marker.color.r = 0.35+dist_marker_hand
            marker.color.g = 0.15
            marker.color.b = 0.10
    
        elif (dist_marker_hand > 0.15 and  dist_marker_hand < 0.25):
            marker.color.r = msg.pose.position.x#dist_marker_hand
            marker.color.g = msg.pose.position.y+dist_marker_hand #0.35+dist_marker_hand
            marker.color.b = msg.pose.position.z#0.15

        elif(dist_marker_hand > 0.4 and  dist_marker_hand < 0.8): #>0.4 <0.8
            marker.color.r = msg.pose.position.x + dist_marker_hand #dist_marker_hand           
            marker.color.g = msg.pose.position.y#0.15
            marker.color.b = msg.pose.position.z#0.10            
        else:            
            # marker.color.r = msg.pose.position.x#0.35
            # marker.color.g = msg.pose.position.y#0.25
            # marker.color.b = msg.pose.position.z+dist_marker_hand #0.25+dist_marker_han
        # marker.color.a = 0.25
            marker.color.r = dist_marker_hand
            marker.color.g = 0.35+dist_marker_hand
            marker.color.b = 0.15
            marker.color.a = 0.65
            
    else:
        if (dist_marker_hand < 0.15):
            marker.color.r = 0.25
            marker.color.g = 0.15
            marker.color.b = 0.10

        elif (dist_marker_hand > 0.16 and  dist_marker_hand < 0.165):
            marker.color.r = dist_marker_hand
            marker.color.g = 0.35+dist_marker_hand
            marker.color.b = 0.15
            marker.color.a = 0.65

        else:            
            marker.color.r = 0.25
            marker.color.g = 0.15
            marker.color.b = 0.10
        
           
   
    control.markers.append( marker )
    msg.controls.append( control )

    return control

def makeCube(server):
    
    args = choose_cost()
    side_length = 15 #number of cubes to form the volume
    step = 0.03#1.0/side_length
    count = 0

    list_most_reachable_positions = []
    list_safest_positions = []
    list_optimal_positions = []
    
    #with open("overall_cost.txt", "w") as txt_file:
    for i in range(side_length):
        x = step * i-((side_length-1)*step/2)
        for j in range(side_length):
            y = step * j -((side_length-1)*step/2)
            for k in range(side_length):
                z = step * k-((side_length-1)*step/2)
                marker = InteractiveMarker()
                marker.header.frame_id = "/sim_hand_palm" #/sim_hand_palm when dealing with simulated human #/hand_pose_link when dealing with real human hand
                marker.scale = step

                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = z
                                
                dist_marker_hand_reach, most_reachable = c.reachability_cost(marker)
                list_most_reachable_positions.append(most_reachable)
                #dist_marker_hand_rech_list.append(dist_marker_hand_reach)

                dist_marker_hand_safety, safest_positions = c.safety_cost (marker)
                list_safest_positions.append(safest_positions)

                
                #positions.append( [x,y,z] )

                marker.name = str(count)

                if args.reachability:
                    makeBoxControl(marker,dist_marker_hand_reach,"-r")

                    #txt_file.write(str(marker.pose.position.x)+" "+str(marker.pose.position.y)+" "+ 
                    #    str(marker.pose.position.z)+" "+str(dist_marker_hand_reach) + "\n")
                elif args.safety:
                    makeBoxControl(marker,dist_marker_hand_safety, "-s")
                    
                    #txt_file.write(str(marker.pose.position.x)+" "+str(marker.pose.position.y)+" "+ 
                    #    str(marker.pose.position.z)+" "+str(dist_marker_hand_safety) + "\n")
                else:
                    if ((dist_marker_hand_reach > 0.09 and  dist_marker_hand_reach < 0.11) and (dist_marker_hand_safety > 0.09 and  dist_marker_hand_safety < 0.11)): 
                        dist_marker_hand = dist_marker_hand_reach
                        list_optimal_positions.append([x,y,z])

                    else:
                        dist_marker_hand = dist_marker_hand_safety

                    makeBoxControl(marker,dist_marker_hand, "-g")

                server.insert( marker, processFeedback )
                count += 1

    #print (list_safest_positions)
    if args.reachability:
        object_positions = c.filtering_optimal_costs(list_most_reachable_positions)
        #return list_most_reachable_positions
    elif args.safety:
        object_positions = c.filtering_optimal_costs(list_safest_positions)
        #return list_safest_positions
    else:
        object_positions = list_optimal_positions

    return object_positions


def choose_cost():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--reachability", help="calculate reachability cost", action="store_true")
    parser.add_argument("-s", "--safety", help="calculate safety cost", action="store_true")
    args = parser.parse_args()
    return args

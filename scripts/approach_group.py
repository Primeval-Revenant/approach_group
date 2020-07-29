#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray

from matplotlib import pyplot as plt
from algorithm import SpaceModeling

import math

import copy

import tf

def euclidean_distance(x1, y1, x2, y2):
    """Euclidean distance between two points in 2D."""
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

def approach_group(data,pos, ori):
   
    group = []
    if not data.poses:
        group = []
    else:
        for pose in data.poses:

            rospy.loginfo("Person Detected")
            
            quartenion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quartenion)

            pose_person = [pose.position.x, pose.position.y,yaw]
            

            group.append(pose_person)
    
    if group:
        aux_group = copy.deepcopy(group)
        groups = [aux_group]
        for gp in groups:
            for p in gp:
                p[0] = p[0] * 100 #algorithm uses cm
                p[1] = p[1] * 100 # m to cm

        

        app = SpaceModeling(groups)
        pparams,gparams, approaching_poses= app.solve()
    
    
        dis = 0
        for idx,pose in enumerate(approaching_poses):
            
            dis_aux = euclidean_distance(pos[0],pos[1],pose[0], pose[1])
            
            if idx == 0:
                goal_pose = pose[0:2]
                goal_quaternion = tf.transformations.quaternion_from_euler(0,0,pose[2])
                dis = dis_aux
            elif dis > dis_aux:
                goal_pose = pose[0:2]
                goal_quaternion = tf.transformations.quaternion_from_euler(0,0,pose[2])
                dis = dis_aux


        #calcular pose mais perto do robot
        try:
            rospy.loginfo("Approaching group!")
            result = movebase_client(goal_pose, goal_quaternion)
            if result:
                rospy.loginfo("Goal execution done!")
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

def movebase_client(goal_pose, goal_quaternion):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()


    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_pose[0]/100
    goal.target_pose.pose.position.y = goal_pose[1]/100
    goal.target_pose.pose.orientation.x = goal_quaternion[0]
    goal.target_pose.pose.orientation.y = goal_quaternion[1]
    goal.target_pose.pose.orientation.z = goal_quaternion[2]
    goal.target_pose.pose.orientation.w = goal_quaternion[3]

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    rospy.init_node("approach_group")
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    #ver pose e verificar se coincide com robot
    while not rospy.is_shutdown():
        try:
            (pos,ori) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(ori)

    #rospy.Subscriber("/approaching_poses",PoseArray,callback, (pos,yaw))
    data = rospy.wait_for_message('/faces', PoseArray)

    approach_group(data,pos,yaw)
    
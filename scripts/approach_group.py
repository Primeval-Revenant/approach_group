#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray

from matplotlib import pyplot as plt

import math

import tf

def euclidean_distance(x1, y1, x2, y2):
    """Euclidean distance between two points in 2D."""
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

def callback(data, args):
    pos = args[0]
    ori = args[1]

    
    
    dis = 0
    for idx,pose in enumerate(data.poses):
        
        dis_aux = euclidean_distance(pos[0],pos[1],pose.position.x, pose.position.y)
        
        if idx == 0:
            goal_pose = pose.position
            goal_quaternion = pose.orientation
            dis = dis_aux
        elif dis > dis_aux:
            goal_pose = pose.position
            goal_quaternion = pose.orientation
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
    goal.target_pose.pose.position = goal_pose
    goal.target_pose.pose.orientation = goal_quaternion

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

    rospy.Subscriber("/approaching_poses",PoseArray,callback, (pos,yaw))
    rospy.spin()
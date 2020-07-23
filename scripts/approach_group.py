#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray

from matplotlib import pyplot as plt

import math

import tf


def callback(data):

    #for pose in data.poses:
    #calcular pose mais perto do robot
        
    goal_pose = data.poses[1].position
    goal_quaternion = data.poses[1].orientation
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


    rospy.Subscriber("/approaching_poses",PoseArray,callback)
    rospy.spin()
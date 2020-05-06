#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray

import tf


def callback(data):
    group = []
    for pose in data.poses:
        rospy.loginfo("Person Detected")

        quartenion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quartenion)

        pose_person = [pose.position.x, pose.position.y, yaw]
        group.append(pose_person)


        goal_pose = [-2.05, 1.42, 2.53] #in base_footprint frame
        goal_quaternion = tf.transformations.quaternion_from_euler(0,0,goal_pose[2])

        # Pose base_footprint frame ---- map frame

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
    goal.target_pose.pose.position.x = goal_pose[0]
    goal.target_pose.pose.position.y = goal_pose[1]
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
    # listener = tf.TransformListener()
    # rate = rospy.Rate(10.0)
    # try:
    #     (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    #     print("entrei")
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     print("erro")

    rospy.Subscriber("/faces",PoseArray,callback )
    rospy.spin()

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
def rotate(px, py, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    qx = math.cos(angle) * px - math.sin(angle) * py
    qy = math.sin(angle) * px + math.cos(angle) * py
    return qx, qy
def approach_group(data ,pos, ori):
    listener = tf.TransformListener()
    group = []
    if not data.poses:
        group = []
    else:
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        tx = trans[0]
        ty = trans[1]
        t_quarterion = rot
        (t_roll, t_pitch, t_yaw) = tf.transformations.euler_from_quaternion(t_quarterion)

        for pose in data.poses:
            rospy.loginfo("Person Detected")

            quartenion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quartenion)

            (px, py) = rotate(pose.position.x, pose.position.y, t_yaw)
            pose_x = px + tx
            pose_y = py + ty
            pose_yaw = yaw + t_yaw


            pose_person = [pose_x, pose_y,pose_yaw]

            group.append(pose_person)

    if group:
        aux_group = copy.deepcopy(group)
        groups = [aux_group]
        for gp in groups:
            for p in gp:
                p[0] = p[0] * 100 #algorithm uses cm
                p[1] = p[1] * 100 # m to cm



        app = SpaceModeling(groups)
        pparams,gparams, approaching_poses, len_areas = app.solve()


       



    # # Choose the nearest pose
    #     dis = 0
    #     for idx,pose in enumerate(approaching_poses):

    #         # Approaching poses are in meteres divide by 100  m - cm
    #         dis_aux = euclidean_distance(pos[0],pos[1],pose[0]/100, pose[1]/100 )

    #         if idx == 0:
    #             goal_pose = pose[0:2]
    #             goal_quaternion = tf.transformations.quaternion_from_euler(0,0,pose[2])
    #             dis = dis_aux
    #         elif dis > dis_aux:
    #             goal_pose = pose[0:2]
    #             goal_quaternion = tf.transformations.quaternion_from_euler(0,0,pose[2])
    #             dis = dis_aux

    #Choose the pose in the biggest approaching area 

        idx = len_areas.index(max(len_areas))
        goal_pose = approaching_poses[idx][0:2]
        goal_quaternion = tf.transformations.quaternion_from_euler(0,0,approaching_poses[idx][2])
        print(goal_pose)
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

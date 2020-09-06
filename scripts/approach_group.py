#!/usr/bin/env python

import rospy
import tf
import actionlib 
import math

from nav_msgs.msg import OccupancyGrid
from group_msgs.msg import People, Groups
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray
from ellipse import plot_ellipse

from approaching_pose import approaching_area_filtering, approaching_heuristic, zones_center

# CONSTANTS
# Human Body Dimensions top view in cm
HUMAN_Y = 45
HUMAN_X = 20


def rotate(px, py, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    qx = math.cos(angle) * px - math.sin(angle) * py
    qy = math.sin(angle) * px + math.cos(angle) * py

    return qx, qy

def euclidean_distance(x1, y1, x2, y2):
    """Euclidean distance between two points in 2D."""
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist


def movebase_client(goal_pose, goal_quaternion):
    """
    """

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


class ApproachingPose():
    """
    """
    def __init__(self):
        """
        """
        rospy.init_node('ApproachPose', anonymous=True)
        rospy.Subscriber("/move_base_flex/global_costmap/costmap",OccupancyGrid , self.callbackCm, queue_size=1)
        rospy.Subscriber("/groups",Groups , self.callbackGr, queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        self.costmap_received = False
        self.people_received = False
        self.groups = []
        self. pose = []

    def callbackGr(self,data):
        self.people_received = True
        self.groups = []
     
        listener = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        tx = trans[0]
        ty = trans[1]
        (_, _, t_yaw) = tf.transformations.euler_from_quaternion(rot)
        self.pose = [tx, ty, t_yaw]

        for group in data.groups:
            tmp_group = []
            if len(group.people) > 1: # Only store groups, ignore individuals
                for people in group.people:
                    (px, py) = rotate(people.position.x, people.position.y, t_yaw)
                    pose_x = px + tx
                    pose_y = py + ty
                    pose_yaw = people.orientation + t_yaw

                    if people.ospace:
                        self.groups.append({'members': tmp_group,'pose':[pose_x, pose_y,pose_yaw], 'parameters' :[people.sx, people.sy] })
                    else:
                        tmp_group.append([pose_x, pose_y, pose_yaw])

             
    def callbackCm(self, data):
        """
        """
        self.costmap = data
        self.costmap_received = True

           
    def run_behavior(self):
        """
        """
        while not rospy.is_shutdown():
            if self.people_received and self.groups:
                self.people_received = False
                
                if self.costmap_received and self.groups:
                    self.costmap_received = False

                    # # Choose the nearest pose
                    dis = 0
                    for idx,group in enumerate(self.groups):

                    # Choose the nearest group
                        dis_aux = euclidean_distance(group["pose"][0],group["pose"][1],self.pose[0], self.pose[1] )

                        if idx == 0:
                            goal_group = group
                            dis = dis_aux
                        elif dis > dis_aux:
                            goal_group = group
                            dis = dis_aux


                    # Choose the nearest group
                    
                    group = goal_group
                    approaching_area = plot_ellipse(semimaj=group["parameters"][0], semimin=group["parameters"][1], x_cent=group["pose"][0],y_cent=group["pose"][1], data_out=True)


                    approaching_filter, approaching_zones = approaching_area_filtering(approaching_area, self.costmap)

                    approaching_filter, approaching_zones = approaching_heuristic(group["parameters"][0], group["parameters"][0] + HUMAN_X / 2 , group["pose"], approaching_filter, self.costmap, approaching_zones)


            
                    center_x, center_y, orientation = zones_center(
                        approaching_zones, group["pose"], group["parameters"][0])

                    approaching_poses = []
                    for l, _ in enumerate(center_x):
                        approaching_poses.append(
                            (center_x[l], center_y[l], orientation[l]))

                    len_areas = []
                    for zone in approaching_zones:
                        len_areas.append(len(zone))

                    # Choose the pose in the biggest approaching area 

                    idx = len_areas.index(max(len_areas))
                    goal_pose = approaching_poses[idx][0:2]
                    goal_quaternion = tf.transformations.quaternion_from_euler(0,0,approaching_poses[idx][2])
                    
                    try:
                        rospy.loginfo("Approaching group!")
                        result = movebase_client(goal_pose, goal_quaternion)
                        if result:
                            rospy.loginfo("Goal execution done!")
                            break
                    except rospy.ROSInterruptException:
                        rospy.loginfo("Navigation test finished.")
                    



if __name__ == '__main__':
    approaching_pose = ApproachingPose()
    approaching_pose.run_behavior()

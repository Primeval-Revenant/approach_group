#!/usr/bin/env python

import rospy
import tf
import actionlib 
import math

from nav_msgs.msg import OccupancyGrid
from group_msgs.msg import People, Groups
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker


from ellipse import plot_ellipse
import matplotlib.pyplot as plt

from approaching_pose import approaching_area_filtering, approaching_heuristic, zones_center
from plot_approach import plot_group, plot_person, draw_arrow
# CONSTANTS
# Human Body Dimensions top view in m
HUMAN_Y = 0.45
HUMAN_X = 0.20


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



def group_radius(persons, group_pose):
    """Computes the radius of a group."""
    group_radius = 0  # average of the distance of the group members to the center -> initial approaching radius
    pspace_radius = 0  # Based on the closest person to the group center
    ospace_radius = 0  # Based on the farthest persons to the group center -> O-space gaussian radius

    sum_radius = 0
    for person in persons:
        # average of the distance between the group members and the center of the group, o-space radius
        distance = euclidean_distance(person[0],
                                      person[1], group_pose[0], group_pose[1])
        sum_radius += distance

        if ospace_radius == 0:
            ospace_radius = distance
        else:
            ospace_aux = distance
            if ospace_aux < ospace_radius:
                ospace_radius = ospace_aux

        if pspace_radius == 0:
            pspace_radius = distance
        else:
            pspace_aux = distance
            if pspace_aux > pspace_radius:
                pspace_radius = pspace_aux

    pspace_radius += HUMAN_X / 2
    ospace_radius -= HUMAN_X / 2
    group_radius = sum_radius / len(persons)
    return group_radius, pspace_radius, ospace_radius


class ApproachingPose():
    """
    """
    def __init__(self):
        """
        """
        rospy.init_node('ApproachPoseMarker', anonymous=True)
        rospy.Subscriber("/move_base_flex/global_costmap/costmap",OccupancyGrid , self.callbackCm, queue_size=1)
        rospy.Subscriber("/groups",Groups , self.callbackGr, queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        self.costmap_received = False
        self.people_received = False
        self.groups = []
        self. pose = []

        self.pubm = rospy.Publisher('/approaching_area', Marker, queue_size=100)
        self.pubg = rospy.Publisher('/approaching_poses', PoseArray, queue_size=1)

    def callbackGr(self,data):
        """ Groups data callback. """
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
                        # group_radius average of the distance of the group members to the center
                        # pspace_radius  Based on the closest person to the group center
                        # ospace_radius Based on the farthest persons to the group center
                        g_radius, pspace_radius, ospace_radius = group_radius(tmp_group, [pose_x, pose_y,pose_yaw])

                        self.groups.append({'members': tmp_group,'pose':[pose_x, pose_y,pose_yaw], 'parametetrs' :[people.sx, people.sy], 'g_radius': g_radius, 'ospace_radius': ospace_radius, 'pspace_radius': pspace_radius})
                    else:
                        tmp_group.append([pose_x, pose_y, pose_yaw])

             
    def callbackCm(self, data):
        """ Costmap Callback. """
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

                    # Meter algures uma condicap que verifica que se nao for possivel aproximar o grupo escolher outro
                    #Tentar plotar costmap
                    # Choose the nearest group
                    
                    group = goal_group
                    g_radius = group["g_radius"]  #Margin to for safer results
                    pspace_radius = group["pspace_radius"]
                    ospace_radius = group["ospace_radius"]
                    approaching_area = plot_ellipse(semimaj=g_radius, semimin=g_radius, x_cent=group["pose"][0],y_cent=group["pose"][1], data_out=True)
                    approaching_filter, approaching_zones = approaching_area_filtering(approaching_area, self.costmap)
                    approaching_filter, approaching_zones = approaching_heuristic(g_radius, pspace_radius, group["pose"], approaching_filter, self.costmap, approaching_zones)
#################################################################################################################################
#Approaching area marker
                    marker = Marker()
                    marker.header.frame_id = "/map"

                    marker.type = marker.SPHERE_LIST
                    marker.action = marker.ADD
                    marker.pose.orientation.w = 1

                    for point_f in approaching_filter:
                        
                        m_point = Point()
                        m_point.x = point_f[0]
                        m_point.y = point_f[1]
                        m_point.z = 0.1
                        marker.points.append(m_point)

                    t = rospy.Duration()
                    marker.lifetime = t
                    marker.scale.x = 0.03
                    marker.scale.y = 0.03
                    marker.scale.z = 0.03
                    marker.color.a = 1.0
                    marker.color.r = 1.0
                    self.pubm.publish(marker)

#######################################################################################        
# Approaching pose Pose Array            
                    ap_points = PoseArray()
                    ap_points.header.frame_id = "/map"
                    ap_points.header.stamp = rospy.Time.now()

            
                    center_x, center_y, orientation = zones_center(
                        approaching_zones, group["pose"], g_radius)

                    approaching_poses = []
                    for l, _ in enumerate(center_x):
                        approaching_poses.append((center_x[l], center_y[l], orientation[l]))


                        ap_pose = Pose()
                        ap_pose.position.x = center_x[l]
                        ap_pose.position.y = center_y[l]
                        ap_pose.position.z = 0.1
                        quaternion = tf.transformations.quaternion_from_euler(0,0,orientation[l])
                        ap_pose.orientation.x = quaternion[0]
                        ap_pose.orientation.y = quaternion[1]
                        ap_pose.orientation.z = quaternion[2]
                        ap_pose.orientation.w = quaternion[3]

                        ap_points.poses.append(ap_pose)
                    print("Publiquei")
                    self.pubg.publish(ap_points)
                    rospy.spin()



            


if __name__ == '__main__':
    approaching_pose = ApproachingPose()
    approaching_pose.run_behavior()

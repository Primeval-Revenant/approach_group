#!/usr/bin/env python

import rospy
import tf as convert
import tf2_ros as tf
import actionlib 
import math
import numpy as np

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from group_msgs.msg import People, Groups
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from move_base_msgs.msg import MoveBaseActionResult
from ellipse import plot_ellipse
from sys import getsizeof, argv
import matplotlib.pyplot as plt

from approaching_pose import approaching_area_filtering, approaching_heuristic, zones_center
from plot_approach import plot_group, plot_person, draw_arrow
# CONSTANTS
# Human Body Dimensions top view in m
HUMAN_Y = 0.45
HUMAN_X = 0.20
DISTANCE_ADAPT = 6
ADAPT_LIMIT = 1.2
VEL_ADAPT_FACTOR = 1.5


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

#https://stackoverflow.com/a/2007355
def shortest_angular_distance(x,y):
    return min(y-x, y-x+2*math.pi, y-x-2*math.pi, key=abs)


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
    
    return client.get_result()


def group_radius(persons, group_pose):
    """Computes the radius of a group."""
    group_radius = 0  # average of the distance of the group members to the center -> initial approaching radius
    pspace_radius = 0  # Based on the closest person to the group center
    ospace_radius = 0  # Based on the farthest persons to the group center -> O-space gaussian radius

    sum_radius = 0
    for person in persons:
        
        # average of the distance between the group members and the center of the group, o-space radius
        distance = euclidean_distance(person["pose"][0],
                                      person["pose"][1], group_pose[0], group_pose[1])
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
        rospy.init_node('ApproachPose', anonymous=True)
        rospy.Subscriber("/groups",Groups , self.callbackGr, queue_size=1) #Receives groups from the the people publisher node
        rospy.Subscriber("/clicked_point",PointStamped, self.callbackPoint, queue_size=1) #Receives the initial location of the approach target
        rospy.Subscriber("/move_base/result",MoveBaseActionResult, self.callbackMoveResult, queue_size=1) #Checks if the approach has concluded
        map_subscriber = rospy.Subscriber("/move_base/global_costmap/costmap",OccupancyGrid , self.callbackCm, queue_size=1) #Subscribe to the costmap
        map_subscriber = rospy.Subscriber("/move_base/global_costmap/costmap_updates",OccupancyGridUpdate , self.callbackCmUpdate, queue_size=10) #Receive costmap updates
        self.pub = rospy.Publisher('/approach_target', PointStamped, queue_size=1)
        self.pubap = rospy.Publisher('/approach_poses', PoseArray, queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        self.costmap_received = False
        self.costmap_update = None
        self.costmap = None
        self.people_received = False
        self.groups = []
        self.pose = []
        self.point_clicked = []
        self.groups_data = []
        self.moveresult = None
        self.target = None
        self.plotting = False

    def callbackGr(self,data):
        """ Groups data callback. """
        self.people_received = True
        self.groups_data = data

        self.groups = []
        for group in self.groups_data.groups:
            tmp_group = []

            #Groups are handled differently from individuals
            if len(group.people) > 1: 
                for people in group.people:
    
                    pose_x = people.position.x
                    pose_y = people.position.y
                    pose_yaw = people.orientation 


                    if people.ospace:
                        # group_radius average of the distance of the group members to the center
                        # pspace_radius  Based on the closest person to the group center
                        # ospace_radius Based on the farthest persons to the group center
                        g_radius, pspace_radius, ospace_radius = group_radius(tmp_group, [pose_x, pose_y,pose_yaw])
                        self.groups.append({'members': tmp_group,'pose':[pose_x, pose_y,pose_yaw], 'parameters' :[people.sx, people.sy, people.sx_back], 'g_radius': g_radius, 'ospace_radius': ospace_radius, 'pspace_radius': pspace_radius, 'velocity': [people.velocity.linear.x, people.velocity.linear.y]})
                    else:
                        tmp_group.append({'pose':[pose_x, pose_y, pose_yaw], 'parameters':[people.sx, people.sy, people.sx_back, people.sy_right]})
            else:
                for people in group.people:
                    tmp_group.append({'pose':[people.position.x, people.position.y, people.orientation], 'parameters':[people.sx, people.sy, people.sx_back, people.sy_right]})
                    self.groups.append({'members': tmp_group, 'pose': [people.position.x, people.position.y, people.orientation],'parameters' :[people.sx, people.sy, people.sx_back], 'velocity': [people.velocity.linear.x, people.velocity.linear.y]})
     
    def callbackPoint(self,data):
        """Receive approach target"""
        self.point_clicked = data

             
    def callbackCm(self, data):
        """ Costmap Callback. """
        self.costmap = data
        self.costmap_received = True
        self.costmap.data = list(self.costmap.data)

    def callbackCmUpdate(self, data):
        """ Costmap Update Callback. """
        self.costmap_update = data
        if self.costmap:
            idx = 0
            for iy in range(self.costmap_update.y, self.costmap_update.height+self.costmap_update.y):
                for ix in range(self.costmap_update.x, self.costmap_update.width+self.costmap_update.x):
                    index = iy * self.costmap.info.width + ix

                    self.costmap.data[index] = self.costmap_update.data[idx]
                    idx += 1

    def callbackMoveResult(self,data):
        """Check if approach has finished"""
        self.moveresult = data

           
    def run_behavior(self):
        """
        """

        fail_count = 0
        tfBuffer = tf.Buffer()

        listener = tf.TransformListener(tfBuffer)

        rate = rospy.Rate(10.0)
        rate2 = rospy.Rate(3.0)
        while not rospy.is_shutdown():

            if self.people_received and self.groups_data and self.point_clicked:

                self.people_received = False
                
                if self.costmap_received and self.groups_data and self.point_clicked:
                    self.costmap_received = False

                    # Calculate the distances between the chosen point and every group
                    dis = []
                    for idx,group in enumerate(self.groups):   
                        dis.append(euclidean_distance(group["pose"][0],group["pose"][1],self.point_clicked.point.x, self.point_clicked.point.y))

                    self.point_clicked = []

                    approach_number = None
                    goal_pose = None

                    while not rospy.is_shutdown():
                        
                        try:
                            transf = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            rate.sleep()
                            continue

                        tx = transf.transform.translation.x
                        ty = transf.transform.translation.y
                        quatern = (transf.transform.rotation.x, transf.transform.rotation.y, transf.transform.rotation.z, transf.transform.rotation.w)
                        (_, _, t_yaw) = convert.transformations.euler_from_quaternion(quatern)
                        self.pose = [tx, ty, t_yaw]


                        # Try to find an appropriate pose and approach a chosen group
                        if self.groups:

                            group_aux = self.groups
                            group_idx = np.argsort(dis)

                            group = group_aux[group_idx[0]]

                            #Check if nearest detected group is the same group it was ordered to approach, by checking distance from last known position to current, and check if member number is the same.
                            #Necessary in case robot loses sight of a group, due to lack of more reliable tracking.
                            if dis[group_idx[0]] < 3 and ((not approach_number) or (len(group['members']) == approach_number)):    

                                if self.moveresult:
                                    if self.moveresult.status.status == 3:
                                        rospy.loginfo("Goal execution done!")
                                        break
                                
                                #Set a target to be used to track the chosen group to approach even if it changes position. Is used locally and in the people tracker
                                self.target = (group["pose"][0],group["pose"][1])

                                targetsend = PointStamped()
                                targetsend.header.frame_id = "/map"
                                targetsend.point.x = group["pose"][0]
                                targetsend.point.y = group["pose"][1]
                                self.pub.publish(targetsend)

                                approach_number = len(group['members'])

                                #Check if it must moderate the velocity adaptation due to being too close to the target
                                dist_pose = euclidean_distance(self.pose[0],self.pose[1],group["pose"][0],group["pose"][1])
                                if dist_pose > DISTANCE_ADAPT:
                                    dist_modifier = 1
                                else:
                                    dist_modifier = min(1,(dist_pose/DISTANCE_ADAPT)*2)

                                vel_magnitude = math.sqrt(group["velocity"][0]**2+group["velocity"][1]**2)

                                vel_factor = min(VEL_ADAPT_FACTOR*dist_modifier*vel_magnitude, ADAPT_LIMIT)
                                
                                if len(group['members']) > 1:
                                    g_radius = group["g_radius"]
                                    pspace_radius = group["pspace_radius"]+vel_factor
                                    ospace_radius = group["ospace_radius"]
                                else:
                                    g_radius = 1
                                    pspace_radius = 1.4+vel_factor
                                    ospace_radius = 0.45

                                #Calculate approaching poses
                                approaching_filter, approaching_zones, approaching_poses, idx = approaching_heuristic(g_radius, pspace_radius, ospace_radius, group["pose"], self.costmap, group, self.pose, vel_magnitude, self.plotting)

                                #publish approaching poses
                                ap_pub = PoseArray()
                                ap_pub.header.frame_id = "/map"
                                ap_pub.header.stamp = rospy.Time.now()
                                for i in range(len(approaching_poses)):
                                    ap_aux = Pose()
                                    ap_aux.position.x = approaching_poses[i][0]
                                    ap_aux.position.y = approaching_poses[i][1]
                                    ap_aux.position.z = 0.1

                                    quaternion = convert.transformations.quaternion_from_euler(0,0,approaching_poses[i][2])

                                    ap_aux.orientation.x = quaternion[0]
                                    ap_aux.orientation.y = quaternion[1]
                                    ap_aux.orientation.z = quaternion[2]
                                    ap_aux.orientation.w = quaternion[3]

                                    ap_pub.poses.append(ap_aux)

                                self.pubap.publish(ap_pub)
                                
                                # Verify if there are approaching zones
                                if approaching_poses:

                                    if idx != -1 and (not goal_pose or euclidean_distance(goal_pose[0], goal_pose[1], approaching_poses[idx][0], approaching_poses[idx][1]) > 0):
                                        goal_pose = approaching_poses[idx][0:2]
                                        fail_count = 0

                                        goal_quaternion = convert.transformations.quaternion_from_euler(0,0,approaching_poses[idx][2])
                                        try:
                                            result = movebase_client(goal_pose, goal_quaternion)
                                        except rospy.ROSInterruptException:
                                            rospy.loginfo("Navigation test finished.")
                                    elif idx == -1:
                                        #Stop approach attempt if it fails at finding an approach pose too many times.
                                        fail_count = fail_count+1
                                        if fail_count == 5:
                                            rospy.loginfo("Failure to find valid approach poses.")
                                            break

                                else:
                                    rospy.loginfo("Impossible to approach group due to no approach poses.") 

                            #Recalculate which is the nearest group to the last one the robot tried to approach to ensure dynamic continuity
                            dis = []
                            for idx,group in enumerate(group_aux):   
                                dis.append(euclidean_distance(group["pose"][0],group["pose"][1],self.target[0], self.target[1]))

                            rate2.sleep()
                        
                    break
                            
                                
if __name__ == '__main__':
    approaching_pose = ApproachingPose()
    if len(argv) > 1 and argv[1] == 'print':
        approaching_pose.plotting = True
    approaching_pose.run_behavior()
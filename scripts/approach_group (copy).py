#!/usr/bin/env python

import rospy
import tf as convert
import tf2_ros as tf
import actionlib 
import math
import numpy as np

from nav_msgs.msg import OccupancyGrid
from group_msgs.msg import People, Groups
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray, PointStamped
from ellipse import plot_ellipse
from sys import getsizeof
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
        rospy.init_node('ApproachPose', anonymous=True)
        #rospy.Subscriber("/move_base/global_costmap/costmap",OccupancyGrid , self.callbackCm, queue_size=1)
        rospy.Subscriber("/groups",Groups , self.callbackGr, queue_size=1)
        rospy.Subscriber("/clicked_point",PointStamped, self.callbackPoint, queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        self.costmap_received = False
        self.people_received = False
        self.groups = []
        self.pose = []
        self.point_clicked = []
        self.groups_data = []

    def callbackGr(self,data):
        """ Groups data callback. """
        self.people_received = True
        self.groups_data = data
        #rospy.loginfo('Group update')
     
    def callbackPoint(self,data):
        self.point_clicked = data

             
    def callbackCm(self, data):
        """ Costmap Callback. """
        self.costmap = data
        self.costmap_received = True
        #rospy.loginfo('Costmap update')

           
    def run_behavior(self):
        """
        """
        while not rospy.is_shutdown():
            print(self.point_clicked)
            if self.people_received and self.groups_data and self.point_clicked:

                self.people_received = False

                rospy.Subscriber("/move_base/global_costmap/costmap",OccupancyGrid , self.callbackCm, queue_size=1)
                
                if self.costmap_received and self.groups_data and self.point_clicked:
                    self.costmap_received = False
                    

                    tfBuffer = tf.Buffer()

                    listener = tf.TransformListener(tfBuffer)

                    #listener.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(4.0))
                    rate = rospy.Rate(10.0)
                    while not rospy.is_shutdown():
                        try:
                            #(trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
                
                            transf = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            rate.sleep()
                            continue

                        tx = transf.transform.translation.x
                        ty = transf.transform.translation.y
                        quatern = (transf.transform.rotation.x, transf.transform.rotation.y, transf.transform.rotation.z, transf.transform.rotation.w)
                        (_, _, t_yaw) = convert.transformations.euler_from_quaternion(quatern)
                        self.pose = [tx, ty, t_yaw]
                        self.groups = []
                        for group in self.groups_data.groups:
                            tmp_group = []
                            if len(group.people) > 1: # Only store groups, ignore individuals
                                for people in group.people:
                    
                                    pose_x = people.position.x
                                    pose_y = people.position.y
                                    pose_yaw = people.orientation 

                    
  
                                    if people.ospace:
                                        # group_radius average of the distance of the group members to the center
                                        # pspace_radius  Based on the closest person to the group center
                                        # ospace_radius Based on the farthest persons to the group center
                                        g_radius, pspace_radius, ospace_radius = group_radius(tmp_group, [pose_x, pose_y,pose_yaw])

                                        self.groups.append({'members': tmp_group,'pose':[pose_x, pose_y,pose_yaw], 'parameters' :[people.sx, people.sy], 'g_radius': g_radius, 'ospace_radius': ospace_radius, 'pspace_radius': pspace_radius})
                                    else:
                                        tmp_group.append([pose_x, pose_y, pose_yaw])


                        # # Choose the nearest pose
                        dis = []
                        for idx,group in enumerate(self.groups):

                            # Approach the chosen group
                            
                            dis.append(euclidean_distance(group["pose"][0],group["pose"][1],self.point_clicked.point.x, self.point_clicked.point.y))

                        self.point_clicked = []
                        group_idx = np.argsort(dis)

                        for group_idxs in group_idx:

                            rospy.loginfo("Trying group %d",group_idxs)


                            # Meter algures uma condicap que verifica que se nao for possivel aproximar o grupo escolher outro
                            #Tentar plotar costmap
                            # Choose the nearest group
                        
                            group = self.groups[group_idxs]
                            g_radius = group["g_radius"]  # Margin for safer results
                            pspace_radius = group["pspace_radius"]
                            ospace_radius = group["ospace_radius"]
                            approaching_area = plot_ellipse(semimaj=g_radius, semimin=g_radius, x_cent=group["pose"][0],y_cent=group["pose"][1], data_out=True)
                            approaching_filter, approaching_zones = approaching_area_filtering(approaching_area, self.costmap)
                            approaching_filter, approaching_zones = approaching_heuristic(g_radius, pspace_radius, group["pose"], approaching_filter, self.costmap, approaching_zones)

                    
                            center_x, center_y, orientation, approach_space = zones_center(
                                approaching_zones, group["pose"], g_radius)


                            approaching_poses = []
                            for l, _ in enumerate(center_x):
                                approaching_poses.append((center_x[l], center_y[l], orientation[l], approach_space[l]))

                            fig = plt.figure()
                            ax = fig.add_subplot(1, 1, 1)
                            plot_kwargs = {'color': 'g', 'linestyle': '-', 'linewidth': 0.8}
                            for person in group["members"]:
                                plot_person(person[0], person[1], person[2], ax, plot_kwargs)

                            _ = plot_group(group["pose"], g_radius, pspace_radius, ospace_radius, ax)

                            for i, angle in enumerate(orientation):
                                draw_arrow(center_x[i], center_y[i], angle, ax)
                            x_approach = [j[0] for j in approaching_filter]
                            y_approach = [k[1] for k in approaching_filter]
                            ax.plot(x_approach, y_approach, 'c.', markersize=5)
                            ax.plot(center_x, center_y, 'r.', markersize=5)
                            ax.set_aspect(aspect=1)
                            #fig.tight_layout()
                            plt.show()

                            # Verificar se zonas de aproximacao estao vazias, se estiverm escolher outro grupo ou fazer break
                            if approaching_zones:
                                
                                indexes = np.argsort(approach_space)
                                indexes = np.flip(indexes)
                                
                                idx = -1
                                for i in indexes:
                                    if approaching_poses[i][3] > 0.8:
                                        if idx == -1:
                                            idx = i
                                        elif euclidean_distance(approaching_poses[i][0],approaching_poses[i][1],self.pose[0],self.pose[1]) < euclidean_distance(approaching_poses[idx][0],approaching_poses[idx][1],self.pose[0],self.pose[1]):
                                            idx = i
                                    else:
                                        break


                                # Choose the pose in the biggest approaching area 

                                if idx == -1:
                                    rospy.loginfo("Impossible to approach group due to distance.")
                                else:
                                    goal_pose = approaching_poses[idx][0:2]
                                    goal_quaternion = convert.transformations.quaternion_from_euler(0,0,approaching_poses[idx][2])
                                    try:
                                        rospy.loginfo("Approaching group!")
                                        result = movebase_client(goal_pose, goal_quaternion)
                                        if result:
                                            rospy.loginfo("Goal execution done!")
                                            break
                                    except rospy.ROSInterruptException:
                                        rospy.loginfo("Navigation test finished.")

                            else:
                                rospy.loginfo("Impossible to approach group due to no approach poses.") 
                            
                                
                      



if __name__ == '__main__':
    approaching_pose = ApproachingPose()
    approaching_pose.run_behavior()

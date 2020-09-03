#!/usr/bin/env python

import rospy
import tf
import actionlib 
import math

from nav_msgs.msg import OccupancyGrid
from group_msgs.msg import People
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseArray
from ellipse import plot_ellipse

from approaching_pose_test_costmap import approaching_area_filtering, approaching_heuristic, zones_center

def rotate(px, py, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.
    The angle should be given in radians.
    """
    qx = math.cos(angle) * px - math.sin(angle) * py
    qy = math.sin(angle) * px + math.cos(angle) * py

    return qx, qy


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


class ApproachingPose():
    def __init__(self):
        rospy.init_node('ApproachPose', anonymous=True)
        rospy.Subscriber("/move_base_flex/global_costmap/costmap",OccupancyGrid , self.callbackCm, queue_size=1)
        rospy.Subscriber("/people",People , self.callbackPe, queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))

        self.costmap_received = False
        self.people_received = False
        self.groups = []


    def callbackCm(self, data):
        self.costmap = data
        self.costmap_received = True


    def callbackPe(self, data):
        self.people = data
        self.people_received = True
        self.groups = []

        # Devia estar a receber os grupos diretamente!!!!!!!! Corrigir-> Assumir por equanto que apenas estou a receber um grupo

        listener = tf.TransformListener()

        if not self.people.people:
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
            (t_roll, t_pitch, t_yaw) = tf.transformations.euler_from_quaternion(rot)
            
            group = []
            for pose in self.people.people:

                (px, py) = rotate(pose.position.x, pose.position.y, t_yaw)
                pose_x = px + tx
                pose_y = py + ty
                pose_yaw = pose.orientation + t_yaw
   
                if pose.ospace:
                    self.groups.append({'members': group,'pose':[pose_x, pose_y,pose_yaw], 'parameters' :[pose.sx, pose.sy] })
                    group = []

                else:
                    group.append([pose_x, pose_y,pose_yaw])
    


           
    def run_behavior(self):
        while not rospy.is_shutdown():
            if self.people_received and self.groups:
                self.people_received = False

                if self.costmap_received:
                    self.costmap_received = False

                    # calcula zonas de aproximacao para cada grupo como faziamos no gaussian_modeling e approaching_pose
                    #1st filtragem da area de aproximacao vendo no costmap os pontos que nao estejam livres

                    # Estou a assumir que apenas esta a trabalhar com um grupo ainda
                    print(self.groups[0])
                    group = self.groups[0]
                    approaching_area = plot_ellipse(semimaj=group["parameters"][0], semimin=group["parameters"][1], x_cent=group["pose"][0],y_cent=group["pose"][1], data_out=True)

                    ################################# Adaptar ao meu caso

                    # Substituir os niveis pelo costmap e depois verificar se estao livres ou nao 
                    # Approaching Area filtering - remove points that are inside the personal space of a person
                    approaching_filter, approaching_zones = approaching_area_filtering(approaching_area, self.costmap)

                    #approaching_filter, approaching_zones = approaching_heuristic(group_radius, pspace_radius, group_pos, approaching_filter, self.costmap, approaching_zones)


            
                    center_x, center_y, orientation = zones_center(
                        approaching_zones, group["pose"], group["parameters"][0])

                    approaching_poses = []
                    for l, value in enumerate(center_x):
                        approaching_poses.append(
                            (center_x[l], center_y[l], orientation[l]))

                    len_areas = []
                    for zone in approaching_zones:
                        len_areas.append(len(zone))


# Calculo das poses de aproximacao fazer aqui como tinha feito no gaussian modeling, mas agora uso a informacao que o costmao me da para saber se a zona esta free
# ou nao. ver o treshold da inflation porque certos valores o robot consegue navegar na inflation e esses pontos devem ser considerados.
# Se zona for vazia ir aumentando raio ocmo tinha feito antes

# Ver qual o grupo mais proxima e depois disso escolher a zona maior do grupo
                        #Choose the pose in the biggest approaching area 

                    idx = len_areas.index(max(len_areas))
                    goal_pose = approaching_poses[idx][0:2]
                    goal_quaternion = tf.transformations.quaternion_from_euler(0,0,approaching_poses[idx][2])
                    
                    try:
                        rospy.loginfo("Approaching group!")
                        result = movebase_client(goal_pose, goal_quaternion)
                        if result:
                            rospy.loginfo("Goal execution done!")
                    except rospy.ROSInterruptException:
                        rospy.loginfo("Navigation test finished.")
                    



if __name__ == '__main__':
    approaching_pose = ApproachingPose()
    approaching_pose.run_behavior()

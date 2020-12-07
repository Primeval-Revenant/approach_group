#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from people_msgs.msg import People, Person
import tf
import math

import copy

import actionlib



STRIDE = 65 # in cm
MDL = 8000

# Relation between personal frontal space and back space
BACK_FACTOR = 1.3

def calc_o_space(persons):
    """Calculates the o-space center of the group given group members pose"""
    c_x = 0
    c_y = 0
    
# Group size
    g_size = len(persons)
    
    for person in persons:
        c_x += person[0] + math.cos(person[2]) * STRIDE
        c_y += person[1] + math.sin(person[2]) * STRIDE

    center = [c_x / g_size, c_y / g_size]


    return center

class PeoplePublisher():
    """
    """
    def __init__(self):
        """
        """
        rospy.init_node('PeoplePublisher', anonymous=True)
        
        rospy.Subscriber("/faces",PoseArray,self.callback,queue_size=1)
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        self.pose_received = False

        self.data = None
        self.pub = rospy.Publisher('/people', People, queue_size=1)


    def callback(self,data):
        """
        """
        
        self.data = data
        self.pose_received = True
        

    def publish(self):
        """
        """
        
        data = self.data
        groups = []
        group = []

        persons = []

        if not data.poses:
            groups = []
        else:
            for pose in data.poses:

                rospy.loginfo("Person Detected")
                
                quartenion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
                (_, _, yaw) = tf.transformations.euler_from_quaternion(quartenion)

                pose_person = [pose.position.x , pose.position.y, yaw]
                persons.append(pose_person)
 
        if persons:
            p = People()
            p.header.frame_id = "/base_footprint"
            p.header.stamp = rospy.Time.now()

            for person in persons:

                p1 = Person()
                p1.position.x = person[0] 
                p1.position.y = person[1] 
        
                p1.velocity.x = 1/math.tan(person[2])
                p1.velocity.y = 1
                p.people.append(p1)
            
            self.pub.publish(p)
 

        else:
            p = People()
            p.header.frame_id = "/base_footprint"
            p.header.stamp = rospy.Time.now()
            self.pub.publish(p)
    def run_behavior(self):
        while not rospy.is_shutdown():
            if self.pose_received:
                
                self.pose_received = False
                self.publish()

if __name__ == '__main__':
    people_publisher = PeoplePublisher()
    people_publisher.run_behavior()
    eng.quit()

#!/usr/bin/env python 

#Joao Avelino, 2020.
#ISR-Lisboa / IST

#Just "detects" and sends out a PoseArray of people's faces in gazebo

import rospy
import random
import math
import numpy as np
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from human_awareness_msgs.msg import TrackedPersonsList, PersonTracker
import tf_conversions.posemath as pm
from copy import deepcopy
import PyKDL
import tf2_ros
import tf2_geometry_msgs
import tf_conversions

from std_srvs.srv import Empty, EmptyResponse, SetBool, SetBoolResponse


class ErrorGenerator:
    def __init__(self):
        self.step = 0
        self.MAXSTEP = 60 
        self.current_state = "default"
        self.active = False
        self.orientation_noise = False

        pos = np.linspace(-2.5, 10, self.MAXSTEP)
        pos = 5/(1 + 2*np.exp(-pos))

        neg = np.linspace(-2.5, 10, self.MAXSTEP)
        neg = -5/(1 + 2*np.exp(-neg))

        zer = np.zeros(self.MAXSTEP)


        self.trajectories = {"diagleft" : {"x" : tuple(pos), "y" : tuple(pos)},
                             "diagright" : {"x" : tuple(pos), "y" : tuple(neg)},
                             "back" : {"x" : tuple(pos), "y" : tuple(zer)},
                             "front" : {"x" : tuple(neg), "y" : tuple(zer)}}
        
    
    def changeState(self, newstate):
        self.step = 0
        self.current_state = newstate

    def getPose(self, pose):

        pose.position.x += random.normalvariate(0, 0.1)
        pose.position.y += random.normalvariate(0, 0.1)

        if self.orientation_noise:

            q = np.array([pose.orientation.x, pose.orientation.y ,pose.orientation.z , pose.orientation.w])
            q[3] += random.normalvariate(0, 0.5)
            nrm = np.linalg.norm(q)
            q = q/nrm

            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
        

        if self.current_state == "big_error_diagleft":
            if self.step >= self.MAXSTEP:
                self.changeState("default")
            else:
                pose = self.error_diagleft_traj(pose)

        elif self.current_state == "big_error_diagright":
            if self.step >= self.MAXSTEP:
                self.changeState("default")
            else:
                pose = self.error_diagright_traj(pose)

        elif self.current_state == "big_error_back":
            if self.step >= self.MAXSTEP:
                self.changeState("default")
            else:
                pose = self.error_back_traj(pose)

        elif self.current_state == "big_error_front":
            if self.step >= self.MAXSTEP:
                self.changeState("default")
            else:
                pose = self.error_front_traj(pose)

        elif self.current_state == "missdetection":
            if self.step >= self.MAXSTEP:
                self.changeState("default")
            else:
                self.step += 1
                pose = None


        return pose

    
    def error_diagleft_traj(self, pose):

        pose.position.x += self.trajectories["diagleft"]["x"][self.step]
        pose.position.y += self.trajectories["diagleft"]["y"][self.step]

        self.step += 1
        return pose

    def error_diagright_traj(self, pose):

        pose.position.x += self.trajectories["diagright"]["x"][self.step]
        pose.position.y += self.trajectories["diagright"]["y"][self.step]

        self.step += 1
        return pose

    def error_back_traj(self, pose):

        pose.position.x += self.trajectories["back"]["x"][self.step]
        pose.position.y += self.trajectories["back"]["y"][self.step]

        self.step += 1
        return pose 

    def error_front_traj(self, pose):

        pose.position.x += self.trajectories["front"]["x"][self.step]
        pose.position.y += self.trajectories["front"]["y"][self.step]

        self.step += 1
        return pose

#Use this to simulate a maximum detection frame rate
cbtime = [None]

#Change id
global change_ids 
change_ids = False

#0.1 seconds
min_interval = rospy.Duration(0,  100000000)

rospy.init_node("faces_detector")
message_pub = rospy.Publisher("/human_trackers", TrackedPersonsList, queue_size=10)

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

error_gen = ErrorGenerator()


def error_diagleft_srv(data):
    error_gen.changeState("big_error_diagleft")

    return EmptyResponse()

def error_diagright_srv(data):
    error_gen.changeState("big_error_diagright")

    return EmptyResponse()

def error_back_srv(data):
    error_gen.changeState("big_error_back")

    return EmptyResponse()

def error_front_srv(data):
    error_gen.changeState("big_error_front")

    return EmptyResponse()


def orientation_srv(data):
    if data.data:
        error_gen.orientation_noise = True
        return SetBoolResponse(success=True, message="Orientation noise enabled")
    else:
        error_gen.orientation_noise = False
        return SetBoolResponse(success=True, message="Orientation noise disabled")


def missdetections_srv(data):
    error_gen.changeState("missdetection")

    return EmptyResponse()


def change_ids_srv(data):
    global change_ids
    if data.data:
        change_ids = True
        return SetBoolResponse(success=True, message="Orientation noise enabled")
    else:
        change_ids = False
        return SetBoolResponse(success=True, message="Orientation noise disabled")


def callback(data):

    global change_ids

    now = rospy.Time.now()

    if cbtime[0] is not None:
        if now-cbtime[0] < min_interval:
            return

    cbtime[0] = now

    #Get the robot current position. We want to publish people detections relative to the /base_footprint

    index = [i for i,x in enumerate(data.name) if 'vizzy::base_footprint' in x]

    robot_pose = Pose()

    for i in index:
        robot_pose.position.x = float(data.pose[i].position.x)
        robot_pose.position.y = float(data.pose[i].position.y)
        robot_pose.position.z = float(data.pose[i].position.z)
        robot_pose.orientation.x = float(data.pose[i].orientation.x)
        robot_pose.orientation.y = float(data.pose[i].orientation.y)
        robot_pose.orientation.z = float(data.pose[i].orientation.z)
        robot_pose.orientation.w = float(data.pose[i].orientation.w)

    map_to_robot_tf = pm.fromMsg(robot_pose)

    # Get people's positions
    idx = [i for i,x in enumerate(data.name) if ('pessoa' in x and 'base' in x) or ('person' in x and 'link' in x)]

    people = TrackedPersonsList()
    people.header.frame_id = "odom"
    people.header.stamp = rospy.Time(0)

    #Publish detections 
    for i in idx:
        

        person_pose = pm.fromMsg(data.pose[i])
        rot = person_pose.M
        pos = person_pose.p


        [R, P, Y] = rot.GetRPY()

        Y -= math.pi/2.0

        if Y < 0:
            Y = Y+2.0*math.pi

        rotated = PyKDL.Frame(PyKDL.Rotation.RPY(R,P,Y),
            PyKDL.Vector(pos.x(),pos.y(),pos.z()))


        person = pm.toMsg(map_to_robot_tf.Inverse()*rotated)

        person_stamped = PoseStamped()


        #First compute in the base_footprint because that's what gazebo gives us.

        person_stamped.header.frame_id = "base_footprint"
        person_stamped.header.stamp = now
        person_stamped.pose.position = person.position
        person_stamped.pose.orientation = person.orientation
        

        # Add noise as needed

        ## Add noise to x and y positions

        ## Big errors simulating missing feet, curved people 3D estimation,
        ## and missdetections

        if error_gen.active:
            person_stamped.pose = error_gen.getPose(person_stamped.pose)

        if person_stamped.pose is None:
            continue


        


        #Then, publish poses in the odom frame to easily account for robot movements.
        #This way we don't have to make extra calculations: tf2 will solve everything
        try:
            transform = tfBuffer.lookup_transform("odom",
                                                  "base_footprint",
                                                  rospy.Time(0))
        except Exception as e:
            print("Error in transform: " + str(e))
            return


        person = tf2_geometry_msgs.do_transform_pose(person_stamped, transform)

        person_tracker = PersonTracker()

        person_tracker.id = str(i)


        person_tracker.body_pose.position = deepcopy(person.pose.position)
        person_tracker.body_pose.orientation = deepcopy(person.pose.orientation)


        person_tracker.head_pose.position = person.pose.position
        person_tracker.head_pose.position.z += 1.7
        person_tracker.head_pose.orientation = person.pose.orientation

        person_tracker.velocity.linear.x = -data.twist[i].linear.x
        person_tracker.velocity.linear.y = -data.twist[i].linear.y


        #If people are too far away or behind the robot, we can't detect them
        #if (math.sqrt(person.position.x**2+person.position.y**2) > 8.0) or (person.position.x < 0):
            #continue

        people.personList.append(person_tracker)
    

    #Simulate id change errors
    if change_ids:
        ids_vect = [p.id for p in people.personList]

        ids_vect.append(ids_vect.pop(0))
        for p, i in zip(people.personList, ids_vect):
            p.id = i


    message_pub.publish(people)
    


if __name__ == '__main__':
    rospy.Subscriber("/gazebo/link_states", LinkStates, callback)

    rospy.Service("/fake_people/error_diagleft", Empty, error_diagleft_srv)
    rospy.Service("/fake_people/error_diagright", Empty, error_diagright_srv)
    rospy.Service("/fake_people/error_back", Empty, error_back_srv)
    rospy.Service("/fake_people/error_front", Empty, error_front_srv)
    rospy.Service("/fake_people/set_orientation_noise", SetBool, orientation_srv)
    rospy.Service("/fake_people/create_missdetections", Empty, missdetections_srv)
    rospy.Service("/fake_people/change_ids", SetBool, change_ids_srv)

    rospy.spin()

#! /usr/bin/env python3
'''
    File name: approaching_pose.py
    Author: Francisco Melo
    Mail: francisco.raposo.melo@tecnico.ulisboa.pt
    Date created: X/XX/XXXX
    Date last modified: X/XX/XXXX
    Python Version: 3.7
'''

import math

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
from ellipse import plot_ellipse
import rospy 
import numpy as np
import matplotlib.pyplot as plt
from plot_approach import plot_group, plot_person, draw_arrow

# Approaching area radius increment in m
R_STEP = 0.01

# Thresold
THRESHOLD = 127
        # 127 -> cost -> definitely not in collision
        # http://wiki.ros.org/costmap_2d/hydro/inflation

MIN_DISTANCE_THRESHOLD = 0.95

#Convert from the altered cost scale used in the costmap topic back to the original costmap scale
costconvert = []
costconvert.append(0)
for i in range(1,99):
    costconvert.append(round((((i-1)*251)/97)+1))

costconvert.append(253)
costconvert.append(254)
costconvert.append(255)

def get_angle(pos1, pos2):
    "Angle between two points"
    return math.atan2(pos1[1] - pos2[1], pos1[0] - pos2[0])

def euclidean_distance(x1, y1, x2, y2):
    """Euclidean distance between two points in 2D."""
    dist = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return dist

# http://nic-gamedev.blogspot.com/2011/11/using-vector-mathematics-and-bit-of.html
#Detect if a point is inside a person/group's field of view
def isFOV(group, point):

    for person in group['members']:
        diff = np.subtract(point,[person['pose'][0],person['pose'][1]])
        diff /= np.linalg.norm(diff)

        facing_direct = math.cos(person['pose'][2])*diff[0]+math.sin(person['pose'][2])*diff[1]

        cosFOV = math.cos(((90/2)*math.pi)/180)
        
        if (facing_direct < cosFOV):
            return False

    return True

def gaussianPerson(x, y, x0, y0, varx, vary, skew):

    A = 211
    dx = x-x0
    dy = y-y0
    h = math.sqrt(dx*dx+dy*dy)
    angle = math.atan2(dy,dx)
    mx = math.cos(angle-skew) * h
    my = math.sin(angle-skew) * h
    f1 = (mx**2)/(2.0 * varx**2)
    f2 = (my**2)/(2.0 * vary**2)
    gauss = A * math.e**(-(f1 + f2))

    if(gauss > 254):
        return 254
    elif (gauss < 0):
        return 0
    else:
        return gauss

#https://stackoverflow.com/a/2007355
def shortest_angular_distance(x,y):
  return min(y-x, y-x+2*math.pi, y-x-2*math.pi, key=abs)

def gaussianMax(group,point):

    angle_point = math.atan2(point[1]-group["pose"][1],point[0]-group["pose"][0])
    diff = shortest_angular_distance(group["pose"][2],angle_point)

    if abs(diff) < math.pi/2:
        cost = gaussianPerson(point[0],point[1],group["pose"][0],group["pose"][1],group["parameters"][0], group["parameters"][1],group["pose"][2])
    else:
        cost = gaussianPerson(point[0],point[1],group["pose"][0],group["pose"][1],group["parameters"][2], group["parameters"][1],group["pose"][2])

    if len(group['members']) > 1:
        for person in group["members"]:

            angle_point = math.atan2(point[1]-person["pose"][1],point[0]-person["pose"][0])
            diff = shortest_angular_distance(person["pose"][2],angle_point)

            if abs(diff) < math.pi/2:
                if diff < 0:
                    cost_aux = gaussianPerson(point[0],point[1],person["pose"][0],person["pose"][1],person["parameters"][0], person["parameters"][3],person["pose"][2])
                else:
                    cost_aux = gaussianPerson(point[0],point[1],person["pose"][0],person["pose"][1],person["parameters"][0], person["parameters"][1],person["pose"][2])
            else:
                if diff < 0:
                    cost_aux = gaussianPerson(point[0],point[1],person["pose"][0],person["pose"][1],person["parameters"][2], person["parameters"][3],person["pose"][2])
                else:
                    cost_aux = gaussianPerson(point[0],point[1],person["pose"][0],person["pose"][1],person["parameters"][2], person["parameters"][1],person["pose"][2])

            if cost < cost_aux:
                cost = cost_aux

    return cost


def approaching_area_filtering(approaching_area, costmap, group, FOVcheck, distance_robot):
    """ Filters the approaching area by checking the points inside personal or group space. Also checks for FOV if the bool is true"""

    approaching_filter = []
    approaching_zones = []
    aux_list = []

    Firstbool = True #Bool that is only true on the first iteration. There are probably better methods but I couldn't see one when this was done
    First = False #Bool to verify wrap-around in approaching zones
    
    for x, y in zip(approaching_area[0], approaching_area[1]):

        if distance_robot > 100:
            cost = gaussianMax(group,[x,y])
        else:
            
            ox = costmap.info.origin.position.x
            oy = costmap.info.origin.position.y 
            resolution = costmap.info.resolution

            ix = int((x - (resolution/2) - ox) / resolution)
            iy = int((y - (resolution/2) - oy) / resolution)
            index = iy * costmap.info.width + ix
            cost = costmap.data[index]
            cost = costconvert[cost]

        if (cost <= THRESHOLD and cost != 255) and (not FOVcheck or (FOVcheck and isFOV(group,[x,y]))): #

            approaching_filter.append((x, y))
            aux_list.append((x, y))

            if Firstbool:
                First = True

        elif aux_list:
            approaching_zones.append(aux_list)
            aux_list = []

        Firstbool = False

    if aux_list:
        if First and approaching_zones:
            approaching_zones[0] = aux_list + approaching_zones[0]
        else:
            approaching_zones.append(aux_list)
    
    return approaching_filter, approaching_zones

#Calculates the approaching zone
def approaching_heuristic(group_radius, pspace_radius, ospace_radius, group_pos, costmap, group, pose, plotting):
    """ """

    approaching_radius = group_radius
    FOVcheck = True
    approach_counter = 0
    approach_max = 0
    approach_aux = None
    
    idx = -1
    idx_aux = -1
    idx_aux2 = -1
    approaching_radius_aux = 0

    distance_robot = euclidean_distance(pose[0],pose[1],group_pos[0], group_pos[1])

    #Consecutively checks each approaching area, widening the radius until it goes over a threshold. Starts with areas within FOV and switches to non-FOV if none is found
    while idx == -1 and (approaching_radius <= pspace_radius or FOVcheck == True):

        approaching_area = None
        approaching_filter = None
        approaching_zones = None
        approaching_poses = []

        if approaching_radius > pspace_radius:
            approaching_radius = group_radius
            FOVcheck = False

        approaching_area = plot_ellipse(
            semimaj=approaching_radius, semimin=approaching_radius, x_cent=group_pos[0], y_cent=group_pos[1], data_out=True)
        approaching_filter, approaching_zones = approaching_area_filtering(
            approaching_area, costmap, group, FOVcheck, distance_robot)

        if approaching_filter:
            center_x, center_y, orientation, approach_space = zones_center(
                        approaching_zones, group["pose"], group_radius)

            for l, _ in enumerate(center_x):
                approaching_poses.append((center_x[l], center_y[l], orientation[l], approach_space[l]))

            if plotting:
                fig = plt.figure()
                ax = fig.add_subplot(1, 1, 1)
                plot_kwargs = {'color': 'g', 'linestyle': '-', 'linewidth': 0.8}
                for person in group["members"]:
                    plot_person(person["pose"][0], person["pose"][1], person["pose"][2], ax, plot_kwargs)

                _ = plot_group(group["pose"], group_radius, pspace_radius, ospace_radius, ax)

                for i, angle in enumerate(orientation):
                    draw_arrow(center_x[i], center_y[i], angle, ax)
                x_approach = [j[0] for j in approaching_filter]
                y_approach = [k[1] for k in approaching_filter]
                ax.plot(x_approach, y_approach, 'c.', markersize=5)
                ax.plot(center_x, center_y, 'r.', markersize=5)
                ax.set_aspect(aspect=1)
                #fig.tight_layout()
                plt.show()

            indexes = np.argsort(approach_space)
            indexes = np.flip(indexes)
            #Check if approaching zone is wide enough for the robot. Value should be adjustable.
            for i in indexes:
                if approaching_poses[i][3] > MIN_DISTANCE_THRESHOLD:
                    #approach_counter += 1
                    if idx == -1:
                        idx = i
                    elif euclidean_distance(approaching_poses[i][0],approaching_poses[i][1],pose[0],pose[1]) < euclidean_distance(approaching_poses[idx][0],approaching_poses[idx][1],pose[0],pose[1]):
                        idx = i
                else:
                    break
            
            # if len(group["members"]) != 1:
            #     if approach_counter > approach_max:
            #         approach_max = approach_counter
            #         approach_aux = approaching_poses
            #         idx_aux2 = idx_aux
            #         approaching_radius_aux = approaching_radius
            # else:
            #     idx = idx_aux
            # approach_counter = 0

        approaching_radius += R_STEP

        # if approaching_radius >= pspace_radius and idx_aux2 != -1:
        #     idx = idx_aux2
        #     approaching_poses = approach_aux

    return approaching_filter, approaching_zones, approaching_poses, idx

#Calculates the center of an approaching zone
def zones_center(approaching_zones, group_pos, group_radius):
    """ """
    # https://stackoverflow.com/questions/26951544/algorithm-find-the-midpoint-of-an-arc
    #https://stackoverflow.com/questions/11674239/find-arcs-mid-point-given-start-end-and-center-of-circle-points
    #https://stackoverflow.com/questions/26951544/algorithm-find-the-midpoint-of-an-arc
    center_x = []
    center_y = []
    orientation = []
    approach_space = []
    
    for zone in approaching_zones:
        if len(approaching_zones) != 1:
            # Sort points clockwise
            zone.sort(key=lambda c: math.atan2(c[0]-group_pos[0], c[1]-group_pos[1]))

        idx = int(len(zone) / 2)
        center_x.append(zone[idx][0])
        center_y.append(zone[idx][1])
        arc_distance = euclidean_distance(zone[-1][0],zone[-1][1],zone[0][0],zone[0][1])
        approach_space.append(arc_distance)
        orientation.append(get_angle(group_pos, (zone[idx][0], zone[idx][1])))

    return center_x, center_y, orientation, approach_space



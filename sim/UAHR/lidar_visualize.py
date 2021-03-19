#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import math
import time

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import String 

from Fast_rviz import new_cube,new_line,size
from uah_msgs.msg import PolarArray, array_arcos,Pose2DArray



# LINEAS VERDES:
def cb_recognized_objects(msg):
    for i,relative_poses in enumerate(msg.array):
        if(relative_poses.angle <0):
            relative_poses.angle += 360 
        endx = (pose.x + relative_poses.dist *
            math.cos(math.radians(relative_poses.angle + pose.theta)))
        endy  = (pose.y + relative_poses.dist * 
            math.sin(math.radians(relative_poses.angle + pose.theta))) 
                
        all_objects.markers[i].points =[Point(
                                        pose.x /SF,
                                        pose.y /SF,
                                        0),
                                        Point(endx/SF,endy/SF,0)]
    pub_recognized_objects.publish(all_objects)

#### LINEAR ROJAS
def cb_enemy_robots(msg):

    for i in range(len(enemy_line_list)):
        if i<len(msg.array):
            if(msg.array[i].angle <0):
                msg.array[i].angle += 360 

            distx = (pose.x + msg.array[i].dist * math.cos(math.radians(msg.array[i].angle + pose.theta)))
            disty = (pose.y + msg.array[i].dist * math.sin(math.radians(msg.array[i].angle + pose.theta))) 
            all_enemies.markers[i].action = 0
            all_enemies.markers[i].text   = "Enemy robot"

            all_enemies.markers[i].points =[Point(
            pose.x /SF,
            pose.y /SF,
            0),
            Point(distx/SF,disty/SF,0)]
            all_enemies.markers[i].action = 0
        else:
            all_enemies.markers[i].action = 2

    pub_recognized_robots.publish(all_enemies)


if __name__ == "__main__":
    rospy.init_node("display_lidar")


    name_x      = rospy.search_param("pose_x")
    name_y      = rospy.search_param("pose_y")
    name_a      = rospy.search_param("pose_alfa")
    known_objects = rospy.get_param("objetos_triangulacion",[])
    pose_x      = rospy.get_param(name_x,0)
    pose_y      = rospy.get_param(name_y,0)
    pose_alfa   = rospy.get_param(name_a,0)
    pose        = Pose2D(pose_x,pose_y,pose_alfa)

    sub_recognized_objects = rospy.Subscriber("lidar_distance", 
            PolarArray, callback = cb_recognized_objects)
    sub_enemy_robots       = rospy.Subscriber("lidar_robots", 
            PolarArray, callback = cb_enemy_robots)

    
    pub_robot_marker_pose  = rospy.Publisher("pose_marker", Marker, queue_size = 10)
    pub_recognized_objects = rospy.Publisher("triangulate_objects",MarkerArray, queue_size = 10)
    pub_recognized_robots  = rospy.Publisher("enemy_markers",MarkerArray, queue_size = 10)

    pub_pose  = rospy.Publisher("pose",Pose2D, queue_size = 10)

    SF = 100 # Factor de escalado     

    all_objects = MarkerArray()
    all_objectives = MarkerArray()
    all_enemies = MarkerArray()
    flechas_velocidad = MarkerArray()

    robot_marker = new_cube(SF,"robot",pose, size(144,300,350),'b')        

    enemy_line_list = []        
    for i in range(10) :
        enemy_line_list.append('LE'+str(i))
        i +=1
    

    for objeto in known_objects:
        line_to_object = new_line(objeto[0],Point(pose.x/SF,pose.y/SF,0),Point(objeto[1]/SF,objeto[2]/SF,0),'g')
        all_objects.markers.append(line_to_object)

    for i in enemy_line_list:
        line_to_object = new_line(i,Point(0,0,0),Point(0,-1,0),'r')
        all_enemies.markers.append(line_to_object)
    time.sleep(1)
    pub_robot_marker_pose.publish(robot_marker)    
    rospy.spin()

#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import math
import time

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import String 

from Fast_rviz import new_cube,new_line,size,new_circunferance
from uah_msgs.msg import PolarArray, array_arcos,Pose2DArray

def cb_pose(msg):
    print("aksfjadsf")
    robot_marker.pose.position.x = msg.x/SF
    robot_marker.pose.position.y = msg.y/SF
    # Va en cuaternios
    angulo = math.radians(msg.theta)
    robot_marker.pose.orientation.z = math.sin(angulo/2)
    robot_marker.pose.orientation.w = math.cos(angulo/2)
    pose.x = msg.x
    pose.y = msg.y
    pose.theta = msg.theta
    pub_robot_marker_pose.publish(robot_marker)    


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

#
def cb_arcos(msg):
    arcos = []
    for i, arco in enumerate(msg.arcos):
        if(arco.motivo == 'R'):
            color = 'r'
            #arco.distance_inf =100
            arco.distance_sup = 500

        elif(arco.motivo == 'A'):
            color = 'b'

        elif(arco.motivo == 'O'):
            color = 'g'

        arcos.append(new_circunferance(SF, str(i)+"_chiquito", pose,
                     arco.distance_inf, arco.angle_inf, arco.angle_sup, color))

        arcos.append(new_circunferance(SF, str(i)+"_grande", pose,
                     arco.distance_sup, arco.angle_inf, arco.angle_sup, color))
    pub_arcos.publish(arcos)

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

    sub_arcos = rospy.Subscriber("arcos", array_arcos, callback=cb_arcos)

    sub_recognized_objects = rospy.Subscriber("lidar_distance", 
            PolarArray, callback = cb_recognized_objects)
    sub_enemy_robots       = rospy.Subscriber("lidar_robots", 
            PolarArray, callback = cb_enemy_robots)
    
    sub_robot_marker_pose = rospy.Subscriber("pose", Pose2D, callback=cb_pose)


    pub_robot_marker_pose  = rospy.Publisher("pose_marker", Marker, queue_size = 10)
    pub_recognized_objects = rospy.Publisher("triangulate_objects",MarkerArray, queue_size = 10)
    pub_recognized_robots  = rospy.Publisher("enemy_markers",MarkerArray, queue_size = 10)
    pub_arcos = rospy.Publisher(
        "rviz_robot_filters", MarkerArray, queue_size=10)
    #pub_pose  = rospy.Publisher("pose",Pose2D, queue_size = 10)


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
    time.sleep(2)
    pub_robot_marker_pose.publish(robot_marker)    
    #pub_pose.publish(pose)
    #print("RESQUEST SENT")
    rospy.spin()

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose2D, Point
from std_msgs.msg import Duration

import rospy
import math
class size:
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z

def new_cube(SF,name,pose,size,color,id="map"):
    robot_marker = Marker()
    robot_marker.header.frame_id = id
    robot_marker.header.stamp = rospy.Time.now()
    robot_marker.ns = name
    robot_marker.id = 0
    robot_marker.type = 1
    robot_marker.action = 0
    robot_marker.pose.orientation.x = 0.0
    robot_marker.pose.orientation.y = 0.0
    robot_marker.pose.orientation.z = pose.theta
    robot_marker.pose.orientation.w = 1.0
    robot_marker.scale.x = size.x / SF
    robot_marker.scale.y = size.y / SF
    robot_marker.scale.z = size.z / SF
    robot_marker.pose.position.x = pose.x / SF
    robot_marker.pose.position.y = pose.y / SF
    robot_marker.pose.position.z = robot_marker.scale.z / 2
    
    if(color=="g"):
        robot_marker.color.a = 1.0 
        robot_marker.color.r = 0.0
        robot_marker.color.g = 1.0
        robot_marker.color.b = 0.0
    elif(color=="b"):
        robot_marker.color.a = 1.0 
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 1.0
    elif(color=="r"):
        robot_marker.color.a = 1.0 
        robot_marker.color.r = 1.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 0.0
    else:
        raise ValueError("Bad color, you can only choose between 'r','b,'g' Fdo: Trompiz")
    return robot_marker

def new_cilinder(SF,name,pose,r,h,color,id="map"):
    cylinder_marker = Marker()
    cylinder_marker.header.frame_id = id
    cylinder_marker.header.stamp = rospy.Time.now()
    cylinder_marker.ns = name
    cylinder_marker.id = 0
    cylinder_marker.type = 3
    cylinder_marker.action = 0
    cylinder_marker.pose.orientation.x = 0.0
    cylinder_marker.pose.orientation.y = 0.0
    cylinder_marker.pose.orientation.z = 0.0
    cylinder_marker.pose.orientation.w = 1.0
    cylinder_marker.scale.x = r / SF
    cylinder_marker.scale.y = r / SF
    cylinder_marker.scale.z = h / SF
    cylinder_marker.pose.position.x = pose.x / SF
    cylinder_marker.pose.position.y = pose.y / SF
    cylinder_marker.pose.position.z = cylinder_marker.scale.z / 2
    
    if(color=="g"):
        cylinder_marker.color.a = 1.0 
        cylinder_marker.color.r = 0.0
        cylinder_marker.color.g = 1.0
        cylinder_marker.color.b = 0.0
    elif(color=="b"):
        cylinder_marker.color.a = 1.0 
        cylinder_marker.color.r = 0.0
        cylinder_marker.color.g = 0.0
        cylinder_marker.color.b = 1.0
    elif(color=="r"):
        cylinder_marker.color.a = 1.0 
        cylinder_marker.color.r = 1.0
        cylinder_marker.color.g = 0.0
        cylinder_marker.color.b = 0.0
    else:
        raise ValueError("Bad color, you can only choose between 'r','b,'g' Fdo: Trompiz")
    return cylinder_marker


def new_circunferance(SF,name,origin,radio,init_angle,end_angle,color,paso=10,id="map"):
    line = Marker()
    line.header.frame_id = id
    line.header.stamp = rospy.Time.now()
    line.ns   = name
    line.id   = 0
    line.type = 4
    line.action = 0
    #line.lifetime = rospy.Duration(3)

    line.pose.position.x = 0
    line.pose.position.y = 0
    line.pose.position.z = 0
    line.pose.orientation.x = 0.0
    line.pose.orientation.y = 0.0
    line.pose.orientation.z =  0 
    line.pose.orientation.w = 1.0
    line.scale.x = 0.1
    line.text = name

    if(color=="g"):
        line.color.a = 1.0 
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
    elif(color=="b"):
        line.color.a = 1.0 
        line.color.r = 0.0
        line.color.g = 0.0
        line.color.b = 1.0
    elif(color=="r"):
        line.color.a = 1.0 
        line.color.r = 1.0
        line.color.g = 0.0
        line.color.b = 0.0
    actual_angle = init_angle
    incremento = (end_angle - init_angle) / paso

    while actual_angle < end_angle:
        endx = (origin.x + radio * math.cos(math.radians(actual_angle + origin.theta)))/SF
        endy = (origin.y + radio * math.sin(math.radians(actual_angle + origin.theta)))/SF 
        line.points.append(Point(endx,endy,0))
        actual_angle += incremento
        endx = (origin.x + radio * math.cos(math.radians(actual_angle + origin.theta)))/SF
        endy = (origin.y + radio * math.sin(math.radians(actual_angle + origin.theta)))/SF
        line.points.append(Point(endx,endy,0))

    return line

def new_line(name,origin,end,color,id="map"):
    line = Marker()
    line.header.frame_id = id
    line.header.stamp = rospy.Time.now()
    line.ns   = name
    line.id   = 0
    line.type = 0
    line.action = 0
    line.pose.position.x = 0
    line.pose.position.y = 0
    line.pose.position.z = 0
    line.pose.orientation.x = 0.0
    line.pose.orientation.y = 0.0
    line.pose.orientation.z =  0 
    line.pose.orientation.w = 1.0
    line.scale.x = 0.2
    line.scale.y = 0.5
    line.scale.z = 0.2

    line.color.a = 1.0 
    line.color.r = 0.0
    line.color.g = 0.0
    line.color.b = 1.0
    line.text = name

    if(color=="g"):
        line.color.a = 1.0 
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
    elif(color=="b"):
        line.color.a = 1.0 
        line.color.r = 0.0
        line.color.g = 0.0
        line.color.b = 1.0
    elif(color=="r"):
        line.color.a = 1.0 
        line.color.r = 1.0
        line.color.g = 0.0
        line.color.b = 0.0
    line.points = [origin,end]

    return line


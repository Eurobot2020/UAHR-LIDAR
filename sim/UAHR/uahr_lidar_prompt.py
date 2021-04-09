#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
from uahr_msgs.msg import PolarArray, array_arcos, Pose2DArray


# LINEAS VERDES:
def cb_recognized_objects(msg):
    for n, p_object in enumerate(msg.array):
        if(p_object.dist):
            print(
                f"El objeto para triangular {n} esta en {p_object.angle} grados a una distancia de {p_object.dist}")
        else:
            print(f"El objeto para triangular {n} no se encuentra")


# LINEAR ROJAS
def cb_enemy_robots(msg):
    for p_enemy in msg.array:
        print(
            f"Puede que haya un enemigo en {p_enemy.angle} grados a una distancia de {p_enemy.dist}")
    print(f"Hay un total de {len(msg.array)} posibles enemigos")


if __name__ == "__main__":
    rospy.init_node("prompt_lidar")

    sub_recognized_objects = rospy.Subscriber("lidar_distance",
                                              PolarArray, callback=cb_recognized_objects)
    sub_enemy_robots = rospy.Subscriber("lidar_robots",
                                        PolarArray, callback=cb_enemy_robots)
    print("LIDAR PROMPT LAUNCHED")
    rospy.spin()

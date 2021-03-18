UAHR-LIDAR package
=====================================================================
Este paquete contiente el nodo de Lidar usado en la competición 
de Eurobot 2020/2021, además incluye una pequeña bateria de test
para comprobar su funcionamiento.

Author: Javier Ortiz Pérez-Jaraiz -> @LesmusTrompiz
Manteiner: Javier Ortiz Pérez-Jaraiz -> @LesmusTrompiz


Dependencias
=====================================================================
| Elemento       | Modelo            |
| -------------  | ----------------- |
| Lidar          | RplidarA2         |
| ROS            | Noetic or Melodic |

Como construir el paquete:
=====================================================================
    1) Clonar el repositirio en el workspace de ros dentro de la carpeta src.
    2) Lanzar catkin_make para compliar y construir el paquete.

ROSPARAMS:
=====================================================================
- Modo: Classic_Lidar / Competitivo / Demo
- Lado: Azul/Amarillo
- Init_PoseX: Posición incial en X
- Init_PoseY: Posición inicial en Y
- Init_PoseTheta: Posición inicial angular Y
- Triangulate objects: Lista con los objetos estáticos que debe de buscar.


Forked:
=====================================================================
Este repositorio hace uso de drivers creador por slamtec y la estructura
usada en sus nodos. Para más información se recomienda ir a las siguientes
páginas:

rplidar repo: https://github.com/Slamtec/rplidar_ros

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage:   http://www.slamtec.com/en/Lidar

rplidar SDK: https://github.com/Slamtec/rplidar_sdk

rplidar Tutorial:  https://github.com/robopeak/rplidar_ros/wiki

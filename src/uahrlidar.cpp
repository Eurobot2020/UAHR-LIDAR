/*
    UAHRL LIDAR NODE

    Este nodo se encargar de encontrar 
    a posibles enemigos dentro del campo 
    y de encontrar los objetos con los
    que triangular.

*/

#include "ros/ros.h"
#include "rplidar.h"
#include "geometry_msgs/Pose2D.h"
/////// Mensajes del equipo:
#include "uahr_msgs/Polar.h"
#include "uahr_msgs/PolarArray.h"
#include "uahr_msgs/Arco_Interes.h"
#include "uahr_msgs/array_arcos.h"

#include <math.h>
#include "uahr_class.hpp"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)
#define DISTANCIA_SEGURIDAD 500
#define LIMX 1500
#define LIMY 1000
#define N_2PI 2*M_PI
#define LIDAR_RANGE_MIN 150 // mm
#define MAX_DISANCE_ENEMY 3100

using namespace rp::standalone::rplidar;

RPlidarDriver * drv = NULL;


std::vector<FiltroAngular> VObjRdistance;


void publish_scan(ros::Publisher *pub_robots,
                  ros::Publisher *pub_objects,
                  rplidar_response_measurement_node_hq_t *nodes,
                  size_t node_count, ros::Time start,
                  double scan_time, bool inverted,
                  float  angle_min, float angle_max,
                  float  range_max,
                  LidarHandler &handler)
{    
    bool  reversed = (angle_max > angle_min);
    float angle_increment;
    float angulo;
    int   cluster = 0;
    float aux;
    static int flag= 0;
    uahr_msgs::PolarArray VPA;


    if (reversed) {
      aux = angle_min;
      angle_min =  RAD2DEG(M_PI - angle_max);
      angle_max =  RAD2DEG(M_PI - aux);
    }
    else {
      angle_min =  RAD2DEG(M_PI - angle_min);
      angle_max =  RAD2DEG(M_PI - angle_max);
    }

    angle_increment = (angle_max - angle_min) / (double)(node_count-1);
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    
        
    // Analiza los datos:
    if (!reverse_data) {
        // Calculo el ángulo
        for (size_t i = 0; i < node_count; i++) {
            angulo = angle_min + angle_increment * i;
        }
    }
    else{
        handler.new_scan2(nodes,node_count, angle_max,angle_increment);
        pub_objects->publish(handler.Vpubtriangulate);
        pub_robots->publish(handler.Vpubrobots);
    }
}


/*    
    pub_robots->publish(VPA);
    VPA.array.clear();
    
    // OJO QUE ESTO SE HA CAMBIADO
    for(int i=0; i<4;i++)
    {
        if(pose_objeto[i][0]!= 0)
        {
            robot_cluster.dist  = pose_objeto[i][1]/pose_objeto[i][0];
            robot_cluster.angle = pose_objeto[i][2]/pose_objeto[i][0];
            VPA.array.push_back(robot_cluster);
            //ROS_INFO("veo %d en %f a una distancia de %f",i,robot_cluster.dist,robot_cluster.angle);
        }
        else
        {
            robot_cluster.dist  = 0;
            robot_cluster.angle = 0;
            VPA.array.push_back(robot_cluster);
        }
    }
    pub_objects->publish(VPA);
    */           

bool getRPLIDARDeviceInfo(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            ROS_ERROR("Error, operation time out. RESULT_OPERATION_TIMEOUT! ");
        } else {
            ROS_ERROR("Error, unexpected error, code: %x",op_result);
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
    printf("\n");
    ROS_INFO("Firmware Ver: %d.%02d",devinfo.firmware_version>>8, devinfo.firmware_version & 0xFF);
    ROS_INFO("Hardware Rev: %d",(int)devinfo.hardware_version);
    return true;
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;
    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { 
        ROS_INFO("RPLidar health status : %d", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            ROS_ERROR("Error, rplidar internal error detected. Please reboot the device to retry.");
            return false;
        } else {
            return true;
        }

    } else {
        ROS_ERROR("Error, cannot retrieve rplidar health code: %x", op_result);
        return false;
    }
}



static float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

int main(int argc, char * argv[]) 
{
    ros::init(argc, argv, "rplidar_node");
    
    std::string channel_type;
    std::string tcp_ip;
    std::string serial_port;
    int tcp_port = 20108;
    int serial_baudrate = 115200;
    bool inverted = false;
    bool angle_compensate = true;
    float max_distance = 8.0;
    int angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("channel_type", channel_type, "serial");
    nh_private.param<std::string>("tcp_ip", tcp_ip, "192.168.0.7"); 
    nh_private.param<int>("tcp_port", tcp_port, 20108);
    nh_private.param<std::string>("serial_port", serial_port, "/dev/rplidar"); 
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200/*256000*/);//ros run for A1 A2, change to 256000 if A3
    nh_private.param<bool>("inverted", inverted, false);
    nh_private.param<bool>("angle_compensate", angle_compensate, false);
    nh_private.param<std::string>("scan_mode", scan_mode, std::string());
    
    std::string name_l; 
    std::string name_m; 

    std::string name_x;
    std::string name_y;
    std::string name_a;
    std::string name_objs;
    int fil_angular[4] {FA+10,FA+10,FA/2,FA};
    std::vector<ObjSearchData> lfobjects;

    std::string modo;
    std::vector<int> ejemplos_list;
    int px;
    int py;
    int ptheta;

    struct pose robot;
    ros::Subscriber sub_pose;
    

    // Parametros euroboteros
    if( nh.searchParam("pose_x",name_x) 
        &&
        nh.searchParam("pose_y",name_y)
        &&
        nh.searchParam("pose_alfa",name_a)
        &&
        nh.searchParam("objetos_busqueda_sucio",name_objs)
        &&
        nh.searchParam("modo",name_m))
    {
        nh.getParam(name_x, px);
        nh.getParam(name_y, py);
        nh.getParam(name_a, ptheta);
        nh.getParam(name_m, modo);
        nh.getParam(name_objs,ejemplos_list);
            
        robot.x = px;
        robot.y = py;
        robot.theta = ptheta;
    
        if(!((ejemplos_list.size()-1) % 4))
        {
            for(int j=0; j<ejemplos_list[0]*4; j=j+4)
            {
                lfobjects.push_back(ObjSearchData{ejemplos_list[j+1],ejemplos_list[j+2], ejemplos_list[j+3],ejemplos_list[j+4],j/4});
            }

            for(int i = 0; i<ejemplos_list[0]; i++)
            {    
                ROS_INFO("%d",lfobjects[i].x_inf);
                lfobjects[i].aexp = fil_angular[i]; 
            }
        }
        else
        {
            ROS_INFO("Error in the configuration");
            // Shutdown this node
            ros::shutdown();
        } 
        
        
        //n.getParam(, i);
    }
    else{
        // PUB THE ERROR:
        ROS_INFO("Error in the configuration");
        // Shutdown this node
        ros::shutdown();
    }

    class LidarHandler Handler(lfobjects,robot);
    sub_pose   = nh.subscribe("pose", 1000, &LidarHandler::cb_pose,&Handler);

    // Eurobot publishers:
    ros::Publisher  pub_robots = nh.advertise<uahr_msgs::PolarArray>("lidar_robots", 1000);
    ros::Publisher  pub_objs   = nh.advertise<uahr_msgs::PolarArray>("lidar_distance", 1000);
    ros::Publisher  pub_arcos  = nh.advertise<uahr_msgs::array_arcos>("arcos", 1000);;


    ROS_INFO("Paquete modificado de rplidar_ros por el UAH ROBOTICS TEAM");
    ROS_INFO("Configure finish");
    Handler.prompt_filters();
    


    u_result     op_result;


    // create the driver instance
    if(channel_type == "tcp"){
        drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
    }
    else{
        drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    }

    
    if (!drv) {
        ROS_ERROR("Create Driver fail, exit");
        return -2;
    }

    if(channel_type == "tcp"){
        // make connection...
        if (IS_FAIL(drv->connect(tcp_ip.c_str(), (_u32)tcp_port))) {
            ROS_ERROR("Error, cannot bind to the specified serial port %s.",serial_port.c_str());
            RPlidarDriver::DisposeDriver(drv);
            return -1;
        }

    }
    else{
       // make connection...
        if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
            ROS_ERROR("Error, cannot bind to the specified serial port %s.",serial_port.c_str());
            RPlidarDriver::DisposeDriver(drv);
            return -1;
        }

    }
    
    // get rplidar device info
    if (!getRPLIDARDeviceInfo(drv)) {
        return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }


    drv->startMotor();

    RplidarScanMode current_scan_mode;
    if (scan_mode.empty()) {
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    } else {
        std::vector<RplidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (IS_OK(op_result)) {
            _u16 selectedScanMode = _u16(-1);
            for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == _u16(-1)) {
                ROS_ERROR("scan mode `%s' is not supported by lidar, supported modes:", scan_mode.c_str());
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    ROS_ERROR("\t%s: max_distance: %.1f m, Point number: %.1fK",  iter->scan_mode,
                            iter->max_distance, (1000/iter->us_per_sample));
                }
                op_result = RESULT_OPERATION_FAIL;
            } else {
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }

    if(IS_OK(op_result))
    {
        //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
        angle_compensate_multiple = (int)(1000*1000/current_scan_mode.us_per_sample/10.0/360.0);
        if(angle_compensate_multiple < 1) 
          angle_compensate_multiple = 1;
        max_distance = current_scan_mode.max_distance;
        ROS_INFO("current scan mode: %s, max_distance: %.1f m, Point number: %.1fK , angle_compensate: %d",  current_scan_mode.scan_mode,
                 current_scan_mode.max_distance, (1000/current_scan_mode.us_per_sample), angle_compensate_multiple);
    }
    else
    {
        ROS_ERROR("Can not start scan: %08x!", op_result);
    }

    ros::Time end_scan_time;
    ros::Time start_scan_time;
    double scan_duration;

    while (ros::ok()) {
        rplidar_response_measurement_node_hq_t nodes[360*8];
        size_t   count = _countof(nodes);
        pub_arcos.publish(Handler.pub_filters());


        start_scan_time = ros::Time::now();
        op_result = drv->grabScanDataHq(nodes, count);
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec();
        if (op_result == RESULT_OK) {
            op_result = drv->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            if (op_result == RESULT_OK) {
                if (angle_compensate) {
                    //const int angle_compensate_multiple = 1;
                    const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple; j++) {

                                int angle_compensate_nodes_index = angle_value-angle_compensate_offset+j;
                                if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                    angle_compensate_nodes_index = angle_compensate_nodes_count-1;
                                angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                            }
                        }
                    }
                    

                    publish_scan(&pub_robots,&pub_objs,
                             angle_compensate_nodes, angle_compensate_nodes_count,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max, max_distance,
                             Handler);

                } else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].dist_mm_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD(getAngle(nodes[start_node]));
                    angle_max = DEG2RAD(getAngle(nodes[end_node]));

                    publish_scan(&pub_robots,&pub_objs,
                            &nodes[start_node], end_node-start_node +1,
                            start_scan_time, scan_duration, inverted,
                            angle_min, angle_max, max_distance,
                            Handler);
               }
            } else if (op_result == RESULT_OPERATION_FAIL) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);
                publish_scan(&pub_robots,&pub_objs,
                             nodes, count,
                             start_scan_time, scan_duration, inverted,
                             angle_min, angle_max, max_distance,
                             Handler);
            }
        }

        ros::spinOnce();
    }

    // done!
    drv->stopMotor();
    drv->stop();
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}

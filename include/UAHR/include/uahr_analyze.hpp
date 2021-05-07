#pragma once

#include "uahr_types.hpp"
#include "uahr_common.hpp"
#include "rplidar.h"
#include <fstream>



void StoreScan(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
    const float &angle_max, const float &angle_increment,std::ofstream &text);

void AnalyzeScan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
                const float &angle_max, const float &angle_increment,
                VFiltros & Vfiltros, Seccion max_distance,
                VSearchObjects & SearchObjects, VVPolars &Vclusters, 
                uahr_msgs::PolarArray  &Vpubposes,  
                uahr_msgs::PolarArray  &Vpubrobots);


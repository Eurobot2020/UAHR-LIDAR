#pragma once

#include "uahr_types.hpp"
#include "uahr_common.hpp"
#include "rplidar.h"


void FilterScan(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
    const float &angle_max, const float &angle_increment,
    VFiltros &Vfiltros, Seccion &enemy_distance,
    VVPolars &Vobjetos, VPolars &Vrobots);

void ObjectClusters(uahr_msgs::PolarArray &Vscans, uahr_msgs::PolarArray &Vposes);
//void enemy_clusters(uahr_msgs::PolarArray Venemies, &uahr_msgs::PolarArray);

void AnalyzeScan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
                const float &angle_max, const float &angle_increment,
                VFiltros &Vfiltros,  Seccion &enemy_distance,
                VVPolars &Vobjetos,  VPolars &Vrobots,
                uahr_msgs::PolarArray &Vpubposes, uahr_msgs::PolarArray &Vpubrobots);

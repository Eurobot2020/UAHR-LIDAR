#pragma once

#include "uahr_types.hpp"
#include "uahr_common.hpp"

void FilterScan(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
    &uahr_msgs::PolarArray Vfiltros, &Seccion enemy_distance,
    &uahr_msgs::PolarArray Vobjetos, &uahr_msgs::PolarArray Vrobots);

void objeto_clusters(&uahr_msgs::PolarArray Vscans,&uahr_msgs::PolarArray Vposes);
void enemy_clusters(&uahr_msgs::PolarArray Venemies, &uahr_msgs::PolarArray);

void AnalyzeScan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
                &uahr_msgs::PolarArray Vfiltros,  &Seccion enemy_distance,
                &uahr_msgs::PolarArray Vobjetos,  &uahr_msgs::PolarArray Vrobots,
                &uahr_msgs::PolarArray Vpubposes, &uahr_msgs::PolarArray Vpubrobots);

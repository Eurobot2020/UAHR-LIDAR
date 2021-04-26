#pragma once
#include "uahr_analyze.hpp"
#include "uahr_filters.hpp"
#include "uahr_types.hpp"
#include "uahr_common.hpp"



class LidarHandler
{
    private:
        const std::vector<ObjSearchData> TriangulateObjects;
        VFiltros SearchSections;
        pose     robot;
        //string   mode;
        VVPolars Vobjetos;
        VPolars  Vrobots;
        uahr_msgs::PolarArray  Vpubrobots;
        uahr_msgs::PolarArray  Vpubtriangulate; 
        

    public:
        LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p);
        ~LidarHandler();
        void cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg);
        void new_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
        const float &angle_max, const float &angle_increment);
        
        
};

// Constructor
LidarHandler::LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p)
: TriangulateObjects{ObjetosBusqueda}, robot{p}
{
    // Reservo la cantidad de memoria
    this->SearchSections.reserve(14); 
    this->Vobjetos.reserve(14); 
    this->Vrobots.reserve(20); 
    this->Vpubrobots.array.reserve(10);
    this->Vpubtriangulate.array.reserve(ObjetosBusqueda.size());

    // Cálculamos los nuevos filtros
    new_filters(this->SearchSections,this->robot,this->TriangulateObjects);
}

void LidarHandler::cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // Actualizo la posición del robot:
    this->robot.x = msg->x;
    this->robot.y = msg->y;
    this->robot.theta = msg->theta; 
    // Calculamos los nuevos filtros:
    new_filters(this->SearchSections,this->robot,this->TriangulateObjects);
}

void LidarHandler::new_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count, 
    const float &angle_max, const float &angle_increment)
{
    // Actualizo la posición del robot:
    Seccion max_distance_enemies(200,3000);
    
    // Calculamos los nuevos filtros:
    AnalyzeScan(nodes, node_count,
        angle_max, 
        angle_increment,
        this->SearchSections,
        max_distance_enemies,
        this->Vobjetos,
        this->Vrobots,
        this->Vpubtriangulate,
        this->Vpubrobots);
}

// Destructor,toda la memoria dinámica la gestiona el compilador 
LidarHandler::~LidarHandler()
{}



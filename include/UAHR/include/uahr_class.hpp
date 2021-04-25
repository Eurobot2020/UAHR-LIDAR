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
        string   mode;
        VVPolars Vobjetos;
        VPolars  Vrobots;
        VPolars  Vpubrobots;
        VPolars  Vpubtriangulate; 
        

    public:
        LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p);
        ~LidarHandler();
        void cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg);
        
};

// Constructor
LidarHandler::LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p)
: TriangulateObjects{ObjetosBusqueda}, robot{p}
{
    // Reservo la cantidad de memoria
    this->ArcOfInterest.reserve(14); 
    this->Vobjetos.reserve(14); 
    this->Vrobots.reserve(20); 
    this->Vpubrobots.reserve(10);
    this->Vpubtriangulate.reserve(ObjetosBusqueda.size());

    // Cálculamos los nuevos filtros
    new_filters(this->SearchSections,this->robot,this->TriangulateObjects);
}

void cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // Actualizo la posición del robot:
    this->pose.x = msg->x
    this->pose.y = msg->y
    this->pose.theta = msg->theta; 
    // Calculamos los nuevos filtros:
    new_filters(this->SearchSections,this->robot,this->TriangulateObjects);
}

void new_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count)
{
    // Actualizo la posición del robot:
    Seccion max_distance_enemies(200,3000);
    
    // Calculamos los nuevos filtros:
    AnalyzeScan(nodes, node_count,SearchSections,max_distance_enemies,
        Vobjetos,Vrobots,Vpubtriangulate,Vpubrobots);

    
    

}


// Destructor,toda la memoria dinámica la gestiona el compilador 
UAHR::~UAHR()
{}



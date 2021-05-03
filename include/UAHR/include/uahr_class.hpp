#pragma once
#include "uahr_analyze.hpp"
#include "uahr_filters.hpp"
#include "uahr_types.hpp"
#include "uahr_common.hpp"



class LidarHandler
{
    private:
        pose            robot;
        VSearchObjects  TriangulateObjects;
        VFiltros        SectionsFilters;
        VVPolars        Vclusters;
        bool            robot_localised;

    public:
        LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p);
        ~LidarHandler();
        
        uahr_msgs::PolarArray   Vpubrobots;
        uahr_msgs::PolarArray   Vpubtriangulate; 

        void cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg);
        void new_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
        const float &angle_max, const float &angle_increment);
        
        void prompt_filters();        
        void prompt_scans();

        //uahr_msgs::array_arcos  pub_filters();
};


// Constructor
LidarHandler::LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p)
: TriangulateObjects{ObjetosBusqueda}, robot{p}
{
    // Reservo la cantidad de memoria
    this->SectionsFilters.reserve(6); 
    this->Vclusters.reserve(15);

    this->Vpubrobots.array.reserve(10);
    this->Vpubtriangulate.array.reserve(ObjetosBusqueda.size());

    // TODO: NO QUIERO HACER ESTO PERO
    // EL FILL NO ME ESTA CAMBIANDO EL
    // SIZE DEL VECTOR :(
    uahr_msgs::Polar aux;
    for(int i = 0; i<ObjetosBusqueda.size();i++)
    {
        Vpubtriangulate.array.emplace_back(aux);
    }

    //std::fill(Vpubtriangulate.array.begin(),Vpubtriangulate.array.end(),aux);

    this->robot_localised = false;
    // Cálculamos los nuevos filtros
    UpdateFilters(this->robot_localised,this->SectionsFilters,this->robot,this->TriangulateObjects);
    prompt_filters();
}

void LidarHandler::cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // Actualizo la posición del robot:
    this->robot.x = msg->x;
    this->robot.y = msg->y;
    this->robot.theta = msg->theta; 
    // Calculamos los nuevos filtros:
    UpdateFilters(this->robot_localised,this->SectionsFilters,this->robot,this->TriangulateObjects);
    
}

void LidarHandler::new_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count, 
    const float &angle_max, const float &angle_increment)
{
    uahr_msgs::Polar aux;
    
    // Limpio los vectores:
    std::fill(Vpubtriangulate.array.begin(),Vpubtriangulate.array.end(),aux);
    this->Vpubrobots.array.clear();
    this->Vclusters.clear();

    // Actualizo la posición del robot:
    Seccion max_distance_enemies(100,3100);

    // Calculamos los nuevos filtros:
    AnalyzeScan(nodes, node_count,
        angle_max, 
        angle_increment,
        this->SectionsFilters,
        max_distance_enemies,
        this->TriangulateObjects,
        this->Vclusters,
        this->Vpubtriangulate,
        this->Vpubrobots);
    //prompt_scans();
}

void LidarHandler::prompt_filters()
{
    for (auto & filtro : SectionsFilters)
    {
        std::cout<<"Angle Start "<<filtro.start<<" Angle End "<<filtro.end<<std::endl;
    }
}

void LidarHandler::prompt_scans()
{
    std::cout<<"Objetos: "<<std::endl;
    for (auto & objeto : Vpubtriangulate.array)
    {
        std::cout<<"Angulo "<<objeto.angle<<" Distancia "<<objeto.dist<<std::endl;
        
    }
    std::cout<<"Robots: "<<std::endl;

    for (auto & objeto : Vpubrobots.array)
    {
        std::cout<<"Angulo "<<objeto.angle<<" Distancia "<<objeto.dist<<std::endl;
        
    }
}


/*
uahr_msgs::array_arcos LidarHandler::pub_filters()
{
    uahr_msgs::array_arcos public_data;
    uahr_msgs::Arco_Interes arc_aux;
    for (auto & filter : SectionsFilters)
    {
        arc_aux.distance_inf = filter.rpose.distance.start;
        arc_aux.distance_sup = filter.rpose.distance.end;
        public_data.arcos.push_back(arc_aux);
    }
    return public_data;
}*/

// Destructor,toda la memoria dinámica la gestiona el compilador 
LidarHandler::~LidarHandler()
{}



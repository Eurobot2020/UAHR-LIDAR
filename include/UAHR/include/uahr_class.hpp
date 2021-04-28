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
        VVPolars Vobjetos;
        VPolars  Vrobots;
        VVPolars Vclusters;

        


    public:
        LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p);
        ~LidarHandler();
        void cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg);
        void new_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
        const float &angle_max, const float &angle_increment);
        void new_scan2(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
        const float &angle_max, const float &angle_increment);

        void prompt_filters();        
        void prompt_scans();
        uahr_msgs::PolarArray   Vpubrobots;
        uahr_msgs::PolarArray   Vpubtriangulate; 
        
        uahr_msgs::array_arcos  pub_filters();


};


// Constructor
LidarHandler::LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p)
: TriangulateObjects{ObjetosBusqueda}, robot{p}
{
    // Reservo la cantidad de memoria
    this->SearchSections.reserve(40); 
    this->Vobjetos.reserve(14); 
    this->Vrobots.reserve(20); 
    this->Vclusters.reserve(15);

    this->Vpubrobots.array.reserve(100);
    this->Vpubtriangulate.array.reserve(ObjetosBusqueda.size());

    // Cálculamos los nuevos filtros
    new_filters(this->SearchSections,this->robot,this->TriangulateObjects);
    //prompt_filters();
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
    // Limpio los vectores:
    this->Vobjetos.clear(),
    this->Vrobots.clear(),
    this->Vpubtriangulate.array.clear(),
    this->Vpubrobots.array.clear();

    // Actualizo la posición del robot:
    Seccion max_distance_enemies(100,3300);
    

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
    
    //prompt_scans();
}

void LidarHandler::new_scan2(rplidar_response_measurement_node_hq_t *nodes, size_t node_count, 
    const float &angle_max, const float &angle_increment)
{
    // Limpio los vectores:
    this->Vclusters.clear(),
    this->Vpubtriangulate.array.clear(),
    this->Vpubrobots.array.clear();

    // Actualizo la posición del robot:
    float max_distance = 3000;
    

    // Calculamos los nuevos filtros:
    AnalyzeScan2(nodes, node_count,
        angle_max, 
        angle_increment,
        this->SearchSections,
        max_distance,
        this->Vclusters,
        this->Vpubtriangulate,
        this->Vpubrobots);
    
    //prompt_scans();
}





void LidarHandler::prompt_filters()
{
    for (auto & filtro : SearchSections)
    {
        std::cout<<"Filtro tipo "<<filtro.tipo<<std::endl;        
        std::cout<<"Filtro motivo "<<filtro.motivo<<std::endl;        
        std::cout<<"Angle Start "<<filtro.rpose.arco.start<<" Angle End "<<filtro.rpose.arco.end<<std::endl;
        std::cout<<"Distance start "<<filtro.rpose.distance.start<<" Angle End "<<filtro.rpose.distance.end<<std::endl;
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

uahr_msgs::array_arcos LidarHandler::pub_filters()
{
    uahr_msgs::array_arcos public_data;
    uahr_msgs::Arco_Interes arc_aux;
    for (auto & filter : SearchSections)
    {
        arc_aux.distance_inf = filter.rpose.distance.start;
        arc_aux.distance_sup = filter.rpose.distance.end;
        arc_aux.angle_inf    = filter.rpose.arco.start;
        arc_aux.angle_sup    = filter.rpose.arco.end;
        arc_aux.motivo       = filter.motivo;
        public_data.arcos.push_back(arc_aux);
    }
    return public_data;
}

// Destructor,toda la memoria dinámica la gestiona el compilador 
LidarHandler::~LidarHandler()
{}



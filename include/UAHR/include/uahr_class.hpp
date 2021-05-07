#pragma once
#include "uahr_analyze.hpp"
#include "uahr_filters.hpp"
#include "uahr_types.hpp"
#include "uahr_common.hpp"
#include "std_srvs/Empty.h"

#define MIN_DIST_ROBOT 100


class LidarHandler
{
    private:
        pose            robot;
        const int       angle_zero_lidar_to_front;
        VSearchObjects  TriangulateObjects;
        VFiltros        SectionsFilters;
        VVPolars        Vclusters;
        bool            robot_localised;

    public:
        LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p,int zero);
        ~LidarHandler();
        
        uahr_msgs::PolarArray   Vpubrobots;
        uahr_msgs::PolarArray   Vpubtriangulate; 

        void cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg);
        void new_scan
        (rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
        const float &angle_max, const float &angle_increment);
        bool reset_tracks(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);


        void prompt_filters();        
        void prompt_scans();


        //uahr_msgs::array_arcos  pub_filters();
};


// Constructor
LidarHandler::LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p,int zero)
: TriangulateObjects{ObjetosBusqueda}, robot{p},angle_zero_lidar_to_front{zero}
{
    // Reservo la cantidad de memoria
    this->SectionsFilters.reserve(6); 
    this->Vclusters.reserve(15);
    this->Vpubrobots.array.reserve(10);
    this->Vpubtriangulate.array.reserve(ObjetosBusqueda.size());

    this->robot.theta = M180(robot.theta-zero);
    
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
    for (auto & obj : TriangulateObjects)
    {
        obj.rpose = obj.theoric_rpose;
    }
    prompt_filters();
}

void LidarHandler::cb_pose(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    // Actualizo la posición del robot:
    this->robot.x = msg->x;
    this->robot.y = msg->y;
    this->robot.theta = M180(msg->theta - this->angle_zero_lidar_to_front); 
    // Calculamos los nuevos filtros:
    UpdateFilters(this->robot_localised,this->SectionsFilters,this->robot,this->TriangulateObjects);    
    prompt_filters();
}

void LidarHandler::new_scan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count, 
    const float &angle_max, const float &angle_increment)
{
    //uahr_msgs::Polar aux;
    
    // Limpio los vectores:
    //std::fill(Vpubtriangulate.array.begin(),Vpubtriangulate.array.end(),aux);
    this->Vpubrobots.array.clear();
    this->Vclusters.clear();

   
    Seccion search_distance;
    search_distance.start = MIN_DIST_ROBOT;
    search_distance.end = biggest_distance_corner(this->robot);

    

    // Calculamos los nuevos filtros:
    AnalyzeScan(nodes, node_count,
        angle_max, 
        angle_increment,
        this->SectionsFilters,
        search_distance,
        this->TriangulateObjects,
        this->Vclusters,
        this->Vpubtriangulate,
        this->Vpubrobots);

    //prompt_scans();
    //prompt_filters();
}

void LidarHandler::prompt_filters()
{
    for (auto & filtro : SectionsFilters)
    {
        std::cout<<"Angle Start "<<filtro.start<<" Angle End "<<filtro.end<<std::endl;
    }

    for (auto & objeto : TriangulateObjects)
    {
        std::cout<<"Objeto ID: "<<objeto.id<<"Real Angle "<< objeto.rpose.angle <<" Real Dist "<<objeto.rpose.dist<<std::endl;
        std::cout<<"Objeto ID: "<<objeto.id<<"Theoric Angle "<< objeto.theoric_rpose.angle <<" Real Dist "<<objeto.theoric_rpose.dist<<std::endl;
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

bool LidarHandler::reset_tracks(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    for (auto & obj : TriangulateObjects)
    {
        obj.rpose = obj.theoric_rpose;
    }
    return true;
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



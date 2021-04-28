#include "uahr_analyze.hpp"


void StoreScan(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
    const float &angle_max, const float &angle_increment,std::ofstream &text)
{
    float angulo;
    float distance;
    text.open("WUENOS DIAS WUENOS DIAS", std::ios::ate);

    for (size_t i = 0; i < node_count; i++)
    {
        //std::cout<<"Filtro "<<filtro->rpose.arco.start<<" "<<filtro->rpose.arco.end<<std::endl;
        angulo = angle_max - angle_increment * i;
        distance = (float) nodes[i].dist_mm_q2/4.0f;
        text<<"Angle: "<<angulo<<" Dist: "<<distance<<std::endl;
    }
    text.close();
}



// Filtro por 
void FilterScan(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
    const float &angle_max, const float &angle_increment,
    VFiltros &Vfiltros, Seccion &enemy_distance,
    VVPolars &Vobjetos, VPolars &Vrobots)
{
    float angulo;
    float distance;
    auto filtro = (Vfiltros.end()-1);

    for (size_t i = 0; i < node_count; i++)
    {
        //std::cout<<"Filtro "<<filtro->rpose.arco.start<<" "<<filtro->rpose.arco.end<<std::endl;

        angulo = angle_max - angle_increment * i;
        if (InSection(angulo,filtro->rpose.arco))
        {
            // Calculamos el valor de la distancia
            distance = (float) nodes[i].dist_mm_q2/4.0f;
            switch (filtro->motivo)
            {
                case OBJETO:
                    if(InSection(distance,filtro->rpose.distance))
                    {
                        if ((filtro->salto==true) && (angulo<180))
                            Vobjetos[filtro->tipo].emplace_back(angulo+360,distance);
                        else
                            Vobjetos[filtro->tipo].emplace_back(angulo,distance);
                    }   
                break;
                
                case ROBOT:
                    if(InSection(distance,enemy_distance))
                    {
                        Vrobots.emplace_back(angulo,distance);
                    }   
                break;
                
                case AMBOS:
                    if(InSection(distance,filtro->rpose.distance))
                    {
                        if (filtro->salto && angulo<180)
                            Vobjetos[filtro->tipo].emplace_back(angulo+360,distance);
                        else
                            Vobjetos[filtro->tipo].emplace_back(angulo,distance);
                    } 
                    else if(InSection(distance,enemy_distance))
                    {
                        Vrobots.emplace_back(angulo,distance);
                    }   
                break;
            }
        }
        
        else if((filtro != Vfiltros.begin())&&(angulo<filtro->rpose.arco.start))
            --filtro;
    }

}


void RPLidarScanToCluster(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
        const float &angle_max, const float &angle_increment, VVPolars &Vclusters,
        VFiltros &Vfiltros,float max_distance)
{
    float angulo;
    float distance;
    auto filtro = (Vfiltros.end()-1);
    int j;
    bool cluster_find=false;

    for (size_t i = 0; i < node_count; i++)
    {
        angulo = angle_max - angle_increment * i;
        
        // El ángulo es correcto?
        if (InSection(angulo,filtro->rpose.arco))
        {
            
            // La distancia es valida?
            distance = (float) nodes[i].dist_mm_q2/4.0f;

            if((distance<max_distance)&&(distance>200))
            {
                cluster_find =false;
                // Recorro los clusters hasta que me paso la condición del ángulo

                for (int j= Vclusters.size()-1;j>=0;j--)
                {
                    if((angulo-Vclusters[j].back().angle)>-5)
                    {
                        if (abs(distance-Vclusters[j].back().dist) <100)
                        {
                            Vclusters[j].emplace_back(angulo,distance);
                            cluster_find=true;
                            break;
                        }   
                    }
                    else break;
                }
            
                /*
                for (int j= 0;j<Vclusters.size();++j)
                {
                    if((angulo-Vclusters[j].back().angle)<5)
                    {
                        if (abs(distance-Vclusters[j].back().dist) <100)
                        {
                            Vclusters[j].emplace_back(angulo,distance);
                            cluster_find=true;
                            break;
                        }
                    }
                    else break;
                }
                */
                if(!cluster_find)
                {
                    Vclusters.emplace_back();
                    Vclusters.back().emplace_back(angulo,distance);
                }

                /*
                while ((angulo-Vclusters[j].back().angle)>-5 || (j!=0))
                {
                    std::cout<<"113 "<<std::endl;

                    if (abs(distance-Vclusters[j].back().dist) <100)
                    {
                        std::cout<<"Filtro "<<std::endl;
                        Vclusters[j].emplace_back(angulo,distance);
                        cluster_find=true;
                        break;
                    }
                    --j;
                }
                
                /*
                // Si el ángulo está cerca de 180 compruebo que no pertenezca a un cluster
                // del principio
                j=0;
                while ((angulo-Vclusters[j].back().angle)<5 || (j!=Vclusters.size()-1))
                {
                    if (abs(distance-Vclusters[j].back().dist) <100)
                    {
                        Vclusters[j].emplace_back(angulo,distance);
                        cluster_find=true;
                        break;
                    }
                    ++j;
                }
                if(!cluster_find)
                {
                    Vclusters.emplace_back();
                    Vclusters.back().emplace_back(angulo,distance);
                }
                */
            }
        }
        else if((filtro != Vfiltros.begin())&&(angulo<filtro->rpose.arco.start))
            --filtro;
    }
}

/*
// Assing clusters
void AssingClusters(VPolars &Vclusters,VPolars &objects)
{
    float min;
    float incert;

    for (auto & objeto : objects)
    {
        for(VPolars)

    
    }
}*/

bool FilterClusterbySize(VPolars & cluster)
{ 
    if ((cluster.size()>1)
        &&     
        (((cluster.back().dist>1200) && (cluster.size()<4))
        ||
        ((cluster.back().dist>800) && (cluster.size()<6))
        ||
        ((cluster.back().dist<=800) && (cluster.size()<22))))
            return true;
    else
        return false;
}


uahr_msgs::Polar MeanCluster(VPolars cluster)
{
    uahr_msgs::Polar p;
    for(auto & haz : cluster)
    {
        p.angle += haz.angle;
        p.dist  += haz.dist;
    }
    p.angle /= cluster.size();
    p.dist  /= cluster.size();
    return p;
}



void AnalyzeScan2(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
                const float &angle_max, const float &angle_increment,
                VFiltros & Vfiltros,  float max_distance,
                VVPolars &Vclusters, uahr_msgs::PolarArray  &Vpubposes,  
                uahr_msgs::PolarArray &Vpubrobots)
{
    //Filtro por forma

    RPLidarScanToCluster(nodes,node_count,angle_max, angle_increment,
                Vclusters,Vfiltros,max_distance);
    


    VVPolars Vclusters2;
    Vclusters2.reserve(Vclusters.size());
    
    
    /*std::copy_if(Vclusters.begin(),Vclusters.end(),std::back_inserter(Vclusters2),[&](VPolars  cluster)
    {
        if (((cluster.back().dist>1200) && (cluster.size()<4))
        ||
        ((cluster.back().dist>800) && (cluster.size()<6))
        ||
        ((cluster.back().dist<=800) && (cluster.size()<20)))
        {
            return true;
        }
        else
        {
            return false;
        }
    });*/

    // Hago la media de los clusters
    for (auto & cluster : Vclusters)
    {
        std::cout<<"Size "<<cluster.size()<<std::endl;
        uahr_msgs::Polar polar = MeanCluster(cluster);
        Vpubrobots.array.emplace_back(polar);
        std::cout<<"Angulo "<<polar.angle<<" Distancia "<<polar.dist<<std::endl;
    }
}


// Analisis objetos
void ObjectClusters(VVPolars &Vscans,uahr_msgs::PolarArray &Vposes)
{
    //std::cout<<"Objetos crudos "<<std::endl;

    for (auto & objeto : Vscans)
    {
        // Creo el polar al objeto 
        // el constructor lo inicaliza
        // a 0,0
        

        Vposes.array.emplace_back();
        if(objeto.size())
        {
            auto &center = Vposes.array.back();
            // Cáculo la media, podria cambiarse 
            // por un filtro exigente:

            for (auto & polar : objeto)
            {
                //std::cout<<"Angulo "<<polar.angle<<" Distancia "<<polar.dist<<std::endl;

                center.dist  += polar.dist;
                center.angle += polar.angle;
            }
            center.dist  /= objeto.size();
            center.angle /= objeto.size();
        }
    }
}

void EnemyClusters(VPolars &Vscans, uahr_msgs::PolarArray &Vrobots)
{

    //std::cout<<"OBJETOS QUE SE VAN A CLUSTERIZAR"<<std::endl;
    //std::sort(Vscans.begin(),Vscans.end(),Compare_dist);    
    /*for (int i = 0; i<Vscans.size(),++i)
    {
        if(Vscans.[i])
            Vrobots.array.push_back()
    }*/
    VVPolars posible_clusters;
    bool cluster_find;
    // Cluster cutre
    for (auto & objeto : Vscans)
    {
        cluster_find = false;
        //std::cout<<"Angulo "<<objeto.angle<<" Distancia "<<objeto.dist<<std::endl;
      
        for(auto & cluster : posible_clusters)
        {
            if (abs(objeto.dist-cluster.back().dist) <100
                && abs(objeto.angle-cluster.back().angle)<5)
            {
                cluster.emplace_back(objeto);
                cluster_find = true;
                break;
            }
        }
        if (!cluster_find)
        {
            posible_clusters.emplace_back();
            posible_clusters.back().emplace_back(objeto);
        }
    }

    // Correct shape


    float last_dist; 
    /*
    for (int i=0; i < posible_clusters.size();++i)
    {

        std::cout<<"El cluster "<<i<<"cuenta con " << posible_clusters[i].size()<<" haces y esta compuesto por: "<<std::endl;
        for (auto & objeto : posible_clusters[i])
        {
            std::cout<<"Angulo "<<objeto.angle<<" Distancia "<<objeto.dist<<std::endl;
        }
    }*/
        
    for(auto & cluster : posible_clusters)
    {
        last_dist = cluster.back().dist;   
        uahr_msgs::Polar p;

        if (((last_dist>1200) && (cluster.size()<4))
            ||
            ((last_dist>800) && (cluster.size()<6))
            ||
            (last_dist<=800))
        {
            for(auto & haz : cluster)
            {
                p.angle += haz.angle;
                p.dist  += haz.dist;
            }
            p.angle /= cluster.size();
            p.dist  /= cluster.size();
            Vrobots.array.push_back(p);    
        }
    }
}

void AnalyzeScan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
                const float &angle_max, const float &angle_increment,
                VFiltros & Vfiltros,  Seccion &enemy_distance,
                VVPolars & Vobjetos,  VPolars &Vrobots,
                uahr_msgs::PolarArray  &Vpubposes,  uahr_msgs::PolarArray &Vpubrobots )
{
    FilterScan(nodes,node_count,angle_max, angle_increment,
            Vfiltros,enemy_distance,Vobjetos, Vrobots);
    ObjectClusters(Vobjetos, Vpubposes);
    EnemyClusters(Vrobots,Vpubrobots);
}

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


void RPLidarScanToCluster(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
        const float &angle_max, const float &angle_increment, VVPolars &Vclusters,
        VFiltros &Vfiltros,Seccion distance_limits)
{
    float angulo;
    float distance;
    auto filtro = (Vfiltros.end()-1);
    bool cluster_find=false;

    for (size_t i = 0; i < node_count; i++)
    {
        angulo = angle_max - angle_increment * i;
        
        // El ángulo es correcto?
        if (InSection(angulo,*filtro))
        {
            // La distancia es valida?
            distance = (float) nodes[i].dist_mm_q2/4.0f;
            if(InSection(distance,distance_limits))
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
                    else 
                        //std::cout<<"No entro por: "<<angulo-Vclusters[j].back().angle<<std::endl;

                        break;
                }

                if(!cluster_find)
                {
                    Vclusters.emplace_back();
                    Vclusters.back().emplace_back(angulo,distance);
                }

            }
        }
        else if((angulo<filtro->start)&&(filtro != Vfiltros.begin()))
            --filtro;
    }
}


// Assing clusters
uahr_msgs::Polar AssingClustertoObject(ObjSearchData  &obj,VPolars &Vclusters)
{
    uahr_msgs::Polar p;
    float difference;
    float difference_min = 31000;
    int   index = -1;
    bool  in_the_midle = false;
    std::vector<int> n;

    if(abs(obj.rpose.angle) >= 170)
    {
        float dist_ang = 0;
        for(int j=0; j<Vclusters.size();j++)
        {
            dist_ang = abs(Vclusters[j].angle) - abs(obj.rpose.angle);
            if((InLimits(Vclusters[j].dist,obj.rpose.dist-50,obj.rpose.dist+50))
                &&
                (InLimits(dist_ang, -10, 10)))
            {
                // TODO: Es esto una buena forma de medir?
                difference = abs((Vclusters[j].angle - obj.rpose.angle) * 
                                (Vclusters[j].dist - obj.rpose.dist));
                n.push_back(j);
                if(difference<difference_min)
                {
                    difference_min = difference;
                    index = j;
                }
            }
        }
    }

    else
    {
        for(int j=0; j<Vclusters.size();j++)
        {
            if(obj.tracked)
            {
                if((InLimits(Vclusters[j].angle, obj.rpose.angle-5,obj.rpose.angle+5)
                    &&
                    Vclusters[j].dist < obj.rpose.dist+50))
                {    
                    in_the_midle = true;
                    if( Vclusters[j].dist > obj.rpose.dist -50)
                    {
                        // TODO: Es esto una buena forma de medir?
                        difference = abs((Vclusters[j].angle - obj.rpose.angle) * 
                                        (Vclusters[j].dist - obj.rpose.dist));
                        n.push_back(j);
                        if(difference<difference_min)
                        {
                            difference_min = difference;
                            index = j;
                        }
                    }
                }
            }
            else
            {
                if((InLimits(Vclusters[j].angle, obj.rpose.angle-10,obj.rpose.angle+10)
                &&
                Vclusters[j].dist < obj.rpose.dist+100))
                {    
                    in_the_midle = true;
                    if( Vclusters[j].dist > obj.rpose.dist -100)
                    {
                        // TODO: Es esto una buena forma de medir?
                        difference = abs((Vclusters[j].angle - obj.rpose.angle) * 
                                        (Vclusters[j].dist - obj.rpose.dist));
                        n.push_back(j);
                        if(difference<difference_min)
                        {
                            difference_min = difference;
                            index = j;
                        }
                    }
                }

            }
            
        }
    }
    
    if(index != -1)
    {
        obj.tracked = true;
        p.dist = Vclusters[index].dist;
        p.angle = Vclusters[index].angle;
        obj.rpose.dist  = p.dist;
        obj.rpose.angle = p.angle;
        // Filtro clusters proximos:
        for(int a=n.size()-1;a>=0;--a)
            Vclusters.erase(Vclusters.begin()+n[a]);
    }
    else
    {
        obj.tracked = false;
        if(!in_the_midle)
        {
            obj.rpose.dist  = obj.theoric_rpose.dist;
            obj.rpose.angle = obj.theoric_rpose.angle;
        }
    }
    return p;
}

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


polar MeanCluster(VPolars cluster)
{
    polar p(0,0);
    for(auto & haz : cluster)
    {
        p.angle += haz.angle;
        p.dist  += haz.dist;
    }
    p.angle /= cluster.size();
    p.dist  /= cluster.size();
    return p;
}


void AnalyzeScan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
                const float &angle_max, const float &angle_increment,
                VFiltros & Vfiltros, const Seccion max_distance,
                VSearchObjects & SearchObjects, VVPolars &Vclusters, 
                uahr_msgs::PolarArray  &Vpubposes,  
                uahr_msgs::PolarArray  &Vpubrobots)
{
    // Hago cluster de puntos:
    RPLidarScanToCluster(nodes,node_count,angle_max, angle_increment,
                Vclusters,Vfiltros,max_distance);
    
    // Hago la media de los cluster sobrantes:
    VPolars medidas_cluster;
    medidas_cluster.reserve(Vclusters.size());
    for (VPolars & cluster : Vclusters)
    {
        medidas_cluster.emplace_back(MeanCluster(cluster));
    }

    if(medidas_cluster.size())
    {
        uahr_msgs::Polar aux_polar;
        // Asocia la distancia de cada cluster con un objeto:
        std::for_each(SearchObjects.begin(),SearchObjects.end(),[&](ObjSearchData & obj)
        {
            aux_polar = AssingClustertoObject(obj,medidas_cluster);
            if(obj.tracked)
            {
                if(!(((aux_polar.angle - Vpubposes.array[obj.id].angle) < 3) 
                    && ((aux_polar.dist - Vpubposes.array[obj.id].dist) < 7)))
                    Vpubposes.array[obj.id] = aux_polar;
            }
            
            // Filtro ruido:
            else
            {
                Vpubposes.array[obj.id] = aux_polar;                
            }
        });
        
        for (auto & cluster : medidas_cluster)
        {
            aux_polar.angle = cluster.angle;
            aux_polar.dist  = cluster.dist;
            Vpubrobots.array.emplace_back(aux_polar);
        }
    }
}

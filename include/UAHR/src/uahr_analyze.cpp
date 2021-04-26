#include "uahr_analyze.hpp"

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
                        if (filtro->salto && angulo<180)
                            
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
        
        else if(filtro != Vfiltros.begin())
            --filtro;
    }
}



// Analisis objetos
void ObjectClusters(VVPolars &Vscans,uahr_msgs::PolarArray Vposes)
{
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
                center.dist  += polar.dist;
                center.angle += polar.angle;
            }
            center.dist  /= objeto.size();
            center.angle /= objeto.size();
        }
    }
}

void enemy_clusters(VPolars &Vscans, VPolars &Venemies)
{
    int a =0;
    // Doble filtro:
    /*
    uahr_msgs::PolarArray;
    cluster
    // Recorro el véctor en busca de posibles clusters
    for (auto & objeto : Vobjetos)
    {

        // Esta cerca en ángulo de alguno de los últimos clusters?
        for (auto & recent_angles : near_clusters)
        {
            if(posible_clusters.distance- objeto.distance)
            {

            }
        }
        // Esta cerca en distancia de un 

        // Hay algún cluster cercano

    }
    for ()
    */
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
    //EnemyClusters(VVPolarrs  Vrobots,  VPolars Vpubrobots);
}

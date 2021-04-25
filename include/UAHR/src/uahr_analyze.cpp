#include "uahr_analyze.hpp"

// Filtro por 
void FilterScan(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
    &VFiltros Vfiltros, &Seccion enemy_distance,
    &VVPolars Vobjetos, &VPolars Vrobots)
{
    float angulo;
    float distance;
    auto filtro = (Vfiltros.end()-1);
    for (size_t i = 0; i < node_count; i++)
    {
        angulo = angle_max - angle_increment * i;
        if (InSection(angulo,filtro.angle))
        {
            // Calculamos el valor de la distancia
            distance = (float) nodes[i].dist_mm_q2/4.0f;
            switch (filter->motivo)
            {
                case OBJETO:
                    if (InSection(distance,filtro->distance))
                    {
                        if (filtro->salto && angulo<180)
                            Vobjetos[filtro->type].emplace_back(angulo+360,distance);
                        else
                            Vobjetos[filtro->type].emplace_back(angulo,distance);
                    }   
                break;
                
                case ROBOT:
                    if(InSection(distance,enemy_distance))
                    {
                        Vrobots.emplace_back(angulo,distance);
                    }   
                break;
                
                case AMBOS:
                    if (InSection(distance,filtro.distance))
                    {
                        if (filtro.salto && angulo<180)
                            Vobjetos[filtro->type].emplace_back(angulo+360,distance);
                        else
                            Vobjetos[filtro->type].emplace_back(angulo,distance);
                    }   
                    else if(InSection(distance,enemy_distance))
                    {
                        Vrobots.emplace_back(angulo,distance);
                    }   
                break;
            }
        }
        
        else if(filtro != VFiltros.begin())
            --filtro;
    }
}



// Analisis objetos
void objeto_clusters(&VVPolars Vscans,&VPolars Vposes)
{
    for (auto & objeto : Vscans)
    {
        // Creo el polar al objeto
        Vposes.emplace_back(0,0);
        if(objeto.size())
        {
            auto center = Vpose.back()
            // Cáculo la media, podria cambiarse 
            // por un filtro exigente:
            for (auto & polar : objeto)
            {
                center.dist  += objeto.dist;
                center.angle += objeto.angle
            }
            center.dist  /= objeto.size();
            center.angle /= objeto.size();
        }
    }
}

void enemy_clusters(&VPolars Vscans, &VPolars Venemies)
{
    // Doble filtro:
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

}

void AnalyzeScan(rplidar_response_measurement_node_hq_t *nodes, size_t node_count,
                &VFiltros Vfiltros,  &Seccion enemy_distance,
                &VVPolars Vobjetos,  &VPolars Vrobots,
                &VPolars Vpubposes,  &VPolars Vpubrobots )
{
    FilterScan(rplidar_response_measurement_node_hq_t *nodes,size_t node_count,
            VFiltros Vfiltros,  Seccion enemy_distance,
            VVPolars Vobjetos,  VVPolars Vrobots);
            
    ObjectClusters(VVPolarrs Vobjetos, VPolars Vpubposes);
    EnemyClusters(VVPolarrs  Vrobots,  VPolars Vpubrobots);
}

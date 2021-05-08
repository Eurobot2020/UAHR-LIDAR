#include <uahr_filters.hpp>
#include <math.h>


void DangerAngles1C(const pose &pr, VFiltros &vdf)
{
    /*
        Esta función calcula los ángulos en los
        que hay que puede que hay un robot enemigo
        si estamos en el primer cuadrante.

        Para ello esta función recibe un vector vacio.
    */
    float fx = 0;
    float fy = 0;

    // Calculo los angulos que hay que filtrar:
    if((LIMX - pr.x) < DISTANCIA_SEGURIDAD) 
    {
        fx = RAD2DEG(acos((LIMX - pr.x)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(fx,-fx);
    }
    if((LIMY - pr.y)< DISTANCIA_SEGURIDAD) 
    {
        fy = RAD2DEG(asin((LIMY - pr.y)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(180-fy,fy);            
    }

    // Caso especial no hay ángulos seguros en esta posición:
    if(!vdf.size())
    {
        vdf.emplace_back(-180,180);
        return;
    }
    // Caso especial hay entrelazamiento entre los dos ángulos:
    else if((vdf.size()==2) && (vdf[0].start > vdf[1].end))
    {
        vdf[0].start = vdf[1].start;
        vdf.pop_back();
    }

    return;
}

void DangerAngles2C(const pose &pr, VFiltros &vdf)
{
    /*
        Esta función calcula una serie de ángulos en 
        los que no es posibe que haya riesgo de 
        colisión con el robot enemigo.
    */
    float fx = 0;
    float fy = 0;

    // Calculo los angulos que hay que filtrar:
    if((pr.x + LIMX) < DISTANCIA_SEGURIDAD) 
    {
        fx = RAD2DEG(acos((-LIMX - pr.x)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(-fx,fx);
    }
    if((LIMY - pr.y) < DISTANCIA_SEGURIDAD) 
    {
        fy = RAD2DEG(asin((LIMY - pr.y)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(180-fy,fy);            
    }

    // Caso especial no hay ángulos seguros en esta posición:
    if(!vdf.size())
    {
        vdf.emplace_back(-180,180);
        return;
    }
    // Caso especial hay entrelazamiento entre los dos ángulos:
    else if((vdf.size()==2) && (vdf[1].start>=vdf[0].end))
    {
        vdf[0].end = vdf[1].end;
        vdf.pop_back();
    }

    return;
}


void DangerAngles3C(const pose &pr, VFiltros &vdf)
{
    /*
        Esta función calcula los ángulos en los
        que hay que puede que hay un robot enemigo
        si estamos en el primer cuadrante.

        Para ello esta función recibe un vector vacio.
    */
    float fx = 0;
    float fy = 0;

    // Calculo los angulos que hay que filtrar:
    if((pr.x + LIMX) < DISTANCIA_SEGURIDAD) 
    {
        fx = RAD2DEG(acos((-LIMX - pr.x)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(-fx,fx);
    }
    if((pr.y + LIMY) < DISTANCIA_SEGURIDAD) 
    {
        fy  = RAD2DEG(asin((-LIMY - pr.y)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(fy,-180 - fy);        
    }

    // Caso especial no hay ángulos seguros en esta posición:
    if(!vdf.size())
    {
        vdf.emplace_back(-180,180);
        return;
    }
    else if((vdf.size()==2) && (vdf[0].start >= vdf[1].end))
    {
        vdf[0].start = vdf[1].start;
        vdf.pop_back();
    }
    return;
}

void DangerAngles4C(const pose &pr, VFiltros &vdf)
{
    /*
        Esta función calcula una serie de ángulos en 
        los que no es posibe que haya riesgo de 
        colisión con el robot enemigo.
    */
    float fx = 0;
    float fy = 0;


    // Calculo los angulos que hay que filtrar:
    if((LIMX - pr.x) < DISTANCIA_SEGURIDAD) 
    {
        fx = RAD2DEG(acos((LIMX - pr.x)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(fx,-fx);
    }
    if((pr.y + LIMY) < DISTANCIA_SEGURIDAD) 
    {
        fy  = RAD2DEG(asin((-LIMY - pr.y)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(fy,-180 - fy);        
    }

    // Caso especial no hay ángulos seguros en esta posición:
    if(!vdf.size())
    {
        vdf.emplace_back(-180,180);
        return;
    }
    // Caso especial no hay ángulos seguros en esta posición:
    else if((vdf.size()==2) && (vdf[1].start>=vdf[0].end))
    {
        vdf[0].end = vdf[1].end;
        vdf.pop_back();
    }
    return;
}


void ObjectsAngles(const pose &p_obs,ObjSearchData &pasive_obj,VFiltros &vf)
{
    /*

        Calcula la corona angular en la que se debe 
        encontar el objeto, esta corona es relativa
        a la posición del observador.

        p_obs      = posición del observador.
        pasive_obj = objeto sobre el que se quiere 
                     triangular.
    */ 

    float ao,dx,dy;
    
    // Calculo la distancia desde el observador
    // hasta las distintas partes del objeto:
    dx = pasive_obj.x - p_obs.x;
    dy = pasive_obj.y - p_obs.y;
    ao  = M180(RAD2DEG(atan2(dy,dx)));
    pasive_obj.theoric_rpose.angle = M180(ao - p_obs.theta);
    pasive_obj.theoric_rpose.dist = sqrt(dx*dx+dy*dy);
    // Intento construir el elemento directamente,
    // en el heap
    vf.emplace_back(ao-FA,ao+FA);
}

void AcoploAngulos(VFiltros &Vf)
{   
    int i = 0;

    // TODO
    // Podria intentar ser más rapida si cambiamos el erase.
    // pero de momento da robustez ante muchos casos
    while(i<Vf.size()-1)
    {
        // Existe acoplamiento entre dos angulos de interes?
        if(Vf[i+1].start<=Vf[i].end)
        {
            // El primer filtro solo contiene parte del segundo filtro
            if(Vf[i].end<Vf[i+1].end)
            {
                Vf[i].end = Vf[i+1].end;
            }

            Vf.erase(Vf.begin()+i+1);
        }
        else i++;        
    }
}




void RelativeAngle(Seccion &f ,const pose &pr,VFiltros &vdf)
{
    if((f.start !=-180) && (f.end != 180))
    {
    
        f.start = M180(f.start - pr.theta);
        f.end   = M180(f.end   - pr.theta);

        // Si al hacerlos relativos pasan desde 180
        // a -180 hay que partirlos
        if(f.start > f.end)                   
        {
            if(f.start != 180)
            {
                if(f.end != -180)
                {    
                    // Si no se hace asi puede que 
                    // haya una relocalización
                    // y el puntero se eche a perder
                    float aux = f.start;
                    f.start = -180.0;
                    vdf.emplace_back(aux,180);
                }  
                else
                    f.end = 180;
            }
            else
                f.start = -180;
        }
    }
}


void UpdateFilters(const bool & robot_localised,VFiltros &filtros,
    pose const &robot,VSearchObjects &SearchObjects)
{    
    filtros.clear();
    if (!robot_localised)
    {
        // Calculo los angulos que estan fuera del campo
        if((robot.x>=0)&&(robot.y>=0))
            DangerAngles1C(robot,filtros);
        else if((robot.x<=0)&&(robot.y>=0))
            DangerAngles2C(robot,filtros);
        else if((robot.x<=0)&&(robot.y<=0))
            DangerAngles3C(robot,filtros);        
        else if((robot.x>=0)&&(robot.y<=0))
            DangerAngles4C(robot,filtros);        
    }
    //filtros.emplace_back(-180,180);
    //VObjRdistance.emplace_back();

    // Calculo donde espero ver los objetos
    /*std::for_each(SearchObjects.begin(),SearchObjects.end(),[&](ObjSearchData obj)
    {
        ObjectsAngles(robot,obj,filtros);
    });*/

    for(auto & obj: SearchObjects)
    {
        ObjectsAngles(robot,obj,filtros);
    }



    // Caso especial que se da con bastante probabilidad
    if((filtros[0].start == -180) && (filtros[0].end == 180))
    {
        filtros.clear();
        filtros.emplace_back(-180,180);
    }

    else
    {
        // Cambio todos los ángulosa ángulos relativos:
        for(int i =SearchObjects.size()-1;i >= 0;--i)
            RelativeAngle(filtros[i],robot,filtros);        
        // Ordenamos los angulos para que analizarlo de manera ordenada desde el lidar:
        std::sort(filtros.begin(),filtros.end(),GenerarAcoples);    

        // Si un ángulo se come a otro los juntamos 
        AcoploAngulos(filtros);
    }
}

float biggest_distance_corner(pose robot)
{
    pose opposite_corner;
    if((robot.x>=0)&&(robot.y>=0))
    {
        opposite_corner.x = -LIMX;
        opposite_corner.y = -LIMY;
    }
    else if((robot.x<=0)&&(robot.y>=0))
    {
        opposite_corner.x = LIMX;
        opposite_corner.y = -LIMY;
    }
    else if((robot.x<=0)&&(robot.y<=0))
    {
        opposite_corner.x = LIMX;
        opposite_corner.y = LIMY;
    }

    else if((robot.x>=0)&&(robot.y<=0))
    {
        opposite_corner.x = -LIMX;
        opposite_corner.y = LIMY;
    }
    return (sqrt((robot.x - opposite_corner.x)*(robot.x - opposite_corner.x) 
    + (robot.y - opposite_corner.y) * (robot.y - opposite_corner.y))+300);

}

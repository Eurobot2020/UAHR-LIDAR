#include <math.h>
#include <algorithm>
#include <vector>
#include <iostream>

#include "UAHRLidarlib.hpp"

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)
#define DISTANCIA_SEGURIDAD 500
#define LIMX 1500
#define LIMY 1000
#define N_2PI 2*M_PI



bool Compare_FiltroAngular(FiltroAngular a1, FiltroAngular a2) {
   if(a1.rpose.arco.start < a2.rpose.arco.start) {
      return true;
    }
   else if(a1.rpose.arco.start == a2.rpose.arco.start) {
        if(a1.rpose.arco.end < a2.rpose.arco.end) 
            return true;
        else
            return false;
    }
    else
        return false;
}



float M180(float angle)
{
    /*
        M180 :: float -> float

        Convierte un angulo a modulo 
        entre -180 y 180.   
    */ 

    // Podria mejorarse con un inline?
    float a = angle;
    while(a>180)
        a = a -360;
    while (a<-180)
       a = a+360;
    return a;
}


void ObjectsAngles(const pose &p_obs,const ObjSearchData &pasive_obj,VFiltros &vf)
{
    /*
        ObjectsAngles :: pose -> ObjSearchData -> Int -> Corona

        Calcula la corona angular en la que se debe 
        encontar el objeto, esta corona es relativa
        a la posición del observador.

        p_obs      = posición del observador.
        pasive_obj = objeto sobre el que se quiere 
                     triangular.
    */ 

    float aoi, aos,adiff,dxi,dyi,di,ds,dxs,dys;
    // Intento construir el elemento directamente,
    // en el heap
    vf.emplace_back(OBJETO,pasive_obj.id);     
    FiltroAngular &f = vf.back();

    // Calculo la distancia desde el observador
    // hasta las distintas partes del objeto:
    dxi = pasive_obj.x_inf - p_obs.x;
    dyi = pasive_obj.y_inf - p_obs.y;
    dxs = pasive_obj.x_sup - p_obs.x;
    dys = pasive_obj.y_sup - p_obs.y;

    // Calculo el ańgulo hasta la parte superior
    // y la parte inferior del objeto:
    aoi  = M180(RAD2DEG(atan2(dyi,dxi)));
    aos  = M180(RAD2DEG(atan2(dys,dxs)));
    adiff = M180(aos-aoi);

    // En el filto aumento un poco los ángulos de 
    // la corona dependiendo de un factor de 
    // seguridad:        
    if(adiff>0)
    {

        f.rpose.arco.start = M180(aoi - FA/adiff * pasive_obj.aexp);
        f.rpose.arco.end   = M180(aos + FA/adiff * pasive_obj.aexp);
    }
    else
    {
        f.rpose.arco.start = M180(aos + FA/adiff * pasive_obj.aexp);
        f.rpose.arco.end   = M180(aoi - FA/adiff * pasive_obj.aexp);
    }
    
    // Calculo la distancia hasta el objeto.
    di = sqrt(dxi*dxi+dyi*dyi);
    ds = sqrt(dxs*dxs+dys*dys);
    f.rpose.distance.start = (1 - pasive_obj.dexp) * (di+ds)/2;
    f.rpose.distance.end   = (1 + pasive_obj.dexp) * (di+ds)/2;
    f.motivo = OBJETO;
    f.tipo = pasive_obj.id;
}

void RelativeAngle(FiltroAngular &f ,const pose &pr,VFiltros &vdf){
    f.rpose.arco.start = M180(f.rpose.arco.start - pr.theta);
    f.rpose.arco.end   = M180(f.rpose.arco.end   - pr.theta);
    
    // Si al hacerlos relativos pasan desde 180
    // a -180 hay que partirlos
    
    
    /// TODO ESTO PUEDE DAR FALLO EN LOS CASOS DE 180 GRADOS
    if(f.rpose.arco.start > f.rpose.arco.end)                   
    {
        
        if(f.rpose.arco.start != 180)
            if(f.rpose.arco.end != -180)
            {    
                vdf.emplace_back(FiltroAngular(
                    Seccion(f.rpose.arco.start,180),
                    f.rpose.distance,
                    f.motivo,
                    f.tipo,
                    true
                    ));     
                f.rpose.arco.start = -180;
                f.salto = true;

            }  

            else
                f.rpose.arco.end = 180;

        else
            f.rpose.arco.start = -180;
    }
}


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
        vdf.emplace_back(Seccion(fx,-fx));
    }
    if((LIMY - pr.y)< DISTANCIA_SEGURIDAD) 
    {
        fy = RAD2DEG(asin((LIMY - pr.y)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(Seccion(180-fy,fy));            
    }

    // Caso especial no hay ángulos seguros en esta posición:
    if(!vdf.size())
    {
        vdf.emplace_back(Seccion(-180,180));
        return;
    }
    // Caso especial hay entrelazamiento entre los dos ángulos:
    else if((vdf.size()==2) && (vdf[0].rpose.arco.start > vdf[1].rpose.arco.end))
    {
        vdf[0].rpose.arco.start = vdf[1].rpose.arco.start;
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
        vdf.emplace_back(Seccion(-fx,fx));
    }
    if((LIMY - pr.y) < DISTANCIA_SEGURIDAD) 
    {
        fy = RAD2DEG(asin((LIMY - pr.y)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(Seccion(180-fy,fy));            
    }

    // Caso especial no hay ángulos seguros en esta posición:
    if(!vdf.size())
    {
        vdf.emplace_back(Seccion(-180,180));
        return;
    }
    // Caso especial hay entrelazamiento entre los dos ángulos:
    else if((vdf.size()==2) && (vdf[1].rpose.arco.start>=vdf[0].rpose.arco.end))
    {
        vdf[0].rpose.arco.end = vdf[1].rpose.arco.end;
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
        vdf.emplace_back(Seccion(-fx,fx));
    }
    if((pr.y + LIMY) < DISTANCIA_SEGURIDAD) 
    {
        fy  = RAD2DEG(asin((-LIMY - pr.y)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(Seccion(fy,-180 - fy));        
    }

    // Caso especial no hay ángulos seguros en esta posición:
    if(!vdf.size())
    {
        vdf.emplace_back(Seccion(-180,180));
        return;
    }
    else if((vdf.size()==2) && (vdf[0].rpose.arco.start >= vdf[1].rpose.arco.end))
    {
        vdf[0].rpose.arco.start = vdf[1].rpose.arco.start;
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
        vdf.emplace_back(Seccion(fx,-fx));
    }
    if((pr.y + LIMY) < DISTANCIA_SEGURIDAD) 
    {
        fy  = RAD2DEG(asin((-LIMY - pr.y)/DISTANCIA_SEGURIDAD));
        vdf.emplace_back(Seccion(fy,-180 - fy));        
    }

    // Caso especial no hay ángulos seguros en esta posición:
    if(!vdf.size())
    {
        vdf.emplace_back(Seccion(-180,180));
        return;
    }
    // Caso especial no hay ángulos seguros en esta posición:
    else if((vdf.size()==2) && (vdf[1].rpose.arco.start>=vdf[0].rpose.arco.end))
    {
        vdf[0].rpose.arco.end = vdf[1].rpose.arco.end;
        vdf.pop_back();
    }
    return;
}


void DesacoploAngulos(VFiltros &Vf)
{   
    int i = 0;
    float aux_angle;
    // Cambiar por punteros?
    while(i<Vf.size()-1)
    {
        // Existe acoplamiento entre dos angulos de interes?
        if(Vf[i+1].rpose.arco.start<Vf[i].rpose.arco.end)
        {
            // El primer filtro solo contiene parte del segundo filtro
            if(Vf[i].rpose.arco.end<Vf[i+1].rpose.arco.end)
            {
                // Si ambos empiezan en el mismo ángulo:
                if(Vf[i].rpose.arco.start == Vf[i+1].rpose.arco.start)
                {
                    Vf[i+1].rpose.arco.start = Vf[i].rpose.arco.end;
                    Vf[i].rpose.distance = Vf[i].rpose.distance + Vf[i+1].rpose.distance;
                    Vf[i].motivo = AMBOS;
                    Vf[i].tipo   = Vf[i].tipo + Vf[i+1].tipo;
                    Vf[i].salto  = Vf[i].salto + Vf[i+1].salto;
                }

                else
                {
                    // Creo el Arco de intersección:
                    Vf.emplace_back(
                        Seccion(Vf[i+1].rpose.arco.start,Vf[i].rpose.arco.end),
                        Vf[i].rpose.distance + Vf[i+1].rpose.distance,
                        AMBOS,
                        Vf[i].tipo + Vf[i+1].tipo,
                        Vf[i].salto + Vf[i+1].salto);
                
                    // Almaceno un ángulo para no borrarlo
                    aux_angle = Vf[i].rpose.arco.end;

                    // Modificamos el primer ángulo:
                    Vf[i].rpose.arco.end = Vf[i+1].rpose.arco.start;

                    // Modificamos el segundo arco
                    Vf[i+1].rpose.arco.start = aux_angle;
                }
            }

            // El segundo filtro esta completamente contenido en el primero
            else if(Vf[i].rpose.arco.end>Vf[i+1].rpose.arco.end)
            {
                // Creamos el arco del final
                Vf.emplace_back(
                    Seccion(Vf[i+1].rpose.arco.end, Vf[i].rpose.arco.end),
                    Vf[i].rpose.distance,
                    Vf[i].motivo,
                    Vf[i].tipo,
                    Vf[i].salto);
                
                // Modificamos el arco del principio 
                Vf[i].rpose.arco.end   = Vf[i+1].rpose.arco.start;
                
                // Modificamos el arco intermedio
                Vf[i+1].tipo           = Vf[i+1].tipo + Vf[i].tipo;
                Vf[i+1].motivo         = AMBOS;
                Vf[i+1].rpose.distance = Vf[i+1].rpose.distance + Vf[i].rpose.distance;
                Vf[i+1].salto          = Vf[i+1].salto + Vf[i].salto;
            }

            else
            {
                // Modificamos el ángulo del arco
                Vf[i].rpose.arco.end  = Vf[i+1].rpose.arco.start;
                
                // Arco de intersección:
                Vf[i+1].rpose.distance    = Vf[i].rpose.distance + Vf[i+1].rpose.distance;
                Vf[i+1].motivo            = AMBOS;
                Vf[i+1].tipo              = Vf[i].tipo  + Vf[i+1].tipo;
                Vf[i+1].salto             = Vf[i].salto + Vf[i+1].salto;
                
                // Un arco de más
                if(Vf[i].rpose.arco.start == Vf[i].rpose.arco.end)
                    Vf.erase(Vf.begin()+i);
            }
            std::sort(Vf.begin()+i,Vf.end(),Compare_FiltroAngular);    
        }
        else i++;        
    }
}


VFiltros new_filters(pose const &robot,VObjetos &lfobjects ){
    // TODO Pasar por referencia
    // TODO Mirar Cache para optimizar el acceso
    VFiltros VObjRdistance;
    VObjRdistance.reserve(20);


    // Calculo los angulos que estan fuera del campo
    if((robot.x>=0)&&(robot.y>=0))
        DangerAngles1C(robot,VObjRdistance);
    else if((robot.x<=0)&&(robot.y>=0))
        DangerAngles2C(robot,VObjRdistance);
    else if((robot.x<=0)&&(robot.y<=0))
        DangerAngles3C(robot,VObjRdistance);        
    else if((robot.x>=0)&&(robot.y<=0))
        DangerAngles4C(robot,VObjRdistance);        

    // Calculo donde espero ver los objetos
    std::for_each(lfobjects.begin(),lfobjects.end(),[&](ObjSearchData  obj)
    {
        ObjectsAngles(robot,obj,VObjRdistance);
    });

    // Cambio todos los ángulosa ángulos relativos:
    std::for_each(VObjRdistance.rbegin(),VObjRdistance.rend(),[&](FiltroAngular f)
    {
        RelativeAngle(f,robot,VObjRdistance);
    });

    // Ordenamos los angulos para que analizarlo 
    // de manera ordenada desde el lidar:
    std::sort(VObjRdistance.begin(),VObjRdistance.end(),Compare_FiltroAngular);    
    
    // Comprobamos que no haya filtrs que
    // comen a otros filtros
    DesacoploAngulos(VObjRdistance);
        
    return VObjRdistance;
}

/*
void ExpandirFiltros(VFiltros &filtros, int objeto){
    
    int upper_filter = 0;
    int lower_filter = -1;

    
    // Busco los ángulos:
    for(int i=0; i<filtros.size(); i++)
    {
        if(filtros[i].tipo == objeto)
        {
            if(filtros[i].salto)
            {
                if( (i != filtros.size()) 
                    && 
                    filtros[i+1].tipo != objeto)
                {
                    lower_filter = i;

                }
                else if ((i != 0) 
                        && 
                        filtros[i-1].tipo != objeto)
                {
                    upper_filter = i;
                }
            }
            else
            {
                upper_filter = i;
                if(lower_filter == -1) lower_filter = i;
            }
        }
    } 

    // Expando:
    // Caso sencillo
    if(upper_filter != filtros.size())
    {
        filtros[i]   +=
        filtros[i+1] += 


    }

}
*/

void ExpandirFiltroSinSalto(VFiltros &filtros, int objeto){
    // Busco los ángulos:
    for (auto &filtro : filtros)
    {
        if(filtro.tipo == objeto)
        {
            
            
        }
    } 
    // 
}


// Funciones para los tests:

bool operator== (const Seccion one, const  Seccion two) {
    return ((int)one.start == (int)two.start && (int)one.end == (int)two.end);
}
Seccion operator+(const Seccion one, const  Seccion two) {
    return Seccion(one.start + two.start, one.end + two.end);
}
bool operator== (const CoronaCircular one, const  CoronaCircular two) {
    return (one.arco == two.arco && one.distance == two.distance);
}
bool operator== (const FiltroAngular one,const FiltroAngular two) {
    return (one.rpose == two.rpose 
        && one.motivo == two.motivo 
        && one.tipo == two.tipo);
}

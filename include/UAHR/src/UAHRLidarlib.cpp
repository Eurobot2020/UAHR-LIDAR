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



CoronaCircular ObjectsAngles(const pose *p_obs,const ObjSearchData *pasive_obj)
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
    struct CoronaCircular obj;
    
    // Calculo la distancia desde el observador
    // hasta las distintas partes del objeto:
    dxi = pasive_obj->x_inf - p_obs->x;
    dyi = pasive_obj->y_inf - p_obs->y;
    dxs = pasive_obj->x_sup - p_obs->x;
    dys = pasive_obj->y_sup - p_obs->y;

    // Cálculo el ańgulo hasta la parte superior
    // y la parte inferior del objeto:
    aoi  = M180(RAD2DEG(atan2(dyi,dxi)) - p_obs->theta);
    aos  = M180(RAD2DEG(atan2(dys,dxs)) - p_obs->theta);
    adiff = M180(aos-aoi);

    // En el filto aumento un poco los ángulos de 
    // la corona dependiendo de un factor de 
    // seguridad:        
    if(adiff>0)
    {
        obj.arco.start = M180(aoi - FA/adiff * pasive_obj->aexp);
        obj.arco.end   = M180(aos + FA/adiff * pasive_obj->aexp);
    }
    else
    {
        obj.arco.start = M180(aos + FA/adiff * pasive_obj->aexp);
        obj.arco.end   = M180(aoi - FA/adiff * pasive_obj->aexp);
    }
    
    // Calculo la distancia hasta el objeto.
    di = sqrt(dxi*dxi+dyi*dyi);
    ds = sqrt(dxs*dxs+dys*dys);
    obj.distance.start = (1 - pasive_obj->dexp) * (di+ds)/2;
    obj.distance.end   = (1 + pasive_obj->dexp) * (di+ds)/2;

    return obj;
}

VFiltros DangerAngles1C(const struct pose *pr)
{
    /*
        Esta función calcula una serie de ángulos en 
        los que no es posibe que haya riesgo de 
        colisión con el robot enemigo.
    */
    float FX_init = 0;
    float FX_fin  = 0;
    float FY_init = 0;
    float FY_fin  = 0;
    int   n = 0;
    int   i;
    Seccion aux_filter[4] = {0};
    VFiltros v_safe;
    FiltroAngular arc;

    
    // Calculo los angulos a que  hay que filtrar:
    if((LIMX - pr->x) < DISTANCIA_SEGURIDAD) 
    {
        FX_fin  = RAD2DEG(acos((LIMX - pr->x)/DISTANCIA_SEGURIDAD));
        FX_init = -FX_fin;
        aux_filter[n].start = FX_fin;
        aux_filter[n].end   = FX_init;
        n++;
    }
    
    if((LIMY - pr->y)< DISTANCIA_SEGURIDAD) 
    {
        FY_init = RAD2DEG(asin((LIMY - pr->y)/DISTANCIA_SEGURIDAD));
        FY_fin = 180 - FY_init;
        //if(FY_fin==0)  FY_fin = 360;
        aux_filter[n].start = FY_fin;
        aux_filter[n].end   = FY_init;
        n++;
    }

    // Si hay entrelazamiento entre los angulos
    // fusionamos el aux_filter
    if(!n)
    {
        arc.rpose.arco.start = -180;
        arc.rpose.arco.end   = 180;
        v_safe.push_back(arc);
        return v_safe;
    }
    else if((n==2) && (FX_fin > FY_init))
    {
        aux_filter[0].start = FY_fin;
        aux_filter[0].end = FX_init;
        n = 1;
    }


    // Lo transformamso a posiciones relativas respecto
    // el cero del lidar:
    for(i = 0; i <n; i++)
    {
        aux_filter[i].start = M180(aux_filter[i].start - pr->theta);
        aux_filter[i].end   = M180(aux_filter[i].end  - pr->theta);
        
        // Si un aux_filter pasa desde 180 a -180 lo partimos
        // en dos partes:
        if(aux_filter[i].start > aux_filter[i].end)                   
        {
            
            arc.rpose.arco.start = -180;
            arc.rpose.arco.end  = aux_filter[i].end;
            v_safe.push_back(arc);
            
            arc.rpose.arco.start = aux_filter[i].start;
            arc.rpose.arco.end  = 180;
            v_safe.push_back(arc);
        }
        else
        {
            arc.rpose.arco = aux_filter[i];
            v_safe.push_back(arc);
        }        
    }
    std::sort(v_safe.begin(), v_safe.end(), Compare_FiltroAngular);
    return v_safe;
}

VFiltros DangerAngles2C(const struct pose *pr)
{
    /*
        Esta función calcula una serie de ángulos en 
        los que no es posibe que haya riesgo de 
        colisión con el robot enemigo.
    */
    float FX_init = 0;
    float FX_fin  = 0;
    float FY_init = 0;
    float FY_fin  = 0;
    int   n = 0;
    int   i;
    Seccion aux_filter[4] = {0};
    VFiltros v_safe;
    FiltroAngular arc;

    // Inicio la estructura
    arc.rpose.distance = 0;
    arc.motivo = ROBOT;
    arc.tipo = 0;
    
    // Calculo los angulos a que  hay que filtrar:
    if((pr->x + LIMX) < DISTANCIA_SEGURIDAD) 
    {
        FX_init  = RAD2DEG(acos((-LIMX - pr->x)/DISTANCIA_SEGURIDAD));
        FX_fin = -FX_init;
        aux_filter[n].start = FX_fin;
        aux_filter[n].end   = FX_init;
        n++;
    }
    
    if((LIMY - pr->y) < DISTANCIA_SEGURIDAD) 
    {
        FY_init  = RAD2DEG(asin((LIMY - pr->y)/DISTANCIA_SEGURIDAD));
        FY_fin   = 180 - FY_init;
        aux_filter[n].start = FY_fin;
        aux_filter[n].end = FY_init;
        n++;        
    }


    // Si hay entrelazamiento entre los angulos
    // fusionamos el aux_filter
    if(!n)
    {
        arc.rpose.arco.start = -180;
        arc.rpose.arco.end   = 180;
        v_safe.push_back(arc);
        return v_safe;
    }
    else if((n==2) && (FY_fin>=FX_init))
    {
        aux_filter[0].start = FX_fin;
        aux_filter[0].end   = FY_init;
        n = 1;
    }

    // Lo transformamso a posiciones relativas respecto
    // el cero del lidar:
    for(i = 0; i <n; i++)
    {
        aux_filter[i].start = M180(aux_filter[i].start - pr->theta);
        aux_filter[i].end   = M180(aux_filter[i].end  - pr->theta);
        
        // Si un aux_filter pasa desde 180 a -180 lo partimos
        // en dos partes:
        if(aux_filter[i].start > aux_filter[i].end)                   
        {
            if(aux_filter[i].end != -180)
            {
                arc.rpose.arco.start = -180;
                arc.rpose.arco.end  = aux_filter[i].end;
                v_safe.push_back(arc);
            }
            
            arc.rpose.arco.start = aux_filter[i].start;
            arc.rpose.arco.end  = 180;
            v_safe.push_back(arc);
        }
        else
        {
            arc.rpose.arco = aux_filter[i];
            v_safe.push_back(arc);
        }        
    }
    std::sort(v_safe.begin(), v_safe.end(), Compare_FiltroAngular);
    return v_safe;
}



VFiltros DangerAngles3C(const struct pose *pr)
{
    /*
        Esta función calcula una serie de ángulos en 
        los que no es posibe que haya riesgo de 
        colisión con el robot enemigo.
    */
    float FX_init = 0;
    float FX_fin  = 0;
    float FY_init = 0;
    float FY_fin  = 0;
    int   n = 0;
    int   i;
    Seccion aux_filter[4] = {0};
    VFiltros v_safe;
    FiltroAngular arc;

    // Inicio la estructura
    arc.rpose.distance = 0;
    arc.motivo = ROBOT;
    arc.tipo = 0;

    if((pr->x + LIMX) < DISTANCIA_SEGURIDAD) 
    {
        FX_init = RAD2DEG(acos((-LIMX - pr->x)/DISTANCIA_SEGURIDAD));
        FX_fin  = -FX_init;
        aux_filter[n].start = FX_fin;
        aux_filter[n].end   = FX_init;
        n++;
    }        
   
    if((pr->y + LIMY) < DISTANCIA_SEGURIDAD) 
    {
        FY_fin  = RAD2DEG(asin((-LIMY - pr->y)/DISTANCIA_SEGURIDAD));
        FY_init = -180 - FY_fin;
        aux_filter[n].start = FY_fin;
        aux_filter[n].end = FY_init;        
        n++;
    }


    // Si hay entrelazamiento entre los angulos
    // fusionamos el aux_filter
    if(!n)
    {
        arc.rpose.arco.start = -180;
        arc.rpose.arco.end   = 180;
        v_safe.push_back(arc);
        return v_safe;
    }
    else if((n==2) && (FY_init<=FX_fin))
    {
        aux_filter[0].start = FY_fin;
        aux_filter[0].end   = FX_init;
        n = 1;
    }

    // Lo transformamso a posiciones relativas respecto
    // el cero del lidar:
    for(i = 0; i <n; i++)
    {
        aux_filter[i].start = M180(aux_filter[i].start - pr->theta);
        aux_filter[i].end   =  M180(aux_filter[i].end  - pr->theta);
        
        // Si un aux_filter pasa desde 180 a -180 lo partimos
        // en dos partes:
        if(aux_filter[i].start > aux_filter[i].end)                   
        {
            if(aux_filter[i].end != -180)
            {
                arc.rpose.arco.start = -180;
                arc.rpose.arco.end  = aux_filter[i].end;
                v_safe.push_back(arc);
            }
            
            arc.rpose.arco.start = aux_filter[i].start;
            arc.rpose.arco.end  = 180;
            v_safe.push_back(arc);
        }
        else
        {
            arc.rpose.arco = aux_filter[i];
            v_safe.push_back(arc);

        }        
    }
    std::sort(v_safe.begin(), v_safe.end(), Compare_FiltroAngular);
    return v_safe;
}

VFiltros DangerAngles4C(const struct pose *pr)
{
    /*
        Esta función calcula una serie de ángulos en 
        los que no es posibe que haya riesgo de 
        colisión con el robot enemigo.
    */
    float FX_init = 0;
    float FX_fin  = 0;
    float FY_init = 0;
    float FY_fin  = 0;
    int   n = 0;
    int   i;
    Seccion aux_filter[4] = {0};
    VFiltros v_safe;
    FiltroAngular arc;

    // Inicio la estructura
    arc.rpose.distance = 0;
    arc.motivo = ROBOT;
    arc.tipo = 0;

    // Calculo los angulos a que  hay que filtrar:
    if(DISTANCIA_SEGURIDAD > (LIMX - pr->x)) 
    {
        FX_fin  = RAD2DEG(acos((LIMX - pr->x)/DISTANCIA_SEGURIDAD));
        FX_init = -FX_fin;
        aux_filter[n].start = FX_fin;
        aux_filter[n].end   = FX_init;
        n++;
    }
    
    if((pr->y + LIMY) < DISTANCIA_SEGURIDAD) 
    {
        FY_fin  = RAD2DEG(asin((-LIMY - pr->y)/DISTANCIA_SEGURIDAD));
        FY_init = -180 - FY_fin;
        aux_filter[n].start = FY_fin;
        aux_filter[n].end = FY_init;        
        n++;
    }


    // Si hay entrelazamiento entre los angulos
    // fusionamos el aux_filter
    if(!n)
    {
        arc.rpose.arco.start = -180;
        arc.rpose.arco.end   = 180;
        v_safe.push_back(arc);
        return v_safe;
    }
    else if((n==2) && (FY_fin>=FX_init))
    {
        aux_filter[0].start = FX_fin;
        aux_filter[0].end   = FY_init;
        n = 1;
    }

    // Lo transformamso a posiciones relativas respecto
    // el cero del lidar:
    for(i = 0; i <n; i++)
    {
        aux_filter[i].start = M180(aux_filter[i].start - pr->theta);
        aux_filter[i].end   = M180(aux_filter[i].end  - pr->theta);
        
        // Si un aux_filter pasa desde 180 a -180 lo partimos
        // en dos partes:
        if(aux_filter[i].start > aux_filter[i].end)                   
        {
            if(aux_filter[i].end != -180)
            {
                arc.rpose.arco.start = -180;
                arc.rpose.arco.end  = aux_filter[i].end;
                v_safe.push_back(arc);
            }
            
            arc.rpose.arco.start = aux_filter[i].start;
            arc.rpose.arco.end  = 180;
            v_safe.push_back(arc);
        }
        else
        {
            arc.rpose.arco = aux_filter[i];
            v_safe.push_back(arc);

        }        
    }
    std::sort(v_safe.begin(), v_safe.end(), Compare_FiltroAngular);
    return v_safe;
}


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


void DesacoploAngulos(VFiltros &VObjRdistance)
{   
    int i;
    int j;
    float aux_angle;
    FiltroAngular aux;
    
    i = 0;
    j = VObjRdistance.size();
    
    // Analizamos si hay angulos que se pisan:
    while(i<VObjRdistance.size()-1)
    {
        // Existe acoplamiento entre dos angulos de interes?
        if(VObjRdistance[i+1].rpose.arco.start<VObjRdistance[i].rpose.arco.end)
        {
            if(VObjRdistance[i].rpose.arco.end<VObjRdistance[i+1].rpose.arco.end)
            {
                // Es necesario añadir condición de inicio de igual igual?
                // Arco de intersección:
                aux.rpose.arco.start  = VObjRdistance[i+1].rpose.arco.start;
                aux.rpose.arco.end    = VObjRdistance[i].rpose.arco.end;
                
                aux.rpose.distance    = VObjRdistance[i].rpose.distance + VObjRdistance[i+1].rpose.distance;
                aux.motivo            = AMBOS;
                aux.tipo              = VObjRdistance[i].tipo + VObjRdistance[i+1].tipo;
                aux.salto             = VObjRdistance[i].salto + VObjRdistance[i+1].salto;

                aux_angle = VObjRdistance[i].rpose.arco.end;

                // Modificamos el primer ángulo:
                VObjRdistance[i].rpose.arco.end = VObjRdistance[i+1].rpose.arco.start;

                // Modificamos el segundo arco
                VObjRdistance[i+1].rpose.arco.start = aux_angle;

                // Insertamos el arco de conflicto
                VObjRdistance.insert(VObjRdistance.begin()+i+1,aux);
                
                // Esta operación se puede quitar
                // Si son iguales, este arco sobra:
                if(VObjRdistance[i].rpose.arco.start == VObjRdistance[i].rpose.arco.end)
                    VObjRdistance.erase(VObjRdistance.begin()+i);
            }
            else if(VObjRdistance[i].rpose.arco.end>VObjRdistance[i+1].rpose.arco.end)
            {
                
                aux = VObjRdistance[i];
                aux.rpose.arco.start = VObjRdistance[i+1].rpose.arco.end;

                VObjRdistance[i].rpose.arco.end = VObjRdistance[i+1].rpose.arco.start;
                VObjRdistance[i+1].tipo = VObjRdistance[i+1].tipo + VObjRdistance[i].tipo;
                VObjRdistance[i+1].motivo = AMBOS;
                VObjRdistance[i+1].rpose.distance = VObjRdistance[i+1].rpose.distance + VObjRdistance[i].rpose.distance;
                VObjRdistance[i+1].salto = VObjRdistance[i+1].salto + VObjRdistance[i].salto;



                // Insertamos el arco de conflicto
                VObjRdistance.insert(VObjRdistance.begin()+i+2,aux);
                // Un arco de más
                if(VObjRdistance[i].rpose.arco.start == VObjRdistance[i].rpose.arco.end)
                    VObjRdistance.erase(VObjRdistance.begin()+i);
            }

            else
            {

                // Modificamos el ángulo del arco
                VObjRdistance[i].rpose.arco.end  = VObjRdistance[i+1].rpose.arco.start;
                
                // Arco de intersección:
                VObjRdistance[i+1].rpose.distance    = VObjRdistance[i].rpose.distance + VObjRdistance[i+1].rpose.distance;
                VObjRdistance[i+1].motivo            = AMBOS;
                VObjRdistance[i+1].tipo              = VObjRdistance[i].tipo + VObjRdistance[i+1].tipo;
                VObjRdistance[i+1].salto             = VObjRdistance[i].salto + VObjRdistance[i+1].salto;
                
                // Un arco de más
                if(VObjRdistance[i].rpose.arco.start == VObjRdistance[i].rpose.arco.end)
                    VObjRdistance.erase(VObjRdistance.begin()+i);
            }
        }
        //std::sort(VObjRdistance.begin(), 
        //VObjRdistance.end(), 
        //Compare_FiltroAngular);

        i++;        
    }
}


VFiltros update_filters(const pose *robot){

    using namespace std;


    // Creo  variables auxiliares.
    FiltroAngular aux;
    CoronaCircular  dist;
    VFiltros VObjRdistance;
    
    //VFiltros::iterator it;
    // Calculo los angulos que estan fuera del campo
    if((robot->x>=0)&&(robot->y>=0))
        VObjRdistance = DangerAngles1C(robot);
    else if((robot->x<=0)&&(robot->y>=0))
        VObjRdistance = DangerAngles2C(robot);
    else if((robot->x<=0)&&(robot->y<=0))
        VObjRdistance = DangerAngles3C(robot);
    else if((robot->x>=0)&&(robot->y<=0))
        VObjRdistance = DangerAngles4C(robot);
    
    // Calculo donde espero ver los objetos
    for(int i = 0;i<lfobjects.size(); i++)
    {     
        // Calculo la distancia
        dist = ObjectsAngles(robot,&lfobjects[i]);

        // Si pasamos por -180 a 180
        if((dist.arco.start > 0)&&(dist.arco.end<dist.arco.start))
        {
            aux.rpose.arco.start = -180;
            aux.rpose.arco.end   = dist.arco.end;
            aux.rpose.distance   = dist.distance;
            aux.motivo           = OBJETO;
            aux.tipo             = i+1;
            aux.salto            = true;

            VObjRdistance.push_back(aux);
            
            aux.rpose.arco.start = dist.arco.start;
            aux.rpose.arco.end   = 180;
            aux.rpose.distance   = dist.distance;
            aux.motivo           = OBJETO;
            aux.tipo             = i+1;
            aux.salto            = false;

            VObjRdistance.push_back(aux);

        }
        else
        {
            aux.rpose   = dist;
            aux.motivo  = OBJETO;
            aux.tipo    = i+1;
            VObjRdistance.push_back(aux);
        }   
    }
        // Ordenamos los angulos para que analizarlo 
        // de manera ordenada desde el lidar:

        std::sort(VObjRdistance.begin(), 
            VObjRdistance.end(), 
            Compare_FiltroAngular);
        
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
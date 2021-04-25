#include "uahr_common.hpp"


#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)
#define N_2PI 2*M_PI

inline bool InLimits(float lowerlimit, float upperlimit, float target)
{
    if((lowerlimit<target) && (target<upperlimit)) return true;
    else false;
}

inline bool InSection(Seccion limit, float target)
{
    if((limit.start<target) && (target<limit.end)) return true;
    else false;
}

inline float M180(float angle)
{
    /*
        M180 :: float -> float

        Convierte un angulo a modulo 
        entre -180 y 180.   
    */ 
    while(angle>180)
        angle = angle -360;
    while (angle<-180)
       angle = angle+360;
    return angle;
}

bool Compare_FiltroAngular(FiltroAngular a1, FiltroAngular a2)
{
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
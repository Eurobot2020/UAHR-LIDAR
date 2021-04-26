#pragma once
#include "uahr_types.hpp"

bool Compare_FiltroAngular(FiltroAngular a1, FiltroAngular a2);

inline bool InLimits(float target,float lowerlimit, float upperlimit)
{
    if((lowerlimit<target) && (target<upperlimit)) return true;
    else return false;
}

inline bool InSection(float target,Seccion limit)
{
    if((limit.start<target) && (target<limit.end)) return true;
    else return false;
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

#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)
#define N_2PI 2*M_PI



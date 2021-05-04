// 
#pragma once
#include <vector>
#include "uahr_msgs/Polar.h"
#include "uahr_msgs/PolarArray.h"
#include "uahr_msgs/array_arcos.h"
#include "uahr_msgs/Arco_Interes.h"

#include "geometry_msgs/Pose2D.h"


// Defines:
#define OBJETO  'O'
#define ROBOT   'R'
#define AMBOS   'A'
#define FA       10
#define FD       0.1  
// Estructuras
struct polar 
{
    float angle;
    float dist;
    polar(float _angle,float _dist) : 
    angle{_angle},dist{_dist}{}

};





struct pose
{
    float x;
    float y;
    float theta;
    pose(float _x,float _y, float _a) : 
    x{_x}, y{_y}, theta{_a}{}
    pose() : x{0}, y{0}, theta{0}{}
};

struct Seccion
{
    /* data */
    float start;
    float end;
    
    // Constructor:
    Seccion():start{0},end{0}{}
    Seccion(float s):start{s},end{s}{}  
    Seccion(float s,float e) : start{s}, end{e}{}

    // Operators:
    friend bool operator== (const Seccion one,const Seccion two);
    friend Seccion operator+(const Seccion one,const Seccion two);

};

struct ObjSearchData 
{
    /* data */
    const int x;
    const int y;
    const int id;
    polar rpose;

    ObjSearchData(int _x,int _y, int _id): 
    x{_x}, y{_y},id{_id},rpose(0,0){}
};

using VPolars  = std::vector<polar>;
using VVPolars = std::vector<VPolars>;
using VFiltros = std::vector<Seccion>;
using VSearchObjects = std::vector<ObjSearchData >;

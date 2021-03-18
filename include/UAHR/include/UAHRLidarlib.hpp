

#pragma once


// Defines utiles
#define OBJETO  'O'
#define ROBOT   'R'
#define AMBOS   'A'
#define FA       3
#define CONSTFA  5
#define FD       0.1        
#define N_OBJETOS  6    
#define MAX_DISANCE_ENEMY 3100

#define BALIZA_ALIADA_I  1
#define BALIZA_ALIADA_S  2
#define TORRE            3
#define BALIZA_ENEMIGA   4


// Estructuras
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

struct CoronaCircular
{
    struct Seccion  arco;
    struct Seccion  distance;
    
    // Constructores:
    CoronaCircular():arco{},distance{}{}
    
    // A veces es útil tratarlo como un arco:
    CoronaCircular(Seccion _arco):
    arco{_arco.start,_arco.end}, distance{}{}
    
    CoronaCircular(Seccion _arco,Seccion _distance):
    arco{_arco.start,_arco.end}, distance{_distance}{}
    
    // Operador util para Test:
    friend bool operator== (const CoronaCircular one,const CoronaCircular two);
};

struct ObjSearchData 
{
    /* data */
    const int x_inf;
    const int x_sup;
    const int y_inf;
    const int y_sup;
    int aexp;
    const float dexp;

    ObjSearchData(int x_i,int x_s, int y_i, int y_s) : 
        x_inf{x_i}, x_sup{x_s}, y_inf{y_i}, 
        y_sup{y_s}, aexp{FA},dexp{FD}{}
};

struct FiltroAngular
{
    struct CoronaCircular rpose;     
    char   motivo;
    int    tipo;
    bool   salto;

    // Constructores:
    FiltroAngular() : rpose(),motivo{ROBOT},salto{false},tipo{0}{}
    
    FiltroAngular(Seccion _arco, char _motivo, int _objeto) : 
    rpose(_arco),motivo{ROBOT},tipo{_objeto},salto(false){}
    
    FiltroAngular(Seccion _arco, float _dist, char _motivo, int _objeto) : 
    rpose(_arco,_dist),motivo{ROBOT},tipo{_objeto},salto{false}{}
    
    // Operadores
    friend bool operator== (const FiltroAngular one,const FiltroAngular two);
};


/*
class BeamHandler
{
    private:
        const std::vector<ObjSearchData> TriangulateObjects;

    public:
        BeamHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p);
        ~BeamHandler();
        VFiltros ArcOfInterest;
        
};
BeamHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p)
: TriangulateObjects{ObjetosBusqueda}, probot{p}
{
    this->
    this->ArcOfInterest  

}


UAHR::~UAHR()
{}
*/

using VFiltros = std::vector<FiltroAngular>;

float M180(float angle);
bool Compare_FiltroAngular(FiltroAngular a1, FiltroAngular a2);
struct CoronaCircular near_objects_pose(const struct pose *r_pose,const struct ObjSearchData *p_obj);
struct CoronaCircular ObjectsAngles(const struct pose *r_pose,const struct ObjSearchData *p_obj);
VFiltros DangerAngles1C(const struct pose *pr);
VFiltros DangerAngles2C(const struct pose *pr);
VFiltros DangerAngles3C(const struct pose *pr);
VFiltros DangerAngles4C(const struct pose *pr);
void DesacoploAngulos(VFiltros &VObjRdistance);
VFiltros  update_filters (const pose *robot);

extern std::vector<ObjSearchData> lfobjects;
extern int fil_angular [4];

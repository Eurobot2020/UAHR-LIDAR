

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
    const int id;


    ObjSearchData(int x_i,int x_s, int y_i, int y_s,int _id): 
        x_inf{x_i}, x_sup{x_s}, y_inf{y_i}, 
        y_sup{y_s}, aexp{FA},dexp{FD},id{_id}{}
};
struct FiltroAngular
{
    struct CoronaCircular rpose;     
    char   motivo;
    int    tipo;
    bool   salto;

    // Constructores:
    FiltroAngular(void) : rpose(),motivo{ROBOT},tipo{0},salto{false}{}
    FiltroAngular(Seccion _arco): 
    rpose(_arco),motivo{ROBOT},tipo{0},salto{false}{}
    FiltroAngular(char _motivo, int _objeto): 
    rpose{},motivo{_motivo},tipo{_objeto},salto(false){}

    
    FiltroAngular(Seccion _arco, float _dist, char _motivo, int _objeto) : 
    rpose(_arco,_dist),motivo{ROBOT},tipo{_objeto},salto{false}{}
    FiltroAngular(Seccion _arco,Seccion _dist, char _motivo, int _objeto,bool _salto) : 
    rpose(_arco,_dist),motivo{ROBOT},tipo{_objeto},salto{_salto}{}

    // Operadores
    friend bool operator== (const FiltroAngular one,const FiltroAngular two);
};
using VFiltros = std::vector<FiltroAngular>;



using VObjetos = std::vector<ObjSearchData>;
using VFiltros = std::vector<FiltroAngular>;

/*
class LidarHandler
{
    private:
        const std::vector<ObjSearchData> TriangulateObjects;
        VFiltros SearchSections;
        pose robot;

    public:
        LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p);
        ~LidarHandler();
        
};

LidarHandler::LidarHandler(std::vector<ObjSearchData> ObjetosBusqueda,pose p)
: TriangulateObjects{ObjetosBusqueda}, robot{p}
{
    this->
    this->ArcOfInterest  

}


UAHR::~UAHR()
{}
*/


float M180(float angle);
bool Compare_FiltroAngular(FiltroAngular a1, FiltroAngular a2);
struct CoronaCircular near_objects_pose(const struct pose *r_pose,const struct ObjSearchData *p_obj);
void ObjectsAngles(const pose &p_obs,const ObjSearchData &pasive_obj,VFiltros &vf);
void DangerAngles1C(const pose &pr, VFiltros &vdf);
void DangerAngles2C(const pose &pr, VFiltros &vdf);
void DangerAngles3C(const pose &pr, VFiltros &vdf);
void DangerAngles4C(const pose &pr, VFiltros &vdf);

void DesacoploAngulos(VFiltros &VObjRdistance);
VFiltros new_filters(pose const &robot,VObjetos &lfobjects);


extern std::vector<ObjSearchData> lfobjects;
extern int fil_angular [4];

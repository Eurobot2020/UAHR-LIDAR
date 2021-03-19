#include <gtest/gtest.h>
#include "UAHRLidarlib.hpp"


// Bring in gtest

std::vector<ObjSearchData> lfobjects;

TEST(UAHRLidar, M180)
{
  EXPECT_EQ(M180(10),10.0);
  EXPECT_EQ(M180(190),-170.0);
  EXPECT_EQ(M180(390),30.0);
  EXPECT_EQ(M180(-390),-30.0);
  EXPECT_EQ(M180(200),-160);
  EXPECT_EQ(M180(-200),160);
  EXPECT_EQ(M180(-270),90);
}

TEST(UAHRLidar, DangerAngles1)
{
  pose robot;
  FiltroAngular arc_aux;
  VFiltros v_safe;
  VFiltros v_aux;
  float a;
  int i=0;

  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  
  arc_aux.rpose.arco.start = -180;
  arc_aux.rpose.arco.end   = 180;
  
  v_safe.push_back(arc_aux);

  EXPECT_EQ(DangerAngles1C(&robot)[0],
          FiltroAngular(Seccion(-180,180),
          ROBOT,0));
  
  robot.x = 100;
  robot.y = 100;
  robot.theta = 30;
  EXPECT_EQ(DangerAngles1C(&robot)[0],
          FiltroAngular(Seccion(-180,180),
          ROBOT,0));
  
  robot.x = 100;
  robot.y = 1000;
  robot.theta = 30;
  
  EXPECT_EQ(DangerAngles1C(&robot)[0].rpose.arco.start,
          -180);
  EXPECT_EQ(DangerAngles1C(&robot)[0].rpose.arco.end,
          -30);
  EXPECT_EQ(DangerAngles1C(&robot)[0],
          FiltroAngular(Seccion(-180,-30),
          ROBOT,0));
  EXPECT_EQ(DangerAngles1C(&robot)[1],
          FiltroAngular(Seccion(150,180),
          ROBOT,0));

  robot.x = 100;
  robot.y = 700;
  robot.theta = 0;
  arc_aux.rpose.arco.start = -180;
  arc_aux.rpose.arco.end   = 36;
  v_safe.push_back(arc_aux);
  arc_aux.rpose.arco.start = 143;
  arc_aux.rpose.arco.end   = 180;
  v_safe.push_back(arc_aux);

  EXPECT_EQ(DangerAngles1C(&robot).size(),2);
  EXPECT_EQ(DangerAngles1C(&robot).size(),2);
  EXPECT_EQ(DangerAngles1C(&robot)[0],
          FiltroAngular(Seccion(-180,36),
          ROBOT,0));
  EXPECT_EQ(DangerAngles1C(&robot)[1],
         FiltroAngular(Seccion(143,180),
          ROBOT,0));

  robot.x = 1500;
  robot.y = 1000;
  robot.theta = 0;

  EXPECT_EQ(DangerAngles1C(&robot)[0],
          FiltroAngular(Seccion(-180,-90),
          ROBOT,0));
  robot.x = 1500;
  robot.y = 1000;
  robot.theta = 180;

  EXPECT_EQ(DangerAngles1C(&robot)[0],
          FiltroAngular(Seccion(0,90),
          ROBOT,0));

  robot.x = 1200;
  robot.y = 800;
  robot.theta = 0;

  EXPECT_EQ(DangerAngles1C(&robot)[0],
          FiltroAngular(Seccion(-180,-53),
          ROBOT,0));
  EXPECT_EQ(DangerAngles1C(&robot)[1],
          FiltroAngular(Seccion(156,180),
          ROBOT,0));
}


TEST(UAHRLidar, UAHRLidar_DangerAngles2_Test)
{
  pose robot;
  FiltroAngular arc_aux;
  VFiltros v_safe;
  VFiltros v_aux;
  float a;
  int i=0;

  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  
  arc_aux.rpose.arco.start = -180;
  arc_aux.rpose.arco.end   = 180;
  
  v_safe.push_back(arc_aux);

  EXPECT_EQ(DangerAngles2C(&robot)[0],
          FiltroAngular(Seccion(-180,180),
          ROBOT,0));
  
  robot.x = -100;
  robot.y = -100;
  robot.theta = 30;
  EXPECT_EQ(DangerAngles2C(&robot)[0],
          FiltroAngular(Seccion(-180,180),
          ROBOT,0));
  
  robot.x = -100;
  robot.y = 1000;
  robot.theta = 30;
  EXPECT_EQ(DangerAngles2C(&robot).size(),2);
  EXPECT_EQ(DangerAngles2C(&robot)[0].rpose.arco.start,
        -180);
  EXPECT_EQ(DangerAngles2C(&robot)[0].rpose.arco.end,
        -30);
  EXPECT_EQ(DangerAngles2C(&robot)[0],
          FiltroAngular(Seccion(-180,-30),
          ROBOT,0));
  EXPECT_EQ(DangerAngles2C(&robot)[1],
          FiltroAngular(Seccion(150,180),
          ROBOT,0));

  robot.x = -100;
  robot.y = 700;
  robot.theta = 0;

 
  EXPECT_EQ(DangerAngles2C(&robot).size(),2);
  
  EXPECT_EQ(DangerAngles2C(&robot)[0],
          FiltroAngular(Seccion(-180,36),
          ROBOT,0));
  EXPECT_EQ(DangerAngles2C(&robot)[1],
         FiltroAngular(Seccion(143,180),
          ROBOT,0));

    robot.x = -1500;
    robot.y = 1000;
    robot.theta = 0;

    EXPECT_EQ(DangerAngles2C(&robot)[0],
          FiltroAngular(Seccion(-90,0),
          ROBOT,0));
    
    robot.x = -1500;
    robot.y = 1000;
    robot.theta = 180;
    
    EXPECT_EQ(DangerAngles2C(&robot)[0],
            FiltroAngular(Seccion(90,180),
            ROBOT,0));

    robot.x = -1200;
    robot.y = 800;
    robot.theta = 0;

    EXPECT_EQ(DangerAngles2C(&robot)[0],
        FiltroAngular(Seccion(-126,23),
        ROBOT,0));

}

TEST(UAHRLidar, UAHRLidar_DangerAngles3_Test)
{
    pose robot;
    FiltroAngular arc_aux;
    VFiltros v_safe;
    VFiltros v_aux;
    float a;
    int i=0;

    robot.x = 0;
    robot.y = 0;
    robot.theta = 0;
    
    v_safe.push_back(arc_aux);

    EXPECT_EQ(DangerAngles3C(&robot)[0],
            FiltroAngular(Seccion(-180,180),
            ROBOT,0));
    
    robot.x = -100;
    robot.y = -100;
    robot.theta = 30;
    EXPECT_EQ(DangerAngles3C(&robot)[0],
          FiltroAngular(Seccion(-180,180),
          ROBOT,0));
    
    robot.x = -100;
    robot.y = -1000;
    robot.theta = 30;
    EXPECT_EQ(DangerAngles3C(&robot).size(),1);
    EXPECT_EQ(DangerAngles3C(&robot)[0],
            FiltroAngular(Seccion(-30,150),
            ROBOT,0));

    robot.x = -100;
    robot.y = -700;
    robot.theta = 0;

    
    EXPECT_EQ(DangerAngles3C(&robot).size(),2);
    
    EXPECT_EQ(DangerAngles3C(&robot)[0],
            FiltroAngular(Seccion(-180,-143),
            ROBOT,0));
    EXPECT_EQ(DangerAngles3C(&robot)[1],
            FiltroAngular(Seccion(-36,180),
            ROBOT,0));

    robot.x = -1500;
    robot.y = -1000;
    robot.theta = 0;

    EXPECT_EQ(DangerAngles3C(&robot)[0],
          FiltroAngular(Seccion(0,90),
          ROBOT,0));
    
    robot.x = -1500;
    robot.y = -1000;
    robot.theta = 180;
    
    EXPECT_EQ(DangerAngles3C(&robot)[0],
            FiltroAngular(Seccion(-180,-90),
            ROBOT,0));

    robot.x = -1200;
    robot.y = -800;
    robot.theta = 0;
    EXPECT_EQ(DangerAngles3C(&robot)[0],
        FiltroAngular(Seccion(-23,126),
        ROBOT,0));
}


TEST(UAHRLidar, UAHRLidar_DangerAngles4_Test)
{
    pose robot;
    FiltroAngular arc_aux;
    VFiltros v_safe;
    VFiltros v_aux;
    float a;
    int i=0;

    robot.x = 0;
    robot.y = 0;
    robot.theta = 0;
    
    arc_aux.rpose.arco.start = -180;
    arc_aux.rpose.arco.end   = 180;

    EXPECT_EQ(DangerAngles4C(&robot)[0],
            FiltroAngular(Seccion(-180,180),
            ROBOT,0));
    
    robot.x = 100;
    robot.y = -100;
    robot.theta = 30;
    EXPECT_EQ(DangerAngles4C(&robot)[0],
            FiltroAngular(Seccion(-180,180),
            ROBOT,0));
    
    robot.x = 100;
    robot.y = -1000;
    robot.theta = 30;
    EXPECT_EQ(DangerAngles4C(&robot).size(),1);
    EXPECT_EQ(DangerAngles4C(&robot)[0],
            FiltroAngular(Seccion(-30,150),
            ROBOT,0));
    robot.x = 100;
    robot.y = -700;
    robot.theta = 0;

    
    EXPECT_EQ(DangerAngles4C(&robot).size(),2);
    
    EXPECT_EQ(DangerAngles4C(&robot)[0],
        FiltroAngular(Seccion(-180,-143),
        ROBOT,0));
    EXPECT_EQ(DangerAngles4C(&robot)[1],
        FiltroAngular(Seccion(-36,180),
        ROBOT,0));

    robot.x = 1500;
    robot.y = -1000;
    robot.theta = 0;

    EXPECT_EQ(DangerAngles4C(&robot)[0],
        FiltroAngular(Seccion(90,180),
        ROBOT,0));
    
    robot.x = 1500;
    robot.y = -1000;
    robot.theta = 180;
    
    EXPECT_EQ(DangerAngles4C(&robot)[0],
            FiltroAngular(Seccion(-90,0),
            ROBOT,0));

    robot.x = 1200;
    robot.y = -800;
    robot.theta = 0;

    EXPECT_EQ(DangerAngles4C(&robot)[0],
        FiltroAngular(Seccion(-180,-156),
        ROBOT,0));
    EXPECT_EQ(DangerAngles4C(&robot)[1],
        FiltroAngular(Seccion(53,180),
        ROBOT,0));
}

TEST(UAHRLidar, UAHRObjectsAngles)
{
    pose robot;
    CoronaCircular dist;

    robot.x = 1200;
    robot.y = -800;
    robot.theta = 0;
    ObjSearchData baliza_sup{1550, 1550, 850,1000};
    ObjSearchData baliza_inf{1540, 1540, -900,-1000};
    ObjSearchData baliza_medio{-1540, -1540, 25,-25};
    ObjSearchData torre{-150 , 150, 1000,  1000};
    dist= ObjectsAngles(&robot,&baliza_sup);

    //EXPECT_EQ((int)dist.distance,(int)1760.21);
    EXPECT_EQ((int)dist.arco.start,(int)62);
    EXPECT_EQ((int)dist.arco.end,(int)94);
    
    robot.x = -400;
    robot.y = 0;
    robot.theta = 0;
    dist= ObjectsAngles(&robot,&baliza_medio);
    //EXPECT_EQ((int)dist.distance,(int)1140);
    EXPECT_EQ((int)dist.arco.start,(int)172);
    EXPECT_EQ((int)dist.arco.end,(int)-172);
}

/*
TEST(UAHRLidar, DesacoploAngulos)
{
    pose robot;
    CoronaCircular dist;
    VFiltros arcos;

    FiltroAngular arc_aux;

    arc_aux.motivo = ROBOT;
    arc_aux.rpose.arco.start = -180;
    arc_aux.rpose.arco.end   =  180;
    arc_aux.salto   =  false;
    arcos.push_back(arc_aux);
    arc_aux.motivo = OBJETO;
    arc_aux.rpose.arco.start = -180;
    arc_aux.rpose.arco.end   =  -30;
    arc_aux.salto   =  true;
    arcos.push_back(arc_aux);
    arcos = DesacoploAngulos(arcos);
    EXPECT_EQ(arcos.size(),2);
    EXPECT_EQ(arcos[0].rpose.arco,arco(-180,-30));
    EXPECT_EQ(arcos[1].rpose.arco.start,-30);
    EXPECT_EQ(arcos[1].rpose.arco.end,180);
    EXPECT_EQ(arcos[1].rpose.arco,arco(-30,180));

    arcos.clear();
    arc_aux.motivo = ROBOT;
    arc_aux.rpose.arco.start = -180;
    arc_aux.rpose.arco.end   =  180;
    arc_aux.salto   =  false;
    arcos.push_back(arc_aux);
    arc_aux.motivo = OBJETO;
    arc_aux.rpose.arco.start = -170;
    arc_aux.rpose.arco.end   =  -30;
    arc_aux.salto   =  true;
    arcos.push_back(arc_aux);
    arcos = DesacoploAngulos(arcos);
    EXPECT_EQ(arcos.size(),3);
    EXPECT_EQ(arcos[0].rpose.arco,arco(-180,-170));
    EXPECT_EQ(arcos[1].rpose.arco,arco(-170,-30));
    EXPECT_EQ(arcos[2].rpose.arco,arco(-30,180));

    arcos.clear();
    arc_aux.motivo = ROBOT;
    arc_aux.rpose.arco.start = -180;
    arc_aux.rpose.arco.end   =  30;
    arc_aux.salto   =  false;
    arcos.push_back(arc_aux);
    arc_aux.motivo = OBJETO;
    arc_aux.rpose.arco.start = -180;
    arc_aux.rpose.arco.end   =  -40;
    arc_aux.salto   =  true;
    arcos.push_back(arc_aux);
    arc_aux.rpose.arco.start = -170;
    arc_aux.rpose.arco.end   =  -35;
    arcos.push_back(arc_aux);
    std::sort(arcos.begin(), 
    arcos.end(), 
    Compare_FiltroAngular);

    arcos = DesacoploAngulos(arcos);
    EXPECT_EQ(arcos.size(),4);
    EXPECT_EQ(arcos[0].rpose.arco,arco(-180,-170));
    EXPECT_EQ(arcos[0].rpose.arco.start,-180);
    EXPECT_EQ(arcos[0].rpose.arco.end,-40);

    
    EXPECT_EQ(arcos[1].rpose.arco.start,-170);

    /*
    arcos.clear();
    arc_aux.motivo = ROBOT;
    arc_aux.rpose.arco.start = -180;
    arc_aux.rpose.arco.end   =  30;
    arc_aux.salto   =  false;
    arcos.push_back(arc_aux);
    arc_aux.motivo = OBJETO;
    arc_aux.rpose.arco.start = -180;
    arc_aux.rpose.arco.end   =  -40;
    arc_aux.salto   =  true;
    arcos.push_back(arc_aux);
    arc_aux.rpose.arco.start =  -50;
    arc_aux.rpose.arco.end   =  -30;
    arcos.push_back(arc_aux);
    arc_aux.rpose.arco.start =  -27;
    arc_aux.rpose.arco.end   =  52;
    arcos.push_back(arc_aux);
    arc_aux.rpose.arco.start = -170;
    arc_aux.rpose.arco.end   =  -35;
    arcos.push_back(arc_aux);
    arc_aux.rpose.arco.start =  130;
    arc_aux.rpose.arco.end   =  150;
    arcos.push_back(arc_aux);
    arc_aux.rpose.arco.start =  140;
    arc_aux.rpose.arco.end   =  160;
    arcos.push_back(arc_aux);
    arc_aux.rpose.arco.start =  165;
    arc_aux.rpose.arco.end   =  180;
    arcos.push_back(arc_aux);
    std::sort(arcos.begin(), 
    arcos.end(), 
    Compare_FiltroAngular);

    arcos = DesacoploAngulos(arcos);
    EXPECT_EQ(arcos.size(),12);
    EXPECT_EQ(arcos[0].rpose.arco,arco(-180,-40));
    */
    //EXPECT_EQ(arcos[1].rpose.arco,arco(-170,-30));
    //EXPECT_EQ(arcos[2].rpose.arco,arco(-30,180));


//}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
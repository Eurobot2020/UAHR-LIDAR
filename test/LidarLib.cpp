#include <gtest/gtest.h>
#include "uahr_class.hpp"


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

TEST(UAHRLidar, DangerAngles1_Test)
{
  pose robot;
  FiltroAngular arc_aux;
  VFiltros vr;
  
  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
    
  DangerAngles1C(robot,vr);
  EXPECT_EQ(vr[0],Seccion(-180,180));
  
  robot.x = 100;
  robot.y = 100;
  robot.theta = 30;
  vr.clear();
  DangerAngles1C(robot,vr);
  EXPECT_EQ(vr[0],
    FiltroAngular(Seccion(-180,180)));


  robot.x = 100;
  robot.y = 999;
  vr.clear();

  DangerAngles1C(robot,vr);
  EXPECT_EQ(vr[0],
    FiltroAngular(Seccion(179,0)));

  robot.x = 100;
  robot.y = 700;
  vr.clear();
  DangerAngles1C(robot,vr);
  EXPECT_EQ(vr[0],
    FiltroAngular(Seccion(143,36)));


  robot.x = 1500;
  robot.y = 999;
  robot.theta = 0;
  vr.clear();
  DangerAngles1C(robot,vr);
  EXPECT_EQ(vr[0],
    FiltroAngular(Seccion(179,-90)));


  robot.x = 1200;
  robot.y = 800;
  robot.theta = 0;
  vr.clear();
  DangerAngles1C(robot,vr);
  EXPECT_EQ(vr[0],
  FiltroAngular(Seccion(156,-53)));
}


TEST(UAHRLidar, DangerAngles2_Test)
{
  pose robot;
  VFiltros vr;

  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  DangerAngles2C(robot,vr);
  EXPECT_EQ(vr[0],
  FiltroAngular(Seccion(-180,180)));
  
  robot.x = -100;
  robot.y = -100;
  robot.theta = 30;
  vr.clear();
  DangerAngles2C(robot,vr);
  EXPECT_EQ(vr[0],
  FiltroAngular(Seccion(-180,180)));
    
  robot.x = -100;
  robot.y = 999;
  vr.clear();
  DangerAngles2C(robot,vr);
  EXPECT_EQ(vr[0],
  FiltroAngular(Seccion(179,0)));
  

  robot.x = -100;
  robot.y = 700;
  robot.theta = 0;
  vr.clear();
  DangerAngles2C(robot,vr);  
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(143,36)));
 

  robot.x = -1500;
  robot.y = 900;
  robot.theta = 0;
  vr.clear();
  DangerAngles2C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-90,11.7)));

  robot.x = -1500;
  robot.y = 0;
  robot.theta = 0;
  vr.clear();
  DangerAngles2C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-90,90)));

  
  robot.x = -1200;
  robot.y = 800;
  robot.theta = 0;
  vr.clear();
  DangerAngles2C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-126,23)));
}

TEST(UAHRLidar, UAHRLidar_DangerAngles3_Test)
{
  pose robot;
  VFiltros vr;

  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  DangerAngles3C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-180,180)));
  
  robot.x = -100;
  robot.y = -100;
  vr.clear();
  DangerAngles3C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-180,180)));
  
  robot.x = -100;
  robot.y = -999;
  vr.clear();
  DangerAngles3C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(0,-179)));
  
  robot.x = -100;
  robot.y = -700;
  robot.theta = 0;
  vr.clear();
  DangerAngles3C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-36,-143)));
 
  robot.x = -1500;
  robot.y = -999;
  robot.theta = 0;
  vr.clear();
  DangerAngles3C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(0,90)));

  robot.x = -1500;
  robot.y = 0;
  robot.theta = 0;
  vr.clear();
  DangerAngles3C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-90,90)));

  
  
  robot.x = -1200;
  robot.y = -800;
  robot.theta = 0;
  vr.clear();
  DangerAngles3C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-23,126)));
}


TEST(UAHRLidar, UAHRLidar_DangerAngles4_Test)
{
  pose robot;
  VFiltros vr;

  robot.x = 0;
  robot.y = 0;
  robot.theta = 0;
  DangerAngles4C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-180,180)));

  robot.x = 100;
  robot.y = -100;
  robot.theta = 30;
  vr.clear();
  DangerAngles4C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-180,180)));

  robot.x = 100;
  robot.y = -999;
  robot.theta = 30;
  vr.clear();
  DangerAngles4C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(0,-179)));

  robot.x = 100;
  robot.y = -700;
  robot.theta = 0;
  vr.clear();
  DangerAngles4C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(-36,-143)));

  robot.x = 1500;
  robot.y = -999;
  robot.theta = 0;
  vr.clear();
  DangerAngles4C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(90,-179)));

  robot.x = 1500;
  robot.y = 0;
  robot.theta = 0;
  vr.clear();
  DangerAngles4C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(90,-90)));


  robot.x = 1200;
  robot.y = -800;
  robot.theta = 0;
  vr.clear();
  DangerAngles4C(robot,vr);
  EXPECT_EQ(vr[0],FiltroAngular(Seccion(53,-156)));

}
TEST(UAHRLidar, UAHRObjectsAngles)
{
    pose robot;
    VFiltros vf;


    robot.x = 1200;
    robot.y = -800;
    robot.theta = 0;
    ObjSearchData baliza_sup{1550, 1550, 850,1000,0};
    ObjSearchData baliza_inf{1540, 1540, -900,-1000,1};
    ObjSearchData baliza_medio{-1540, -1540, 25,-25,2};
    ObjSearchData torre{-150 , 150, 1000,  1000,3};
    ObjectsAngles(robot,baliza_sup,vf);

    //EXPECT_EQ((int)dist.distance,(int)1760.21);
    EXPECT_EQ((int)vf.size(),1);
    EXPECT_EQ((int)vf[0].motivo,OBJETO);

    EXPECT_EQ((int)vf[0].rpose.arco.start,68);
    EXPECT_EQ((int)vf[0].rpose.arco.end,88);
    
    vf.clear();
    robot.x = -400;
    robot.y = 0;
    robot.theta = 0;
    
    ObjectsAngles(robot,baliza_sup,vf);
    EXPECT_EQ((int)vf[0].rpose.arco.start,21);
    EXPECT_EQ((int)vf[0].rpose.arco.end,(int)29);
}
TEST(UAHRLidar,RelativeAngle)
{
  FiltroAngular f;
  pose pr(0,0,45);
  VFiltros vdf;
  
  f.rpose.arco.start = 10;
  f.rpose.arco.end  = 30;
  vdf.push_back(f);
  
  RelativeAngle(vdf[0] ,pr,vdf);
  EXPECT_EQ((int)vdf.size(),1);

  EXPECT_EQ((int)vdf[0].rpose.arco.start,-35);
  EXPECT_EQ((int)vdf[0].rpose.arco.end,-15);
  f.rpose.arco.start = 90;
  f.rpose.arco.end  = 0;
  vdf.push_back(f);
  f.rpose.arco.start = -170;
  f.rpose.arco.end  = -120;
  vdf.push_back(f);
  RelativeAngle(vdf[2] ,pr,vdf);
  EXPECT_EQ(vdf.size(),4);
  EXPECT_EQ(vdf[2].rpose.arco.start,-180);
  EXPECT_EQ(vdf[2].rpose.arco.end,-165);
  EXPECT_EQ(vdf[3].rpose.arco.start,145);
  EXPECT_EQ(vdf[3].rpose.arco.end,180);
  
  f.rpose.arco.start = -180+45;
  f.rpose.arco.end   = -120 +45;
  vdf.push_back(f);
  RelativeAngle(vdf[4] ,pr,vdf);
  EXPECT_EQ(vdf.size(),5);
  EXPECT_EQ(vdf[4].rpose.arco.start,-180);
  EXPECT_EQ(vdf[4].rpose.arco.end,-120);


  f.rpose.arco.start = 120 - 45;
  f.rpose.arco.end   = 180 - 45;
  vdf.push_back(f);
  RelativeAngle(vdf[5] ,pr,vdf);
  EXPECT_EQ(vdf.size(),6);
  EXPECT_EQ(vdf[5].rpose.arco.start,30);
  EXPECT_EQ(vdf[5].rpose.arco.end,90);


  f.rpose.arco.start = -180;
  f.rpose.arco.end   = 180 ;
  vdf.push_back(f);
  RelativeAngle(vdf[6] ,pr,vdf);
  EXPECT_EQ(vdf.size(),7);
  EXPECT_EQ(vdf[6].rpose.arco.start,-180);
  EXPECT_EQ(vdf[6].rpose.arco.end,180);


}

TEST(UAHRLidar, DesacoploAngulos)
{
  pose robot;
  CoronaCircular dist;
  VFiltros arcos;

  FiltroAngular arc_aux;
  arc_aux.motivo = OBJETO;
  arc_aux.rpose.arco.start = -180;
  arc_aux.rpose.arco.end   =  -30;
  arc_aux.salto =  true;
  arcos.push_back(arc_aux);
  arc_aux.motivo = ROBOT;
  arc_aux.rpose.arco.start = -180;
  arc_aux.rpose.arco.end   =  180;
  arc_aux.salto =  false;
  arcos.push_back(arc_aux);
  DesacoploAngulos(arcos);

  EXPECT_EQ(arcos.size(),2);
  EXPECT_EQ(arcos[0].rpose.arco,Seccion(-180,-30));
  EXPECT_EQ(arcos[1].rpose.arco.start,-30);
  EXPECT_EQ(arcos[1].rpose.arco.end,180);
  EXPECT_EQ(arcos[1].rpose.arco,Seccion(-30,180));

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
  DesacoploAngulos(arcos);
  EXPECT_EQ(arcos.size(),3);
  EXPECT_EQ(arcos[0].rpose.arco,Seccion(-180,-170));
  EXPECT_EQ(arcos[1].rpose.arco,Seccion(-170,-30));
  EXPECT_EQ(arcos[2].rpose.arco,Seccion(-30,180));

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

  DesacoploAngulos(arcos);
  EXPECT_EQ(arcos.size(),4);
  EXPECT_EQ(arcos[0].rpose.arco,Seccion(-180,-170));
  EXPECT_EQ(arcos[1].rpose.arco,Seccion(-170,-40));
  EXPECT_EQ(arcos[2].rpose.arco,Seccion(-40,-35));
  EXPECT_EQ(arcos[3].rpose.arco,Seccion(-35,30)); 

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

  DesacoploAngulos(arcos);
  EXPECT_EQ(arcos.size(),12);
  EXPECT_EQ(arcos[0].rpose.arco,Seccion(-180,-170));
  EXPECT_EQ(arcos[1].rpose.arco,Seccion(-170,-50));
  EXPECT_EQ(arcos[2].rpose.arco,Seccion(-50,-40));
  EXPECT_EQ(arcos[3].rpose.arco,Seccion(-40,-35));
  EXPECT_EQ(arcos[4].rpose.arco,Seccion(-35,-30));
  EXPECT_EQ(arcos[5].rpose.arco,Seccion(-30,-27));
  EXPECT_EQ(arcos[6].rpose.arco,Seccion(-27,30));
  EXPECT_EQ(arcos[7].rpose.arco,Seccion(30,52));
  EXPECT_EQ(arcos[8].rpose.arco,Seccion(130,140));
  EXPECT_EQ(arcos[9].rpose.arco,Seccion(140,150));
  EXPECT_EQ(arcos[10].rpose.arco,Seccion(150,160));
  EXPECT_EQ(arcos[11].rpose.arco,Seccion(165,180));
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
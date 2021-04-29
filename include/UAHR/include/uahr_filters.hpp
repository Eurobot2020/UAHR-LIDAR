#pragma once
#include "uahr_types.hpp"
#include "uahr_common.hpp"

#define DISTANCIA_SEGURIDAD 500
#define LIMX 1500
#define LIMY 1000

void ObjectsAngles(const pose &p_obs,const ObjSearchData &pasive_obj,VFiltros &vf);
void DangerAngles1C(const pose &pr, VFiltros &vdf);
void DangerAngles2C(const pose &pr, VFiltros &vdf);
void DangerAngles3C(const pose &pr, VFiltros &vdf);
void DangerAngles4C(const pose &pr, VFiltros &vdf);
void RelativeAngle(Seccion &f ,const pose &pr,VFiltros &vdf);
void AcoploAngulos(VFiltros &Vf);
void UpdateFilters(const bool & robot_localised,VFiltros &filtros,
    pose const &robot,VSearchObjects &SearchObjects);

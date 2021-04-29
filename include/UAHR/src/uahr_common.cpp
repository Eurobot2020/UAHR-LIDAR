#include "uahr_common.hpp"



bool GenerarAcoples(Seccion a1, Seccion a2)
{
   if(a1.start < a2.start) {
      return true;
    }
    else if(a1.start == a2.start) {
        if(a1.end > a2.end) 
            return true;
        else
            return false;
    }
    else
        return false;
}
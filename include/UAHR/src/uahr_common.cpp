#include "uahr_common.hpp"



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
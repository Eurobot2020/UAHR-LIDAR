#include "uahr_types.hpp"

// Funciones para los tests:
bool operator== (const Seccion one, const  Seccion two) {
    return ((int)one.start == (int)two.start && (int)one.end == (int)two.end);
}
Seccion operator+(const Seccion one, const  Seccion two) {
    return Seccion(one.start + two.start, one.end + two.end);
}
bool operator== (const CoronaCircular one, const  CoronaCircular two) {
    return (one.arco == two.arco && one.distance == two.distance);
}
bool operator== (const FiltroAngular one,const FiltroAngular two) {
    return (one.rpose == two.rpose 
        && one.motivo == two.motivo 
        && one.tipo == two.tipo);
}

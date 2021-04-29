#include "uahr_types.hpp"

// Funciones para los tests:
bool operator== (const Seccion one, const  Seccion two) {
    return ((int)one.start == (int)two.start && (int)one.end == (int)two.end);
}
Seccion operator+(const Seccion one, const  Seccion two) {
    return Seccion(one.start + two.start, one.end + two.end);
}


#ifndef RENDER_H
#define RENDER_H

#include "math.h"

void rasterizeTriangle(unsigned long *dest, const unsigned int &destWidth, const unsigned int &destHeight, const unsigned int &color,
                       VEC2 t0, VEC2 t1, VEC2 t2);

#endif // RENDER_H

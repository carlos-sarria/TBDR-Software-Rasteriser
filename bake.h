#ifndef BAKE_H
#define BAKE_H

#include "raster.h"

void bakeNormals (TEXTURE normalMap, unsigned short *indices, const unsigned int &numIndices, VERTEX *vertices);

#endif // BAKE_H

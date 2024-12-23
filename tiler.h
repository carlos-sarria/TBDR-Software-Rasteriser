#ifndef TILER_H
#define TILER_H

#include "math.h"
#include "api.h"

#define MEGABYTE 1048576
#define TILE_SIZE 8

struct T_TRI
{
    int indices[3];
    float barycentric[3];
    int material;
};

void tilerSetup(const int &width, const int &height, const unsigned int& bytesPM = MEGABYTE, void *debugBuffer = nullptr);

void tilerSetMaterial(const int &materialID);

void tilerSetTransformation(const float transformationMatrix[16]);

void tilerSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numTriangles);

#endif // TILER_H

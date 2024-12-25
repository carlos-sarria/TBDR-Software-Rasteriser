#ifndef TILER_H
#define TILER_H

#include "math.h"
#include "api.h"

#define MEGABYTE 1048576
#define TILE_SIZE 8

struct TR_VERTEX
{
    VEC3 pos;
    VEC2 uv;
    float intensity;
};

void tilerSetup(const int &width, const int &height, const unsigned int& bytesPM = MEGABYTE, void *debugBuffer = nullptr);

void tilerSetMaterial(TEXTURE texture);

void tilerSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numTriangles);

void tilerClear();

void tilerSetWorldMatrix(MATRIX m);

void tilerSetViewMatrix(MATRIX m);

void tilerSetProjectionMatrix(MATRIX m);

void tilerSetLight(VEC3 lightPosition);

#endif // TILER_H

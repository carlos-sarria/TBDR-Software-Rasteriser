#ifndef RASTER_H
#define RASTER_H

#include "math.h"

struct VERTEX
{
    VEC3 pos;
    VEC3 nor;
    VEC2 uv;
};

struct TEXTURE
{
    unsigned int width;
    unsigned int height;
    unsigned int *data;
};

enum CULLING
{
    CULL_NONE,
    CULL_BACK,
    CULL_FRONT
};

void rasterClear(unsigned int color=0x00000000, float depth=0.0f);

void rasterInitialise(const int &width, const int &height, void *debugBuffer = nullptr);

void rasterRelease();

void rasterSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices);

void rasterSetMaterial(TEXTURE texture);

void rasterSetWorldMatrix(MATRIX m);

void rasterSetViewMatrix(MATRIX m);

void rasterSetProjectionMatrix(MATRIX m);

void rasterSetLight(VEC3 pos);

#endif // RASTER_H

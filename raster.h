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

enum BLEND_MODE
{
    NONE,
    ALPHA,
    ADDITIVE
};

struct MATERIAL {
    TEXTURE baseColor;
    TEXTURE metallicRoughness;
    TEXTURE emissive;
    TEXTURE normal;
    TEXTURE reflection;
    BLEND_MODE blend_mode;
    float factor; // value from 0.0f to 1.0f
    unsigned int color; // used if texture is nullptr
    bool smooth_shade;
};

struct TR_VERTEX
{
    VEC3 pos;
    VEC2 uv;
    float intensity;
};

struct RENDER_STATE
{
    float  *depthBuffer;
    unsigned long *colorBuffer;

    unsigned int frameWidth;
    unsigned int frameHeight;

    MATERIAL material;

    VEC3 lightPosition;
    VEC3 eyePosition;

    VEC3 lightInvPosition;
    VEC3 eyeInvPosition;

    MATRIX worldMatrix;
    MATRIX projectionMatrix;
    MATRIX viewMatrix;
};

void rasterClear(unsigned int color=0x00000000, float depth=0.0f);

void rasterInitialise(const int &width, const int &height, void *debugBuffer = nullptr);

void rasterRelease();

void rasterSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices, VEC3 center);

void rasterSetMaterial(MATERIAL material);

void rasterSetWorldMatrix(MATRIX m);

void rasterSetViewMatrix(MATRIX m);

void rasterSetProjectionMatrix(MATRIX m);

void rasterSetLight(VEC3 pos);

void rasterSetEye(VEC3 pos);

#endif // RASTER_H

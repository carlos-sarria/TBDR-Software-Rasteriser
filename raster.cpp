#include "raster.h"
#include "log.h"
#include <stdlib.h>

#define COLOR_DEBUG 0

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

    TEXTURE texture;

    VEC3 lightPosition;

    MATRIX worldMatrix;
    MATRIX projectionMatrix;
    MATRIX viewMatrix;
} rs;

unsigned int colors[] = {0xFFFF0000,0xFF00FF00,0xFF0000FF,0xFFFFFF00,0xFFFF00FF,0xFF00FFFF};
unsigned int currColor = 0;

void rasterSetWorldMatrix(MATRIX m) { rs.worldMatrix = m; }

void rasterSetViewMatrix(MATRIX m) { rs.viewMatrix = m; }

void rasterSetProjectionMatrix(MATRIX m) { rs.projectionMatrix = m; }

void rasterSetLight(VEC3 pos) { rs.lightPosition = pos;};

void rasterInitialise(const int &width, const int &height, void *debugBuffer)
{
    rs.frameWidth = width;
    rs.frameHeight = height;
    rs.depthBuffer = (float *) malloc(rs.frameWidth*rs.frameHeight*sizeof(float));
    rs.colorBuffer = (unsigned long *)debugBuffer;
}

void rasterRelease()
{
    if(rs.depthBuffer) { free(rs.depthBuffer); rs.depthBuffer = nullptr;}
}

void rasterClear(unsigned int color, float depth)
{
    unsigned int i = rs.frameWidth*rs.frameHeight;
    while(i--)
    {
        rs.depthBuffer[i]= 0.0f;
        rs.colorBuffer[i] = 0x00000000;
    }

#if COLOR_DEBUG
    currColor = 0;
    srand(0);
#endif
}

void rasterSetMaterial(TEXTURE texture)
{
    rs.texture = texture;
}

#define GUARDBAND 0.0f
inline bool rasterCulling (VEC3 v0, VEC3 v1, VEC3 v2)
{
    // Backface culling: sign = ab*ac.y - ac.x*ab.y;
    if((v0.x-v1.x)*(v0.y-v2.y) - (v0.x-v2.x)*(v0.y-v1.y) >= 0.0f) return true;

    // Depth culling (front plane only) TODO: Front plane clipping
    if(v0.z<0.0f && v1.z<0.0f && v2.z<0.0f) return true;

    // Bounding box check (out-of-the-screen)
    VEC2INT BB_A( (v0.x>v1.x) ? (v0.x>v2.x) ? v0.x:v2.x:v1.x , (v0.y>v1.y) ? (v0.y>v2.y) ? v0.y:v2.y:v1.y);
    VEC2INT BB_B( (v0.x<v1.x) ? (v0.x<v2.x) ? v0.x:v2.x:v1.x , (v0.y<v1.y) ? (v0.y<v2.y) ? v0.y:v2.y:v1.y);
    VEC2INT BB_C( (BB_A.x + BB_B.x)/2, (BB_A.y + BB_B.y)/2);
    VEC2INT BB_D( abs(BB_A.x - BB_B.x)/2, abs(BB_A.y - BB_B.y)/2);
    VEC2INT BB_S (rs.frameWidth/2, rs.frameHeight/2);

    if( abs(BB_S.x-BB_C.x)>(BB_S.x+BB_D.x) &&
        abs(BB_S.y-BB_C.y)>(BB_S.y+BB_D.y) ) return true;

    return false;
}

#define SWAP(a,b) {int temp; temp=a; a=b; b=temp;}
#define DOT(v0,v1) (v0.x*v1.x+v0.y*v1.y)
void rasterRasterize(unsigned short *indices, const unsigned int &numIndices, TR_VERTEX *meshTVB)
{
    for (int indexCount = 0; indexCount < numIndices;)
    {
#if COLOR_DEBUG
        float debugShade = float(rand()%128)/255.0f+0.5f;
#endif

        unsigned short in0 = indices[indexCount++];
        unsigned short in1 = indices[indexCount++];
        unsigned short in2 = indices[indexCount++];

        if (rasterCulling(meshTVB[in0].pos, meshTVB[in1].pos, meshTVB[in2].pos)) continue;

        if (meshTVB[in0].pos.y > meshTVB[in1].pos.y) SWAP(in0, in1);
        if (meshTVB[in0].pos.y > meshTVB[in2].pos.y) SWAP(in0, in2);
        if (meshTVB[in1].pos.y > meshTVB[in2].pos.y) SWAP(in1, in2);

        TR_VERTEX v0 = meshTVB[in0];
        TR_VERTEX v1 = meshTVB[in1];
        TR_VERTEX v2 = meshTVB[in2];

        int total_height = v2.pos.y - v0.pos.y;

        // Barycentric coordinates
        VEC3 bry0 = v1.pos - v0.pos;
        VEC3 bry1 = v2.pos - v0.pos;
        float d00 = DOT(bry0, bry0);
        float d01 = DOT(bry0, bry1);
        float d11 = DOT(bry1, bry1);
        float invD = 1.0 / (d00 * d11 - d01 * d01);

        for (int i = 0; i <= total_height; i++)
        {
            int y = int(v0.pos.y)+i;

            if(y<0 || y>=rs.frameHeight) continue;

            bool  second_half = (y > v1.pos.y || v1.pos.y == v0.pos.y);
            float segment_height, alpha = 1.0f, beta = 1.0f;

            int a, b;

            if(total_height>0) alpha = (float)i / (float)total_height;
            a = int(v0.pos.x + (v2.pos.x - v0.pos.x) * alpha);

            if(second_half)
            {
                segment_height = ceil(v2.pos.y - v1.pos.y);
                if(segment_height > 0.0f) beta = (float)(y - v1.pos.y) / segment_height;
                b = int(v1.pos.x + (v2.pos.x - v1.pos.x) * beta);
            }
            else
            {
                segment_height = ceil(v1.pos.y - v0.pos.y);
                if(segment_height > 0.0f) beta = (float)(i) / segment_height;
                b = int(v0.pos.x + (v1.pos.x - v0.pos.x) * beta);
            }

            if (a > b) SWAP(a, b);

            unsigned long incr = (a + y * rs.frameWidth);

            for (int x = a; x < b; x++)
            {
                if(x<0 || x>=rs.frameWidth) { incr++; continue; }

                VEC3 p(x, i+v0.pos.y, 0.0f);
                VEC3 bry2 = p - v0.pos;
                float d20 = DOT(bry2, bry0);
                float d21 = DOT(bry2, bry1);
                float v = (d11 * d20 - d01 * d21) * invD;
                float w = (d00 * d21 - d01 * d20) * invD;
                float u = 1.0f - v - w;

                float depth = v0.pos.z*u + v1.pos.z*v + v2.pos.z*w;

                if(depth>=rs.depthBuffer[incr]) // Depth comparison
                {
                    // Update the depth buffer with the new value
                    rs.depthBuffer[incr] = depth;

                    // Perspective correct texture coordinates
                    unsigned int texU = (unsigned int)((v0.uv.u*u + v1.uv.u*v + v2.uv.u*w) * rs.texture.width / depth) % rs.texture.width;
                    unsigned int texV = (unsigned int)((v0.uv.v*u + v1.uv.v*v + v2.uv.v*w) * rs.texture.width / depth) % rs.texture.height;

                    // Smooth shading
                    float Shade = (v0.intensity*u + v1.intensity*v + v2.intensity*w)  / depth;
#if COLOR_DEBUG
                    Shade = debugShade;
#endif
                    // Draw the pixel on the screen buffer
                    unsigned int c = rs.texture.data[texU+texV*rs.texture.width];
#if COLOR_DEBUG
                    c = colors[currColor%6];
#endif
                    unsigned int r = int((float)(c>>16&0xFF)*Shade);
                    unsigned int g = int((float)(c>>8&0xFF)*Shade);
                    unsigned int b = int((float)(c&0xFF)*Shade);
                    rs.colorBuffer[incr] = 0xFF<<24|r<<16|g<<8|b;
                }

                incr++;
            }
        }
    }
}

void rasterTransform (VERTEX *v, const unsigned int &numVertices, TR_VERTEX *meshTVB)
{
    MATRIX mMVP = rs.worldMatrix * rs.viewMatrix * rs.projectionMatrix;

    MATRIX invW = rs.worldMatrix;
    invW.inverse();

    VEC3 transformedLight = invW * rs.lightPosition;

    for (int i = 0; i < numVertices; i++)
    {
        VEC3 out;

        out.x = v[i].pos.x*mMVP.f[0] + v[i].pos.y*mMVP.f[4] + v[i].pos.z*mMVP.f[8]  + 1.0f*mMVP.f[12];
        out.y = v[i].pos.x*mMVP.f[1] + v[i].pos.y*mMVP.f[5] + v[i].pos.z*mMVP.f[9]  + 1.0f*mMVP.f[13];
        out.z = v[i].pos.x*mMVP.f[2] + v[i].pos.y*mMVP.f[6] + v[i].pos.z*mMVP.f[10] + 1.0f*mMVP.f[14];

        VEC3 light = transformedLight-v[i].pos;
        light.normalize();
        float shade = (v[i].nor.x*light.x + v[i].nor.y*light.y + v[i].nor.z*light.z) * 0.5 + 0.5f;

        float div = 1.0f/out.z;
        meshTVB[i].pos.x = out.x * div * (rs.frameWidth/2) + rs.frameWidth/2;
        meshTVB[i].pos.y = out.y * div * (rs.frameWidth/2) + rs.frameHeight/2;
        meshTVB[i].pos.z = div;

        meshTVB[i].uv.u = v[i].uv.u * div;
        meshTVB[i].uv.v = v[i].uv.v * div;

        meshTVB[i].intensity = shade * div;
    }
}

void rasterSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices)
{
    TR_VERTEX *meshTVB = (TR_VERTEX *)malloc(numVertices*sizeof(TR_VERTEX));

    rasterTransform (vertices, numVertices, meshTVB);
    rasterRasterize (indices, numIndices, meshTVB);

#if COLOR_DEBUG
    currColor++;
#endif

    free(meshTVB);
}

#include "raster.h"
#include "log.h"
#include <stdlib.h>

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

inline unsigned int intensity (unsigned int color, float intensity)
{
    unsigned int c = color;
    unsigned int r = (unsigned int)((float)(c>>16&0xFF)*intensity);
    unsigned int g = (unsigned int)((float)(c>>8&0xFF)*intensity);
    unsigned int b = (unsigned int)((float)(c&0xFF)*intensity);
    c = 0xFF<<24|r<<16|g<<8|b;

    return c;
}

// Barycentric coordinates. Cramer's rule to solve linear systems
inline void barycentric (VEC3 v0, VEC3 v1, VEC3 v2, VEC3 a, VEC3 b, VEC3 &by1, VEC3 &by2, VEC3 &by_incr)
{
    VEC3 br0 = v1 - v0;
    VEC3 br1 = v2 - v0;
    float d00 = DOT(br0, br0);
    float d01 = DOT(br0, br1);
    float d11 = DOT(br1, br1);
    float invD = 1.0 / (d00 * d11 - d01 * d01);

    VEC3 br2 = a - v0;
    float d20 = DOT(br2, br0);
    float d21 = DOT(br2, br1);
    by1.y = (d11 * d20 - d01 * d21) * invD;
    by1.z = (d00 * d21 - d01 * d20) * invD;
    by1.x = 1.0f - by1.y - by1.z;

    VEC3 br3 = b - v0;
    float d30 = DOT(br3, br0);
    float d31 = DOT(br3, br1);
    by2.y = (d11 * d30 - d01 * d31) * invD;
    by2.z = (d00 * d31 - d01 * d30) * invD;
    by2.x = 1.0f - by2.y - by2.z;

    float d = 1.0f/float(b.x-a.x);
    by_incr = (by2-by1)*d;
}

void rasterRasterize(unsigned short *indices, const unsigned int &numIndices, TR_VERTEX *meshTVB)
{
    for (int indexCount = 0; indexCount < numIndices;)
    {
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

        for (int i = 0; i <= total_height; i++)
        {
            int y = int(v0.pos.y)+i;

            if(y<0 || y>=rs.frameHeight) continue;

            float alpha = 1.0f, beta = 1.0f;

            int a, b;

            if(total_height>0) alpha = (float)i / (float)total_height;
            a = int(v0.pos.x + (v2.pos.x - v0.pos.x) * alpha);

            if(y > v1.pos.y || v1.pos.y == v0.pos.y) // bottom half of the triangle
            {
                float h = v2.pos.y - v1.pos.y;
                if(h > 0.0f) beta = (float)(y - v1.pos.y) / h;
                b = int(v1.pos.x + (v2.pos.x - v1.pos.x) * beta);
            }
            else
            {
                float h = v1.pos.y - v0.pos.y;
                if(h > 0.0f) beta = (float)(i) / h;
                b = int(v0.pos.x + (v1.pos.x - v0.pos.x) * beta);
            }

            if (a > b) SWAP(a, b);

            float d = 1.0f/float(b-a);

            VEC3 vA(a,y,0.0f), vB(b,y,0.0f), by, byB, by_incr;
            barycentric (v0.pos, v1.pos, v2.pos, vA, vB, by, byB, by_incr);

            float depth =  v0.pos.z*by.x + v1.pos.z*by.y + v2.pos.z*by.z;
            float depthB =  v0.pos.z*byB.x + v1.pos.z*byB.y + v2.pos.z*byB.z;
            float depth_incr = (depthB-depth)*d;

            float invD = 1.0f/depth;
            float texU = ((v0.uv.u*by.x + v1.uv.u*by.y + v2.uv.u*by.z) * rs.texture.width / depth);
            float texUB = ((v0.uv.u*byB.x + v1.uv.u*byB.y + v2.uv.u*byB.z) * rs.texture.width / depthB);
            float texU_incr = (texUB-texU)*d;

            float texV = ((v0.uv.v*by.x + v1.uv.v*by.y + v2.uv.v*by.z) * rs.texture.width / depth);
            float texVB = ((v0.uv.v*byB.x + v1.uv.v*byB.y + v2.uv.v*byB.z) * rs.texture.height / depthB);
            float texV_incr = (texVB-texV)*d;

            float shade = (v0.intensity*by.x + v1.intensity*by.y + v2.intensity*by.z) * invD;
            float shadeB = (v0.intensity*byB.x + v1.intensity*byB.y + v2.intensity*byB.z) * invD;
            float shade_incr = (shadeB-shade)*d;

            unsigned long incr = (a + y * rs.frameWidth);

            for (int x = a; x <= b; x++)
            {
                if(x>=0 && x<rs.frameWidth)
                {
                    if(depth>=rs.depthBuffer[incr]) // Depth comparison GREATER-EQUAL
                    {
                        // Update the depth buffer with the new value
                        rs.depthBuffer[incr] = depth;

                        // Perspective correct texture coordinates
                        unsigned int texU_int = (unsigned int)(texU) & (rs.texture.width-1);
                        unsigned int texV_int = (unsigned int)(texV) & (rs.texture.height-1);

                        // Draw the pixel on the screen buffer
#if 1
                        rs.colorBuffer[incr] = intensity (rs.texture.data[texU_int+texV_int*rs.texture.width], shade);
#else
                        rs.colorBuffer[incr] = rs.texture.data[texU_int+texV_int*rs.texture.width];
#endif
                    }
                }

                by.x += by_incr.x;
                by.y += by_incr.y;
                by.z += by_incr.z;
                depth += depth_incr;
                texU += texU_incr;
                texV += texV_incr;
                shade += shade_incr;
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
        meshTVB[i].pos.x = ceil(out.x * div * (rs.frameWidth/2) + rs.frameWidth/2); // ceil = pixel left-top convention
        meshTVB[i].pos.y = ceil(out.y * div * (rs.frameWidth/2) + rs.frameHeight/2);
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

    free(meshTVB);
}

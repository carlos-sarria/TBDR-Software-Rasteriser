#include "tiler.h"
#include "log.h"
#include <stdlib.h>

VERTEX *tilerVB;
int    *tilerPB;
float  *tilerDB;

int   frameWidth, frameHeight;
int   frameArea;
TEXTURE tilerTexture;
unsigned int colors[5] = {0x00FF0000,0x0000FF00,0x000000FF,0x00FFFF00,0x0000FFFF};
unsigned long *tilerDebugBuffer;
VEC3 tilerLightPosition;

MATRIX tilerWorld;
MATRIX tilerProjection;
MATRIX tilerView;

void tilerSetWorldMatrix(MATRIX m) { tilerWorld = m; }
void tilerSetViewMatrix(MATRIX m) { tilerView = m; }
void tilerSetProjectionMatrix(MATRIX m) { tilerProjection = m; }
void tilerSetLight(VEC3 lightPosition) { tilerLightPosition = lightPosition;};

void tilerSetup(const int &width, const int &height, const unsigned int& bytesPM, void *debugBuffer)
{
    frameWidth = width;
    frameHeight = height;
    tilerVB = (VERTEX *) malloc(bytesPM);
    tilerPB = (int *) malloc(bytesPM);
    tilerDB = (float *) malloc(frameWidth*frameHeight*sizeof(float));
    frameArea = frameWidth*frameHeight;
    tilerDebugBuffer = (unsigned long *)debugBuffer;
}

void tilerRelease()
{
    if(tilerVB) { free(tilerVB); tilerVB = nullptr;}
    if(tilerPB) { free(tilerPB); tilerPB = nullptr;}
    if(tilerDB) { free(tilerDB); tilerDB = nullptr;}
}

void tilerClear()
{
    unsigned int i = frameArea;
    while(i--)
    {
        tilerDB[i]= 0.0f;
        tilerDebugBuffer[i] = 0x00000000;
    }
}

void tilerSetMaterial(TEXTURE texture)
{
    tilerTexture = texture;
}

#define GUARDBAND 0.0f
inline bool tilerCulling (VEC3 t0, VEC3 t1, VEC3 t2)
{
    // Backface culling: sign = ab.x*ac.y - ac.x*ab.y;
    if((t0.x-t1.x)*(t0.y-t2.y) - (t0.x-t2.x)*(t0.y-t1.y) >= 0.0f) return true;

    // Visibility check
    // If there is at least one vertex inside, then the triangle is visible
    // This will not work with some triangles visible but with all vertices ouitside
    // if(t0.x>-GUARDBAND && t0.x<frameWidth+GUARDBAND && t0.y>-GUARDBAND && t0.y<frameHeight+GUARDBAND) return false;
    // if(t1.x>-GUARDBAND && t1.x<frameWidth+GUARDBAND && t1.y>-GUARDBAND && t1.y<frameHeight+GUARDBAND) return false;
    // if(t2.x>-GUARDBAND && t2.x<frameWidth+GUARDBAND && t2.y>-GUARDBAND && t2.y<frameHeight+GUARDBAND) return false;
    // return true; // Triangle fully outside the screen

    return false;
}

#define dot(v0,v1) (v0.x*v1.x+v0.y*v1.y)
void tilerRasterize(unsigned short *indices, const unsigned int &numIndices, TR_VERTEX *meshTVB)
{
    for (int idx = 0; idx < numIndices;)
    {
        TR_VERTEX v0 = meshTVB[indices[idx++]];
        TR_VERTEX v1 = meshTVB[indices[idx++]];
        TR_VERTEX v2 = meshTVB[indices[idx++]];

        if (tilerCulling(v0.pos, v1.pos, v2.pos)) continue;

        if (v0.pos.y > v1.pos.y) std::swap(v0, v1);
        if (v0.pos.y > v2.pos.y) std::swap(v0, v2);
        if (v1.pos.y > v2.pos.y) std::swap(v1, v2);

        VEC3 t0 = v0.pos, t1 = v1.pos, t2 = v2.pos;
        int total_height = t2.y - t0.y;

        // Barycentric coordinates
        VEC3 bry0 = t1 - t0, bry1 = t2 - t0;
        float d00 = dot(bry0, bry0);
        float d01 = dot(bry0, bry1);
        float d11 = dot(bry1, bry1);
        float invDenom = 1.0 / (d00 * d11 - d01 * d01);

        for (int i = 0; i < total_height; i++)
        {
            bool second_half = i > t1.y - t0.y || t1.y == t0.y;
            int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
            float alpha = (float)i / total_height;
            float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height;

            int Ax, Ay, Bx, By;
            Ax = int(t0.x + (t2.x - t0.x) * alpha);
            Ay = int(t0.y + (t2.y - t0.y) * alpha);
            Bx = int(second_half ? t1.x + (t2.x - t1.x) * beta : t0.x + (t1.x - t0.x) * beta);
            By = int(second_half ? t1.y + (t2.y - t1.y) * beta : t0.y + (t1.y - t0.y) * beta);
            if (Ax > Bx) { std::swap(Ax, Bx); std::swap(Ay, By);}

            unsigned long incr = (Ax + ((int)t0.y + i) * frameWidth);
            if(incr < frameArea) // ??
            {
                for (int x = Ax; x <= Bx; x++)
                {
                    if(x>=0 && x<frameWidth && incr < frameArea)
                    {
                        VEC3 p(x, i+t0.y, 0.0f);
                        VEC3 bry2 = p - t0;
                        float d20 = dot(bry2, bry0);
                        float d21 = dot(bry2, bry1);
                        float v = (d11 * d20 - d01 * d21) * invDenom;
                        float w = (d00 * d21 - d01 * d20) * invDenom;
                        float u = 1.0f - v - w;

                        float depth = v0.pos.z*u + v1.pos.z*v + v2.pos.z*w;

                        if(depth>=tilerDB[incr])
                        {
                            // Update the depth buffer with the new value
                            tilerDB[incr] = depth;

                            // Perspective correct texture coordinates
                            unsigned int texU = (unsigned int)((v0.uv.u*u + v1.uv.u*v + v2.uv.u*w) * tilerTexture.width / depth) % tilerTexture.width;
                            unsigned int texV = (unsigned int)((v0.uv.v*u + v1.uv.v*v + v2.uv.v*w) * tilerTexture.width / depth) % tilerTexture.height;

                            // Smooth shading
                            float Shade = (v0.intensity*u + v1.intensity*v + v2.intensity*w)  / depth;

                            // Draw the pixel on the screen buffer
                            unsigned int c = tilerTexture.data[texU+texV*tilerTexture.width];
                            unsigned int r = int((float)(c>>16&0xFF)*Shade);
                            unsigned int g = int((float)(c>>8&0xFF)*Shade);
                            unsigned int b = int((float)(c&0xFF)*Shade);
                            tilerDebugBuffer[incr] = 0xFF<<24|r<<16|g<<8|b;
                        }
                    }
                    incr++;
                }
            }
        }
    }
}

void tilerTransform (VERTEX *v, const unsigned int &numVertices, TR_VERTEX *meshTVB)
{
    MATRIX mMVP = tilerWorld * tilerView * tilerProjection;

    for (int i = 0; i < numVertices; i++)
    {
        VEC3 out;

        out.x = v[i].pos.x*mMVP.f[0] + v[i].pos.y*mMVP.f[4] + v[i].pos.z*mMVP.f[8]  + 1.0f*mMVP.f[12];
        out.y = v[i].pos.x*mMVP.f[1] + v[i].pos.y*mMVP.f[5] + v[i].pos.z*mMVP.f[9]  + 1.0f*mMVP.f[13];
        out.z = v[i].pos.x*mMVP.f[2] + v[i].pos.y*mMVP.f[6] + v[i].pos.z*mMVP.f[10] + 1.0f*mMVP.f[14];

        VEC3 light = tilerLightPosition-v[i].pos;
        light.normalize();
        float shade = (v[i].nor.x*light.x + v[i].nor.y*light.y + v[i].nor.z*light.z) * 0.5 + 0.5f;

        float div = 1.0f/out.z;
        meshTVB[i].pos.x = out.x * div * (frameWidth/2) + frameWidth/2;
        meshTVB[i].pos.y = out.y * div * (frameWidth/2) + frameHeight/2;
        meshTVB[i].pos.z = div;

        meshTVB[i].uv.u = v[i].uv.u * div;
        meshTVB[i].uv.v = v[i].uv.v * div;

        meshTVB[i].intensity = shade * div;
    }
}

void tilerSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices)
{
    TR_VERTEX *meshTVB = (TR_VERTEX *)malloc(numVertices*sizeof(TR_VERTEX));

    tilerTransform (vertices, numVertices, meshTVB);
    tilerRasterize (indices, numIndices, meshTVB);

    free(meshTVB);
}

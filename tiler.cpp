#include "tiler.h"
#include "log.h"
#include <stdlib.h>

VERTEX *tilerVB;
int    *tilerPB;
float  *tilerDB;

int   frameWidth, frameHeight;
int   frameArea;
TEXTURE tilerTexture;
float tilerTM[16];
unsigned int colors[5] = {0x00FF0000,0x0000FF00,0x000000FF,0x00FFFF00,0x0000FFFF};
unsigned long *tilerDebugBuffer;

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

void tilerSetTransformation(const float transformationMatrix[16])
{
    memcpy (tilerTM, transformationMatrix, sizeof(float)*16);
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

                            // Draw the pixel on the screen buffer
                            tilerDebugBuffer[incr] = tilerTexture.data[texU+texV*tilerTexture.width];
                        }
                    }
                    incr++;
                }
            }
        }
    }
}

inline VEC3 tilerTransform (VERTEX *v, const unsigned int &numVertices, TR_VERTEX *meshTVB)
{ 
    for (int i = 0; i < numVertices; i++)
    {
        VEC3 out;

        out.x = v[i].pos.x*tilerTM[0] + v[i].pos.y*tilerTM[4] + v[i].pos.z*tilerTM[8]  + 1.0f*tilerTM[12];
        out.y = v[i].pos.x*tilerTM[1] + v[i].pos.y*tilerTM[5] + v[i].pos.z*tilerTM[9]  + 1.0f*tilerTM[13];
        out.z = v[i].pos.x*tilerTM[2] + v[i].pos.y*tilerTM[6] + v[i].pos.z*tilerTM[10] + 1.0f*tilerTM[14];

        float div = 1.0f/out.z;
        meshTVB[i].pos.x = out.x * div * (frameWidth/2) + frameWidth/2;
        meshTVB[i].pos.y = out.y * div * (frameWidth/2) + frameHeight/2;
        meshTVB[i].pos.z = div;

        meshTVB[i].uv.u = v[i].uv.u * div;
        meshTVB[i].uv.v = v[i].uv.v * div;

        meshTVB[i].intensity = 1.0f;
    }
}

void tilerSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices)
{
    TR_VERTEX *meshTVB = (TR_VERTEX *)malloc(numVertices*sizeof(TR_VERTEX));

    tilerTransform (vertices, numVertices, meshTVB);
    tilerRasterize (indices, numIndices, meshTVB);

    free(meshTVB);
}

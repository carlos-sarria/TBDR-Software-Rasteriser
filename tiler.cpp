#include "tiler.h"
#include "log.h"
#include <stdlib.h>

T_TRI  *tilerTB;
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
    tilerTB = (T_TRI *) malloc(bytesPM);
    tilerVB = (VERTEX *) malloc(bytesPM);
    tilerPB = (int *) malloc(bytesPM);
    tilerDB = (float *) malloc(frameWidth*frameHeight*sizeof(float));
    frameArea = frameWidth*frameHeight;
    tilerDebugBuffer = (unsigned long *)debugBuffer;
}

void tilerRelease()
{
    if(tilerTB) { free(tilerTB); tilerTB = nullptr;}
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

#define dot(v0,v1) (v0.x*v1.x+v0.y*v1.y)

void tilerRasterize(VERTEX in_v0, VERTEX in_v1, VERTEX in_v2, unsigned int color)
{
    VERTEX v0=in_v0, v1=in_v1, v2=in_v2;
    if (v0.pos.y > v1.pos.y) std::swap(v0, v1);
    if (v0.pos.y > v2.pos.y) std::swap(v0, v2);
    if (v1.pos.y > v2.pos.y) std::swap(v1, v2);

    VEC3 t0 = v0.pos, t1 = v1.pos, t2 = v2.pos;
    int total_height = t2.y - t0.y;

    // Barycentric coordinates
    VEC3 b0 = t1 - t0, b1 = t2 - t0;
    float d00 = dot(b0, b0);
    float d01 = dot(b0, b1);
    float d11 = dot(b1, b1);
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
                    VEC3 b2 = p - t0;
                    float d20 = dot(b2, b0);
                    float d21 = dot(b2, b1);
                    float v = (d11 * d20 - d01 * d21) * invDenom;
                    float w = (d00 * d21 - d01 * d20) * invDenom;
                    float u = 1.0f - v - w;

                    float depth = v0.pos.z*u + v1.pos.z*v + v1.pos.z*w;

                    if(depth>=tilerDB[incr])
                    {

                        tilerDB[incr] = depth; // Update the depth buffer with the new value

                        unsigned int texU = (unsigned int)((v0.uv.u*u + v1.uv.u*v + v2.uv.u*w)*tilerTexture.width)%tilerTexture.width;
                        unsigned int texV = (unsigned int)((v0.uv.v*u + v1.uv.v*v + v2.uv.v*w)*tilerTexture.height)%tilerTexture.height;

                        tilerDebugBuffer[incr] = tilerTexture.data[texU+texV*tilerTexture.width];
                    }
                }
                incr++;
            }
        }
    }
}

inline VEC3 tilerTransform (VEC3 pos)
{
    VEC3 out;
    VEC3 screenCenter(frameWidth/2,frameHeight/2, 0.0f);

    out.x = pos.x*tilerTM[0] + pos.y*tilerTM[4] + pos.z*tilerTM[8]  + 1.0f*tilerTM[12];
    out.y = pos.x*tilerTM[1] + pos.y*tilerTM[5] + pos.z*tilerTM[9]  + 1.0f*tilerTM[13];
    out.z = pos.x*tilerTM[2] + pos.y*tilerTM[6] + pos.z*tilerTM[10] + 1.0f*tilerTM[14];

    float div = screenCenter.x/out.z;
    out.x = out.x * div;
    out.y = out.y * div;
    out.z = 1.0f / out.z;
    out = out + screenCenter;

    return out;
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

void tilerSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numTriangles)
{
    for (int i=0; i<numTriangles*3;)
    {
        VERTEX v0, v1, v2;

        v0 = vertices[indices[i]];
        v0.pos = tilerTransform (vertices[indices[i++]].pos);
        v1 = vertices[indices[i]];
        v1.pos = tilerTransform (vertices[indices[i++]].pos);
        v2 = vertices[indices[i]];
        v2.pos = tilerTransform (vertices[indices[i++]].pos);

        if (tilerCulling(v0.pos, v1.pos, v2.pos)) continue;

        tilerRasterize(v0,v1,v2,colors[i%5]);
    }
}

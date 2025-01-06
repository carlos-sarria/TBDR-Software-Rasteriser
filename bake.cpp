#include "bake.h"
#include "math.h"
#include "raster.h"

void bakeNormals (TEXTURE normalMap, unsigned short *indices, const unsigned int &numIndices, VERTEX *vertices)
{
    for (int indexCount = 0; indexCount < numIndices;)
    {
        unsigned short in0 = indices[indexCount++];
        unsigned short in1 = indices[indexCount++];
        unsigned short in2 = indices[indexCount++];

        if (vertices[in0].uv.v > vertices[in1].uv.v) SWAP(in0, in1);
        if (vertices[in0].uv.v > vertices[in2].uv.v) SWAP(in0, in2);
        if (vertices[in1].uv.v > vertices[in2].uv.v) SWAP(in1, in2);

        VEC2 uv[3];
        uv[0] = VEC2(ceil(vertices[in0].uv.u*normalMap.width), ceil(vertices[in0].uv.v*normalMap.height));
        uv[1] = VEC2(ceil(vertices[in1].uv.u*normalMap.width), ceil(vertices[in1].uv.v*normalMap.height));
        uv[2] = VEC2(ceil(vertices[in2].uv.u*normalMap.width), ceil(vertices[in2].uv.v*normalMap.height));

        VEC3 normal[3];
        normal[0] = vertices[in0].nor;
        normal[1] = vertices[in1].nor;
        normal[2] = vertices[in2].nor;

        int total_height = uv[2].y - uv[0].y;

        VEC2 br0 = uv[1] - uv[0];
        VEC2 br1 = uv[2] - uv[0];
        float d00 = DOT(br0, br0);
        float d01 = DOT(br0, br1);
        float d11 = DOT(br1, br1);
        float invD = 1.0 / (d00 * d11 - d01 * d01);

        for (int i = 0; i < total_height; i++)
        {
            int y = int(uv[0].y)+i;

            if(y<0 || y>=normalMap.height) continue;

            float alpha = 1.0f, beta = 1.0f;

            int a, b;

            if(total_height>0) alpha = (float)i / (float)total_height;
            a = int(uv[0].x + (uv[2].x - uv[0].x) * alpha);

            if(y > uv[1].y || uv[1].y == uv[0].y) // bottom half of the triangle
            {
                float h = uv[2].y - uv[1].y;
                if(h > 0.0f) beta = (float)(y - uv[1].y) / h;
                b = int(uv[1].x + (uv[2].x - uv[1].x) * beta);
            }
            else
            {
                float h = uv[1].y - uv[0].y;
                if(h > 0.0f) beta = (float)(i) / h;
                b = int(uv[0].x + (uv[1].x - uv[0].x) * beta);
            }

            if (a > b) SWAP(a, b);

            float d = 1.0f/float(b-a);

            VEC2 vA(a,y), vB(b,y);
            VEC3 byA, byB;

            VEC2 br2 = vA - uv[0];
            float d20 = DOT(br2, br0);
            float d21 = DOT(br2, br1);
            byA.y = (d11 * d20 - d01 * d21) * invD;
            byA.z = (d00 * d21 - d01 * d20) * invD;
            byA.x = 1.0f - byA.y - byA.z;

            br2 = vB - uv[0];
            d20 = DOT(br2, br0);
            d21 = DOT(br2, br1);
            byB.y = (d11 * d20 - d01 * d21) * invD;
            byB.z = (d00 * d21 - d01 * d20) * invD;
            byB.x = 1.0f - byB.y - byB.z;

            VEC3 n, n_incr, vN, vNB;
            vN = VEC3(normal[0].x, normal[1].x, normal[2].x);
            n.x      =  vN.dot(byA);
            vNB.x    =  vN.dot(byB);
            n_incr.x = (vNB.x-n.x)*d;

            vN = VEC3(normal[0].y, normal[1].y, normal[2].y);
            n.y      =  vN.dot(byA);
            vNB.y    =  vN.dot(byB);
            n_incr.y = (vNB.y-n.y)*d;

            vN = VEC3(normal[0].z, normal[1].z, normal[2].z);
            n.z      =  vN.dot(byA);
            vNB.z    =  vN.dot(byB);
            n_incr.z = (vNB.z-n.z)*d;

            unsigned long incr = (a + y * normalMap.width);

            for (int x = a; x < b; x++)
            {
                if(x>=0 && x<normalMap.width)
                {
                    VEC3 normal;

                    // Normal maps have the base vector pointing up: (0,0,1.0) or (0,0,128) or (128,128,255)
                    unsigned int tex_color = normalMap.data[incr];
                    normal.x = (float(tex_color>>16&0xFF) - 128.0f) / 128.0f;
                    normal.y = (float(tex_color>>8&0xFF)  - 128.0f) / 128.0f;
                    normal.z = (float(tex_color&0xFF)     - 128.0f) / 128.0f - 0.0f;

                    int r = int((n.x-normal.x) * 128.0f + 128.0f);
                    int g = int((n.y-normal.y) * 128.0f + 128.0f);
                    int b = int((n.z-normal.z) * 128.0f + 128.0f);

                    if(r<0) r=abs(r); if(r>255) r=510-r;
                    if(g<0) g=abs(g); if(g>255) g=510-g;
                    if(b<0) b=abs(b); if(b>255) b=510-b;

                    normalMap.data[incr] = 0xFF<<24|(r&0xFF)<<16|(g&0xFF)<<8|(b&0xFF);
                }
                n = n + n_incr;
                incr++;
            }
        }
    }
}

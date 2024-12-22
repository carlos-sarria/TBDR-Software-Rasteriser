#include "render.h"
#include "math.h"

struct INTVEC2
{
    int x;
    int y;
};

void rasterizeTriangle(unsigned long *dest, const unsigned int &destWidth, const unsigned int &destHeight, const unsigned int &color,
                       VEC2 t0, VEC2 t1, VEC2 t2)
{
    if (t0.y > t1.y) std::swap(t0, t1);
    if (t0.y > t2.y) std::swap(t0, t2);
    if (t1.y > t2.y) std::swap(t1, t2);
    int total_height = t2.y - t0.y;

    for (int i = 0; i < total_height; i++)
    {
        bool second_half = i > t1.y - t0.y || t1.y == t0.y;
        int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
        float alpha = (float)i / total_height;
        float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height;
        INTVEC2 A, B;
        A.x = int(t0.x + (t2.x - t0.x) * alpha);
        B.x = int(second_half ? t1.x + (t2.x - t1.x) * beta : t0.x + (t1.x - t0.x) * beta);
        A.y = int(t0.y + (t2.y - t0.y) * alpha);
        B.y = int(second_half ? t1.y + (t2.y - t1.y) * beta : t0.y + (t1.y - t0.y) * beta);
        if (A.x > B.x) std::swap(A, B);
        unsigned long incr = (A.x + ((int)t0.y + i) * destWidth);
        if(incr < destWidth*destHeight) // ??
        {
            //unsigned long *pData = dest + incr;
            for (int x = A.x; x <= B.x; x++)
            {
                if(incr > 0 && incr < destWidth*destHeight) dest[incr] = color;
                incr++;
            }
       }
    }
}


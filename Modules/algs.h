#ifndef __ALGS_H
#define __ALGS_H

#include "main.h"

//type define
typedef struct 
{
    float Angle;
    float AngleRaw;
    float zeroPosition;
    float speed;
}jointParam;

typedef enum
{
    AlgsModeIdle = 1,
    AlgsModeJog,
    AlgsModePtp,
}AlgsMode_t;


#endif


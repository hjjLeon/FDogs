#ifndef __ALGS_H
#define __ALGS_H

#include "main.h"

#define ROBOT_LEG_NUM 1
#define ROBOT_LEG_JOINT_NUM 1

//type define
typedef struct 
{
    float angleCurrent;
    float angleTarget;
    float angleRaw;
    uint32_t periodTickCount;

    float angleMax;//in angle
    float angleMin;//in angle
    float speed;//in angle
    float zeroPosition;//in raw
}jointParam_t;

typedef enum
{
    AlgsModeIdle = 1,
    AlgsModeJog,
    AlgsModePtp,
}AlgsMode_t;


extern jointParam_t jointParam[ROBOT_LEG_NUM][ROBOT_LEG_JOINT_NUM];

void jointParamInit(void);
void jointServoOutput(void);
void algsProfile(void);

#endif


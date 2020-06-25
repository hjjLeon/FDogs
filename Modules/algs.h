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
    float speedPesent;
    float zeroPosition;//in raw
    float angleMaxRaw;//in Raw
    float angleMinRaw;//in Raw
}jointParam_t;

typedef enum
{
    AlgsModeIdle = 1,
    AlgsModeJog,
    AlgsModePtp,
}AlgsMode_t;

typedef enum
{
    AlgsJogStop = 0,
    AlgsJogPos = 1,
    AlgsJogNeg,

    AlgsJogCmdMax,
}AlgsJogCmd_t;

extern AlgsMode_t gAlgsMode;
extern jointParam_t jointParam[ROBOT_LEG_NUM][ROBOT_LEG_JOINT_NUM];

void jointParamInit(void);
void jointServoOutput(void);
void algsProfile(void);

void AlgsPtpPredeal(void);
void AlgsJogPredeal(AlgsJogCmd_t cmd);

void hhtKeyCheck(void);


#endif


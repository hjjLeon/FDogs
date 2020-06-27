#ifndef __ALGS_H
#define __ALGS_H

#include "main.h"

#define ROBOT_LEG_NUM 4
#define ROBOT_LEG_JOINT_NUM 3

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
    AlgsModePowOff=0,
    AlgsModeIdle,
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

typedef enum
{
    LedRed = 1,
    LedGreed,
    LedBlue,
    LedYellow,
    LedWhite,
}LedColor_t;

typedef enum
{
    LedFlash = 1,
    LedNormal,
}LedMode_t;

extern AlgsMode_t gAlgsMode;
extern jointParam_t jointParam[ROBOT_LEG_NUM][ROBOT_LEG_JOINT_NUM];

void jointParamInit(void);
void jointServoOutput(void);
void algsProfile(void);

void AlgsPtpPredeal(void);
void AlgsJogPredeal(AlgsJogCmd_t cmd);

void hhtKeyCheck(void);
void ledStatusCheck(void);


#endif


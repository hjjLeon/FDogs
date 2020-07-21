#ifndef __ALGS_H
#define __ALGS_H

#include "main.h"

#define ROBOT_LEG_NUM 4
#define ROBOT_LEG_JOINT_NUM 3

//type define

typedef struct
{
    float x;
    float y;
    float z;
}WCSPosition;

typedef struct
{
    float theta1;
    float theta2;
    float theta3;
}JointTheta;

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

    float rate;
    float direction;
    uint32_t* PwmRegist;
}jointParam_t;

typedef struct
{
    jointParam_t param[ROBOT_LEG_NUM][ROBOT_LEG_JOINT_NUM];
    uint8_t StoreFlag;
}paramStore_t;


typedef struct
{
    WCSPosition positionOwn;
    WCSPosition positionPub;
    float speed;
}legParam_t;


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
    AlgsJogJointPos = 1,
    AlgsJogJointNeg,
    AlgsJogPosPos,
    AlgsJogPosNeg,

    AlgsJogCmdMax,
}AlgsJogCmd_t;


/*******LED********/
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
void jointServoOutput(uint8_t i);
void algsProfile(void);

void AlgsPtpPredeal(void);
void AlgsJogPredeal(AlgsJogCmd_t cmd, uint8_t leg, uint8_t index);

void hhtKeyCheck(void);
void ledStatusCheck(void);


#endif


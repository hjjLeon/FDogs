#include "algs.h"
#include "tim.h"
#include "gpio.h"

AlgsMode_t gAlgsMode = AlgsModePowOff;
jointParam_t jointParam[ROBOT_LEG_NUM][ROBOT_LEG_JOINT_NUM] = {0};
legParam_t   legParam[ROBOT_LEG_NUM] = {0};


#include "math.h"

#define LENTH_D 20
#define LENTH_L1 80
#define LENTH_L2 100
#define PI 3.1415926

int InverseCal(WCSPosition *pPosition, JointTheta *pTheta)
{
    float x, y, z, d, L1, L2;
    float ATp, AT, TC, AC, Theta_ATTp, Theta_TAC;
    float Theta1, Theta2, Theta3;

    d = LENTH_D;
    L1 = LENTH_L1;
    L2 = LENTH_L2;

    x = pPosition->x;
    y = pPosition->y;
    z = pPosition->z;


    ATp = sqrt(pow(z, 2) + pow(y, 2) - pow(d, 2));
    // printf("ATp = %f\r\n", ATp);

    
    Theta1 = atan(ATp / d) - atan(fabs(z) / y);

    AT = sqrt(pow(x, 2) + pow(ATp, 2));
    // printf("AT = %f\r\n", AT);

    Theta3 = acos((pow(L1, 2) + pow(L2, 2) - pow(AT, 2)) / (2 * L1 * L2));

    TC = L2 * sin(Theta3);
    // printf("TC = %f\r\n", TC);
    AC = L1 - L2 * cos(Theta3);
    // printf("AC = %f\r\n", AC);

    Theta_ATTp = atan(ATp / (x));
    Theta_TAC = atan(TC / AC);
    
    // printf("Theta_ATTp = %f\r\n", Theta_ATTp/PI*180.0);
    // printf("Theta_TAC = %f\r\n", Theta_TAC/PI*180.0);

    if(Theta_TAC < 0.0) 
        Theta_TAC += PI;

    if(x > 0)
    {
        if(z > 0)
        {
            Theta_ATTp = 2*PI - Theta_ATTp;
        }
        else
        {
            Theta_ATTp;
        }
    }
    else
    {
        if(z > 0)
        {
            Theta_ATTp = fabs(Theta_ATTp) + PI;
        }
        else
        {
            Theta_ATTp += PI;
        }
    }

    Theta2 = Theta_ATTp + Theta_TAC;

    pTheta->theta1 = fmod(Theta1/PI*180.0, 360.0);
    pTheta->theta2 = fmod(Theta2/PI*180.0, 360.0);
    pTheta->theta3 = fmod(Theta3/PI*180.0, 360.0);

    pTheta->theta2 -= 180.0;
    pTheta->theta3 -= 90.0 ;
}

void jointAngleUpdate(void);

void jointParamInit(void)
{
    jointParam[0][0].angleCurrent = 0;
    jointParam[0][0].PwmRegist = (uint32_t*)&(htim2.Instance->CCR1);
    jointParam[0][0].direction = 1;
    jointParam[0][0].rate = 1;
    jointParam[0][0].zeroPosition = 90.0;
    jointParam[0][0].speedPesent = 50.0;
    jointParam[0][0].angleMax = 30.0;
    jointParam[0][0].angleMin = -30.0;

    
    jointParam[0][1].angleCurrent = 0;
    jointParam[0][1].PwmRegist = (uint32_t*)&(htim2.Instance->CCR2);
    jointParam[0][1].direction = 1;
    jointParam[0][1].rate = 1;
    jointParam[0][1].zeroPosition = 90.0;
    jointParam[0][1].speedPesent = 100.0;
    jointParam[0][1].angleMax = 90.0;
    jointParam[0][1].angleMin = -90.0;

    jointParam[0][2].angleCurrent = 0;
    jointParam[0][2].PwmRegist = (uint32_t*)&(htim2.Instance->CCR4);
    jointParam[0][2].direction = -1;
    jointParam[0][2].rate = 20.0/16.0;
    jointParam[0][2].zeroPosition = 90.0;
    jointParam[0][2].speedPesent = 100.0;
    jointParam[0][2].angleMax = 55.0;
    jointParam[0][2].angleMin = -30.0;

    legParam[0].positionOwn.x = -80;
    legParam[0].positionOwn.y = 20;
    legParam[0].positionOwn.z = -100;
    legParam[0].speed = 50.0;

    jointAngleUpdate();

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void jointAngleUpdate(void)
{
    jointParam_t *pJ;

    for(uint8_t i = 0; i < ROBOT_LEG_NUM; i++)
    {
        for(uint8_t j = 0; j< ROBOT_LEG_JOINT_NUM; j++)
        {
            pJ = &jointParam[i][j];
            pJ->angleRaw = (pJ->angleCurrent * pJ->rate * pJ->direction + 
                            (pJ->direction>0?pJ->zeroPosition:(180.0 - pJ->zeroPosition)));
            pJ->periodTickCount = (uint32_t)(((pJ->angleRaw)*11.11111111111111*240.0+120000.0)/27.0) - 1;
        }
    }
}

void jointServoOutput(uint8_t i)
{
    jointParam_t *pJ;
    if(i >= ROBOT_LEG_NUM)
        return;

    for(uint8_t j = 0; j < ROBOT_LEG_JOINT_NUM; j++)
    {
        pJ = &jointParam[i][j];
        if(pJ->PwmRegist != NULL)
            *(pJ->PwmRegist) = pJ->periodTickCount;
    }
}

void hhtKeyCheck(void)
{
    #define SERVO_POWER_PORT GPIOA
    #define SERVO_POWER_PIN  GPIO_PIN_4
    #define SERVO_POWER_ON    1


    static uint8_t keyStatusLast = 0;

    if(keyStatusLast == 1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
    {
        //key press

        switch(gAlgsMode)
        {
            case AlgsModePowOff:
                gAlgsMode = AlgsModeIdle;
                HAL_GPIO_WritePin(SERVO_POWER_PORT, SERVO_POWER_PIN, SERVO_POWER_ON);
                break;
            default:
                gAlgsMode = AlgsModePowOff;
                HAL_GPIO_WritePin(SERVO_POWER_PORT, SERVO_POWER_PIN, !SERVO_POWER_ON);
                break;
        }

        keyStatusLast = 0;
    }
    else if(keyStatusLast == 0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1)
    {
        //key relase



        keyStatusLast = 1;
    }

}

void ledDisplay(LedColor_t color, LedMode_t mode)
{
    #define LED_ON 0
    #define LED_OFF 1

    #define LED_PORT GPIOC
    #define LED_PIN_RED GPIO_PIN_0
    #define LED_PIN_GREED GPIO_PIN_1
    #define LED_PIN_BLUE GPIO_PIN_2

    static uint8_t flashCount = 0;
    uint16_t ledPinOn, ledPinOff;

    flashCount++;
    flashCount%=2;

    switch(color)
    {
        case LedRed:
            ledPinOn  = LED_PIN_RED;
            ledPinOff = LED_PIN_GREED|LED_PIN_BLUE;
        break;
        case LedGreed:
            ledPinOn  = LED_PIN_GREED;
            ledPinOff = LED_PIN_RED|LED_PIN_BLUE;
        break;
        case LedBlue:
            ledPinOn  = LED_PIN_BLUE;
            ledPinOff = LED_PIN_GREED|LED_PIN_RED;
        break;
        case LedYellow:
            ledPinOn  = LED_PIN_GREED|LED_PIN_RED;
            ledPinOff = LED_PIN_BLUE;
        break;
    }


    switch (mode)
    {
        case LedFlash:
            HAL_GPIO_WritePin(LED_PORT, ledPinOn, flashCount);
            HAL_GPIO_WritePin(LED_PORT, ledPinOff, LED_OFF);
            break;
        case LedNormal:
            HAL_GPIO_WritePin(LED_PORT, ledPinOn, LED_ON);
            HAL_GPIO_WritePin(LED_PORT, ledPinOff, LED_OFF);
            break;
        default:
            break;
    }
}

void ledStatusCheck(void)
{
    LedColor_t color = LedGreed;
    LedMode_t mode = LedFlash;

    switch(gAlgsMode)
    {
        case AlgsModePowOff:
            color = LedRed;
            mode = LedNormal;
            break;
        case AlgsModeIdle:
            color = LedGreed;
            mode = LedFlash;
            break;
        case AlgsModeJog:
            color = LedGreed;
            mode = LedNormal;
            break;
        case AlgsModePtp:
            color = LedBlue;
            mode = LedFlash;
            break;
    }
    ledDisplay(color, mode);

}

static uint8_t sJogLeg, sJogIndex;
static AlgsJogCmd_t sJogCmd;
void AlgsJogPredeal(AlgsJogCmd_t cmd, uint8_t leg, uint8_t index)
{
    jointParam_t *pJ = NULL;
    legParam_t   *pL = NULL;

    #define DEG_PER_Ms_MAX (60.0/130.0)
    #define MM_PER_MS_MAX (0.20)

    switch(cmd)
    {
        case AlgsJogJointPos:
            pJ = &jointParam[leg][index];
            pJ->speed = 10.0*(DEG_PER_Ms_MAX)*(1.0/pJ->rate)*(pJ->speedPesent/100.0);//deg/10ms
        break;
        case AlgsJogJointNeg:
            pJ = &jointParam[leg][index];
            pJ->speed = -10.0*(DEG_PER_Ms_MAX)*(1.0/pJ->rate)*(pJ->speedPesent/100.0);//deg/10ms
        break;
        case AlgsJogPosPos:
            pL = &legParam[leg];
            pL->speed = 10.0*(MM_PER_MS_MAX)*(jointParam[0][0].speedPesent/100.0);//mm/10ms
        break;
        case AlgsJogPosNeg:
            pL = &legParam[leg];
            pL->speed = -10.0*(MM_PER_MS_MAX)*(jointParam[0][0].speedPesent/100.0);//mm/10ms
        break;
        default:
        break;
    }

    if(cmd == AlgsJogStop && gAlgsMode == AlgsModeJog)
    {
        gAlgsMode = AlgsModeIdle;
    }
    else if(cmd < AlgsJogCmdMax && gAlgsMode == AlgsModeIdle)
    {
        gAlgsMode = AlgsModeJog;
        sJogCmd = cmd;
        sJogLeg = leg;
        sJogIndex = index;
    }
}

void AlgsJog(void)
{
    if((jointParam[0][0].angleCurrent + jointParam[0][0].speed > jointParam[0][0].angleMax && jointParam[0][0].speed > 0) ||
    (jointParam[0][0].angleCurrent + jointParam[0][0].speed < jointParam[0][0].angleMin && jointParam[0][0].speed < 0))
    {
        gAlgsMode = AlgsModeIdle;
    }
    else
    {
        jointParam[0][0].angleCurrent += jointParam[0][0].speed;
    }

    jointParam_t *pJ = NULL;
    legParam_t   *pL = NULL;

    pJ = &jointParam[sJogLeg][sJogIndex];
    pL = &legParam[sJogLeg];

    switch(sJogCmd)
    {
        case AlgsJogJointPos:
        case AlgsJogJointNeg:
            pJ->angleCurrent += pJ->speed;
        break;
        case AlgsJogPosPos:
        case AlgsJogPosNeg:
            JointTheta  joint;
            *(&(pL->positionOwn.x) + sJogIndex) += pL->speed;
            InverseCal(&pL->positionOwn, &joint);
            jointParam[sJogLeg][0].angleCurrent = joint.theta1;
            jointParam[sJogLeg][1].angleCurrent = joint.theta2;
            jointParam[sJogLeg][2].angleCurrent = joint.theta3;
        break;
    }
}


void algsMoveJSlgPredeal(void)
{

}

void AlgsPtpPredeal(void)
{
    if(jointParam[0][0].angleTarget > jointParam[0][0].angleMax || 
    jointParam[0][0].angleTarget < jointParam[0][0].angleMin)
    {
        return;
    }

    jointParam[0][0].speed = 10.0*(60.0/130.0)*(16.0/20.0)*(jointParam[0][0].speedPesent/100.0);//deg/10ms

    if(jointParam[0][0].angleTarget < jointParam[0][0].angleCurrent)
    {
        jointParam[0][0].speed *= -1.0;
    }

    gAlgsMode = AlgsModePtp;
}

uint8_t AlgsPtp(void)
{
    if(jointParam[0][0].angleCurrent == jointParam[0][0].angleTarget)
    {
        return 1;
    }

    jointParam[0][0].angleCurrent += jointParam[0][0].speed;

    if((jointParam[0][0].speed < 0 && jointParam[0][0].angleCurrent < jointParam[0][0].angleTarget) || 
    (jointParam[0][0].speed > 0 && jointParam[0][0].angleCurrent > jointParam[0][0].angleTarget) )
    {
        jointParam[0][0].angleCurrent = jointParam[0][0].angleTarget;
    }

    return 0;
}

void algsAxisCheck(uint8_t leg)
{
    jointParam_t *pJ = NULL;

    for(uint8_t i; i < ROBOT_LEG_JOINT_NUM; i++)
    {
        pJ = &jointParam[leg][i];
        if(pJ->angleCurrent > pJ->angleMax)
        {
            pJ->angleCurrent = pJ->angleMax;
        }
        else if(pJ->angleCurrent < pJ->angleMin)
        {
            pJ->angleCurrent = pJ->angleMin;
        }
    }
}

void algsProfile()
{

    switch(gAlgsMode)
    {
        case AlgsModeIdle:
        break;
        case AlgsModeJog:
            AlgsJog();
        break;
        case AlgsModePtp:
            if(AlgsPtp() != 0)
            {
                gAlgsMode = AlgsModeIdle;
            }
        break;
    }

    for(uint8_t i = 0; i < ROBOT_LEG_NUM; i++)
    {
        algsAxisCheck(i);
    }
    jointAngleUpdate();
    
}




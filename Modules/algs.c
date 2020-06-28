#include "algs.h"
#include "tim.h"
#include "gpio.h"

AlgsMode_t gAlgsMode = AlgsModePowOff;
jointParam_t jointParam[ROBOT_LEG_NUM][ROBOT_LEG_JOINT_NUM] = {0};


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

    pTheta->theta1 = Theta1/PI*180.0;
    pTheta->theta2 = Theta2/PI*180.0;
    pTheta->theta3 = Theta3/PI*180.0;
}

void jointAngleUpdate(void);

void jointParamInit(void)
{
    jointParam[0][0].zeroPosition = 90.0;
    jointParam[0][0].angleCurrent = 0;
    jointParam[0][0].speedPesent = 100.0;
    jointParam[0][0].angleMax = 55.0;
    jointParam[0][0].angleMin = -30.0;
    jointParam[0][1].zeroPosition = 90.0;
    jointParam[0][1].angleCurrent = 0;
    jointParam[0][1].speedPesent = 100.0;
    jointParam[0][1].angleMax = 55.0;
    jointParam[0][1].angleMin = -30.0;
    jointAngleUpdate();

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void jointAngleUpdate(void)
{
    // for(uint8_t i; i < ROBOT_LEG_NUM; i++)
    // {
    //     for(uint8_t j; j < ROBOT_LEG_JOINT_NUM; j++)
    //     {

    //     }
    // }
    jointParam[0][0].angleRaw = (jointParam[0][0].angleCurrent/16.0*20.0+jointParam[0][0].zeroPosition);
    jointParam[0][0].periodTickCount = (uint32_t)(((jointParam[0][0].angleRaw)*11.11111111111111*240+120000)/27) - 1;
    jointParam[0][1].angleRaw = (jointParam[0][1].angleCurrent/16.0*20.0+jointParam[0][1].zeroPosition);
    jointParam[0][1].periodTickCount = (uint32_t)(((jointParam[0][0].angleRaw)*11.11111111111111*240+120000)/27) - 1;

}

void jointServoOutput(void)
{
    htim2.Instance->CCR1 = jointParam[0][0].periodTickCount;
    htim2.Instance->CCR2 = jointParam[0][1].periodTickCount;
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


void AlgsJogPredeal(AlgsJogCmd_t cmd)
{
    switch(cmd)
    {
        case AlgsJogPos:
            jointParam[0][0].speed = 10.0*(60.0/130.0)*(16.0/20.0)*(jointParam[0][0].speedPesent/100.0);//deg/10ms
        break;
        case AlgsJogNeg:
            jointParam[0][0].speed = -10.0*(60.0/130.0)*(16.0/20.0)*(jointParam[0][0].speedPesent/100.0);//deg/10ms
        break;
    }

    if(cmd == AlgsJogStop && gAlgsMode == AlgsModeJog)
    {
        gAlgsMode = AlgsModeIdle;
    }
    else if(cmd < AlgsJogCmdMax && gAlgsMode == AlgsModeIdle)
    {
        gAlgsMode = AlgsModeJog;
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

    jointAngleUpdate();
    
}




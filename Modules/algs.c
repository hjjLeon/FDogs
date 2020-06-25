#include "algs.h"
#include "tim.h"
#include "gpio.h"

AlgsMode_t gAlgsMode = AlgsModeIdle;
jointParam_t jointParam[ROBOT_LEG_NUM][ROBOT_LEG_JOINT_NUM] = {0};

void jointAngleUpdate(void);

void jointParamInit(void)
{
    jointParam[0][0].zeroPosition = 90.0;
    jointParam[0][0].angleCurrent = 0;
    jointParam[0][0].speedPesent = 100.0;
    jointParam[0][0].angleMax = 55.0;
    jointParam[0][0].angleMin = -30.0;
    jointAngleUpdate();

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
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

}

void jointServoOutput(void)
{
    htim2.Instance->CCR1 = jointParam[0][0].periodTickCount;
}

void hhtKeyCheck(void)
{
    static uint8_t keyStatusLast = 0;

    if(keyStatusLast == 1 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 0)
    {
        //key press

        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_3);

        keyStatusLast = 0;
    }
    else if(keyStatusLast == 0 && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == 1)
    {
        //key relase



        keyStatusLast = 1;
    }

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

    if(cmd == AlgsJogStop)
    {
        gAlgsMode = AlgsModeIdle;
    }
    else if(cmd < AlgsJogCmdMax)
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




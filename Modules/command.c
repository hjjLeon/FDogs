#include "cmsis_os2.h"
#include "command.h"
#include "usbd_cdc_if.h"
#include "algs.h"


typedef struct
{
    uint8_t id;
    void (*command)(uint8_t rw, void* param, uint8_t* paramLenth);
}CommList_t;



extern osMessageQueueId_t portRawDataTxHandle;
extern osMessageQueueId_t portRawDataRxHandle;
extern osMessageQueueId_t cmdPacketTxHandle;
extern osMessageQueueId_t cmdPacketRxHandle;
extern osMessageQueueId_t cmdPacketRxInstantHandle;

static uint8_t testValue = 0;
void testCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    if(rw)
    {
        if(*paramLenth == 1)
            testValue = ((uint8_t*)param)[0];
        *paramLenth = 0;
    }
    else
    {
        ((uint8_t*)param)[0] = testValue;
        *paramLenth = 1;
    }
}

void speedParamCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    float temp;
    uint8_t index;
    if(rw)
    {
        if(*paramLenth == 5)
        {
            index = ((uint8_t*)param)[0];
            memcpy(&temp, ((uint8_t*)param)+1, 4);
            if(temp > 100)
            {
                temp = 100;
            }
            else if(temp < 0)
            {
                temp = 0;
            }

            jointParam_t* pjp = (&jointParam[0][0]) + index;
            (*pjp).speedPesent = temp;
        }
        
        *paramLenth = 0;
    }
    else
    {
        *paramLenth = 0;
    }
}

void posParamCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    float temp;
    uint8_t index;
    if(rw)
    {
        *paramLenth = 0;
    }
    else
    {
        if(*paramLenth == 1)
        {
            index = ((uint8_t*)param)[0];

            
            jointParam_t* pjp = (&jointParam[0][0]) + index;

            temp = (*pjp).angleCurrent;
            memcpy(((uint8_t*)param)+1, &temp, 4);
        }

        *paramLenth = 5;
    }
}

void setZeroCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    float temp;
    uint8_t index;
    if(rw)
    {
        if(*paramLenth == 1 && gAlgsMode == AlgsModeIdle)
        {
            index = ((uint8_t*)param)[0];
            jointParam_t* pjp = (&jointParam[0][0]) + index;

            (*pjp).zeroPosition = (*pjp).angleRaw;
            (*pjp).angleCurrent = 0.0;
        }
        
        *paramLenth = 0;
    }
    else
    {
        *paramLenth = 0;
    }
}

void setMaxCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    float temp;
    uint8_t index;
    if(rw)
    {
        if(*paramLenth == 1 && gAlgsMode == AlgsModeIdle)
        {
            index = ((uint8_t*)param)[0];
            jointParam_t* pjp = (&jointParam[0][0]) + index;
            (*pjp).angleMax = (*pjp).angleCurrent;
        }
        
        *paramLenth = 0;
    }
    else
    {
        *paramLenth = 0;
    }
}

void setMinCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    float temp;
    uint8_t index;
    if(rw)
    {
        if(*paramLenth == 1 && gAlgsMode == AlgsModeIdle)
        {
            index = ((uint8_t*)param)[0];
            jointParam_t* pjp = (&jointParam[0][0]) + index;
            (*pjp).angleMin = (*pjp).angleCurrent;
        }
        
        *paramLenth = 0;
    }
    else
    {
        *paramLenth = 0;
    }
}

void jogCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    uint8_t jogcmd, leg, index;
    if(rw)
    {
        if(*paramLenth == 3)
        {
            jogcmd = ((uint8_t*)param)[0];
            leg = ((uint8_t*)param)[1];
            index = ((uint8_t*)param)[2];

            if(jogcmd < AlgsJogCmdMax)
                AlgsJogPredeal(jogcmd, leg, index);
        }
        
        *paramLenth = 0;
    }
    else
    {
        *paramLenth = 0;
    }
}

void goCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    float temp;
    uint8_t index;
    if(rw)
    {
        if(*paramLenth == 5)
        {
            index = ((uint8_t*)param)[0];
            memcpy(&temp, ((uint8_t*)param)+1, 4);
            if(temp > 90)
            {
                temp = 90;
            }
            else if(temp < -90)
            {
                temp = -90;
            }

            jointParam_t* pjp = (&jointParam[0][0]) + index;
            (*pjp).angleTarget = temp;
            AlgsPtpPredeal();
        }
        
        *paramLenth = 0;
    }
    else
    {
        *paramLenth = 0;
    }
}


void moveLSlgCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{

}

void moveJSlgCmdFunc(uint8_t rw, void* param, uint8_t* paramLenth)
{
    if(rw)
    {
        //if(*paramLenth == )

        *paramLenth = 0;
    }
    else
    {
        *paramLenth = 0;
    }
    
}

CommList_t commList[] = {
    {0, testCmdFunc},

    {10, speedParamCmdFunc},
    {11, posParamCmdFunc},
    {12, setZeroCmdFunc},
    {13, setMaxCmdFunc},
    {14, setMinCmdFunc},

    {30, jogCmdFunc},

    {40, goCmdFunc},
    {41, moveLSlgCmdFunc},
    {42, moveJSlgCmdFunc},

    {255, NULL},
};

void protocolRawDataSend(void)
{
    osStatus_t sta;
    uint8_t rawBuff[64];
    uint16_t dataCount;

    dataCount = osMessageQueueGetCount(portRawDataTxHandle);
    if(dataCount != 0)
    {
        while(dataCount > 64)
        {
            for(uint8_t i = 0; i < 64; i++)
            {
                sta = osMessageQueueGet(portRawDataTxHandle, rawBuff+i, 0, 0);
            }
            while(CDC_Transmit_FS(rawBuff, 64) != 0)
                osDelay(5);

            dataCount -= 64;
        }

        for(uint8_t i = 0; i < dataCount; i++)
        {
            sta = osMessageQueueGet(portRawDataTxHandle, rawBuff+i, 0, 0);
        }
        while(CDC_Transmit_FS(rawBuff, dataCount) != 0)
            osDelay(5);
    }
}

void ProtocolWritePacket(void)
{
    osStatus_t sta;
    uint8_t rawBuff[64];
    commandPacket_t packet;
    uint16_t waitTimeOutCount = 0;
    while(osMessageQueueGetCount(cmdPacketTxHandle) != 0 && 
        osMessageQueueGetSpace(portRawDataTxHandle) >= 7)
    {
        sta = osMessageQueueGet(cmdPacketTxHandle, &packet, NULL, 0);
        while(osMessageQueueGetSpace(portRawDataTxHandle) < 2+1+2+packet.lenth+2)
        {
            osDelay(5);
            waitTimeOutCount++;
            if(waitTimeOutCount >= 200*1)
            {
                break;
            }
        }
        if(osMessageQueueGetSpace(portRawDataTxHandle) < 2+1+2+packet.lenth+2)
            break;
        
        rawBuff[0] = 0xAB;
        rawBuff[1] = 0xCD;
        rawBuff[2] = packet.lenth;
        rawBuff[3] = packet.id;
        rawBuff[4] = packet.ctrl.u8;
        if(packet.lenth)
            memcpy(rawBuff+5, packet.param, packet.lenth);
        rawBuff[5+packet.lenth] = 0;//CRC
        rawBuff[6+packet.lenth] = 0;//CRC

        for(uint8_t i = 0; i < 2+1+2+packet.lenth+2; i++)
        {
            osMessageQueuePut(portRawDataTxHandle, rawBuff+i, 0, 0);
        }
    }
}

void ProtocolReadPacket(void)
{
    uint8_t rawData[70];
    uint8_t temp = 0;
    osStatus_t sta;
    uint8_t lenth = 0, id = 0, ctrl = 0;
    uint16_t crcData, crcCalculate;
    uint16_t waitTimeOutCount = 0;
    commandPacket_t packet;

    while(osMessageQueueGetCount(portRawDataRxHandle) >= 7)
    {
        //absorting data until 0xAB
        sta = osMessageQueueGet(portRawDataRxHandle, &temp, NULL, 0);
        while(sta == osOK && temp != 0xAB)
        {
            sta = osMessageQueueGet(portRawDataRxHandle, &temp, NULL, 0);
        }
        if(sta != osOK)
            break;

        //wait untile have enoug data
        waitTimeOutCount = 0;
        while(osMessageQueueGetCount(portRawDataRxHandle) < 6)
        {
            osDelay(5);
            waitTimeOutCount++;
            if(waitTimeOutCount >= 200*1)
            {
                break;
            }
        }
        if(osMessageQueueGetCount(portRawDataRxHandle) < 6)
            break;

        //cant find 0xAB 0xCD, try again
        sta = osMessageQueueGet(portRawDataRxHandle, &temp, NULL, 0);
        if(sta != osOK || temp != 0xCD)
            continue;

        //check lenth is valid
        sta = osMessageQueueGet(portRawDataRxHandle, &lenth, NULL, 0);//Lenth
        if(sta != osOK)
            break;
        else if(lenth > 64)
            continue;

        //wait until data compelet
        waitTimeOutCount = 0;
        while(osMessageQueueGetCount(portRawDataRxHandle) < 2 + lenth + 2)//id+ctrl + data + crc16
        {
            osDelay(5);
            waitTimeOutCount++;
            if(waitTimeOutCount >= 200*1)
            {
                break;
            }
        }
        if(osMessageQueueGetCount(portRawDataRxHandle) < 2 + lenth + 2)
            continue;

        //get ID & Ctrl & data
        osMessageQueueGet(portRawDataRxHandle, &id, NULL, 0);//ID
        osMessageQueueGet(portRawDataRxHandle, &ctrl, NULL, 0);//Ctrl
        for(uint8_t i = 0; i < lenth; i++)
        {
            osMessageQueueGet(portRawDataRxHandle, rawData+i, NULL, 0);//DATA
        }

        //get CRC data
        crcData = 0;
        osMessageQueueGet(portRawDataRxHandle, &temp, NULL, 0);
        crcData += temp;
        osMessageQueueGet(portRawDataRxHandle, &temp, NULL, 0);
        crcData += temp<<8;

        //check CRC
        //TODO

        //put in packet
        packet.id = id;
        packet.ctrl.u8 = ctrl;
        packet.lenth = lenth;
        memcpy(packet.param, rawData, lenth);
        if(packet.ctrl.bit.isQueue)
            sta = osMessageQueuePut(cmdPacketRxHandle, &packet, 0, 100);
        else
            sta = osMessageQueuePut(cmdPacketRxInstantHandle, &packet, 0, 100);
    }
}

void commandExc(void)
{
    osStatus_t sta;
    commandPacket_t packet;
    while(osMessageQueueGetCount(cmdPacketRxInstantHandle) != 0)
    {
        sta = osMessageQueueGet(cmdPacketRxInstantHandle, &packet, NULL, 0);
        for(uint8_t i = 0; i < 255; i++)
        {
            if(commList[i].id == packet.id)
            {
                commList[i].command(
                    packet.ctrl.bit.rw,
                    (void*)packet.param,
                    &packet.lenth);
                if(packet.lenth)
                    osMessageQueuePut(cmdPacketTxHandle, &packet, 0, 0);

                break;
            }
            else if(commList[i].id == 255)
            {
                break;
            }
        }
    }
    if(gAlgsMode == AlgsModeIdle)
    {
        if(osMessageQueueGetCount(cmdPacketRxHandle) != 0)
        {
            sta = osMessageQueueGet(cmdPacketRxHandle, &packet, NULL, 0);
            for(uint8_t i = 0; i < 255; i++)
            {
                if(commList[i].id == packet.id)
                {
                    commList[i].command(
                        packet.ctrl.bit.rw,
                        (void*)packet.param,
                        &packet.lenth);
                    if(packet.lenth)
                        osMessageQueuePut(cmdPacketTxHandle, &packet, 0, 0);

                    break;
                }
                else if(commList[i].id == 255)
                {
                    break;
                }
            }
        }
    }
}




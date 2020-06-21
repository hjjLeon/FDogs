#include "cmsis_os2.h"
#include "command.h"


extern osMessageQueueId_t portRawDataQueueRxHandle;
extern osMessageQueueId_t cmdPacketRxHandle;


void dealWithProtocol(void)
{
    uint8_t rawData[70];
    uint8_t temp = 0;
    osStatus_t sta;
    uint8_t lenth = 0, id = 0, ctrl = 0;
    uint16_t crcData, crcCalculate;
    uint16_t waitTimeOutCount = 0;
    commandPacket_t packet;

    while(osMessageQueueGetCount(portRawDataQueueRxHandle) >= 7)
    {
        //absorting data until 0xAB
        sta = osMessageQueueGet(portRawDataQueueRxHandle, &temp, NULL, 0);
        while(sta == osOK && temp != 0xAB)
        {
            sta = osMessageQueueGet(portRawDataQueueRxHandle, &temp, NULL, 0);
        }
        if(sta != osOK)
            break;

        //wait untile have enoug data
        waitTimeOutCount = 0;
        while(osMessageQueueGetCount(portRawDataQueueRxHandle) < 6)
        {
            osDelay(5);
            waitTimeOutCount++;
            if(waitTimeOutCount >= 200*1)
            {
                break;
            }
        }
        if(osMessageQueueGetCount(portRawDataQueueRxHandle) < 6)
            break;

        //cant find 0xAB 0xCD, try again
        sta = osMessageQueueGet(portRawDataQueueRxHandle, &temp, NULL, 0);
        if(sta != osOK || temp != 0xCD)
            continue;

        //check lenth is valid
        sta = osMessageQueueGet(portRawDataQueueRxHandle, &lenth, NULL, 0);//Lenth
        if(sta != osOK)
            break;
        else if(lenth > 64)
            continue;

        //wait until data compelet
        waitTimeOutCount = 0;
        while(osMessageQueueGetCount(portRawDataQueueRxHandle) < 2 + lenth + 2)//id+ctrl + data + crc16
        {
            osDelay(5);
            waitTimeOutCount++;
            if(waitTimeOutCount >= 200*1)
            {
                break;
            }
        }
        if(osMessageQueueGetCount(portRawDataQueueRxHandle) < 2 + lenth + 2)
            continue;

        //get ID & Ctrl & data
        osMessageQueueGet(portRawDataQueueRxHandle, &id, NULL, 0);//ID
        osMessageQueueGet(portRawDataQueueRxHandle, &ctrl, NULL, 0);//Ctrl
        for(uint8_t i = 0; i < lenth; i++)
        {
            osMessageQueueGet(portRawDataQueueRxHandle, rawData+i, NULL, 0);//DATA
        }

        //get CRC data
        crcData = 0;
        osMessageQueueGet(portRawDataQueueRxHandle, &temp, NULL, 0);
        crcData += temp;
        osMessageQueueGet(portRawDataQueueRxHandle, &temp, NULL, 0);
        crcData += temp<<8;

        //check CRC
        //TODO

        //put in packet
        packet.id = id;
        packet.ctrl.u8 = ctrl;
        packet.lenth = lenth;
        memcpy(packet.param, rawData, lenth);
        sta = osMessageQueuePut(cmdPacketRxHandle, &packet, 0, 100);
    }
}




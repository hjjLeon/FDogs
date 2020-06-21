#ifndef __COMMAND_H
#define __COMMAND_H

#include "main.h"

typedef struct 
{
    uint8_t id;
    union command
    {
        struct
        {
            uint8_t rw:1;
            uint8_t isQueue:1;
            uint8_t rsv:6;
        }bit;
        uint8_t u8;
    }ctrl;
    uint8_t lenth;
    uint8_t param[62];
}commandPacket_t;


void dealWithProtocol(void);


#endif


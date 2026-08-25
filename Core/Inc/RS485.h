#ifndef __RS485_H
#define __RS485_H

#include "main.h"

#define RS485_FRAME_START                   0x5A
#define RS485_FRAME_TYPE                    0xFD
#define RS485_FRAME_CMD_OFF                 0x00
#define RS485_FRAME_CMD_ON                  0x01
#define RS485_FRAME_CMD_SET_CONTROL_MODE    0x02
#define RS485_FRAME_CMD_GET_CONTROL_MODE    0x03
#define RS485_FRAME_CMD_SET_POWER_LEVEL     0x04
#define RS485_FRAME_CMD_GET_POWER_LEVEL     0x05
#define RS485_FRAME_CMD_GET_DEVICE_INFO     0x10
#define RS485_FRAME_CMD_GET_SYS_STATE       0x23
#define RS485_FRAME_CMD_FIRMWARE_UPGRADE    0x45

#define RS485_FRAME_INDEX_START             0
#define RS485_FRAME_INDEX_TYPE              1
#define RS485_FRAME_INDEX_CMD               2
#define RS485_FRAME_INDEX_DATASIZE          3

#define RS485_FRAME_OVERHEAD_LEN            5
#define RS485_FRAME_HEAD_LEN                4
#define RS485_FRAME_MIN_LEN                 5

typedef struct {
    uint8_t cmd;
    uint8_t data[3];
} RS485_QueueMsg;

void RS485_Receive_To_Transmit(uint16_t GPIO_Pin);
void RS485_Transmit_To_Receive(uint16_t GPIO_Pin);
uint8_t RS485_CheckFrameSum(const uint8_t *frame, uint16_t len);
bool RS485_ProcessFrame(const uint8_t *frame, uint16_t len, RS485_QueueMsg *msg);
bool RS485_TransmitFrame(uint8_t *frame, uint16_t len);

#endif

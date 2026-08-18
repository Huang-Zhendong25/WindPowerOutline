#include "RS485.h"
#include <string.h>

void RS485_Transmit_Enable(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RS485_EN1_Pin)
        HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_SET);
    else if (GPIO_Pin == RS485_EN2_Pin)
        HAL_GPIO_WritePin(RS485_EN2_GPIO_Port, RS485_EN2_Pin, GPIO_PIN_SET);
}

void RS485_Transmit_Disable(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RS485_EN1_Pin)
        HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_RESET);
    else if (GPIO_Pin == RS485_EN2_Pin)
        HAL_GPIO_WritePin(RS485_EN2_GPIO_Port, RS485_EN2_Pin, GPIO_PIN_RESET);
}

static uint8_t RS485_CheckFrameSum(const uint8_t *frame, uint16_t len)
{
    uint8_t rs485_frame_sum = 0;
    for (uint8_t i = 0; i < len - 1; i++)
    {
        rs485_frame_sum += frame[i];
    }
    return rs485_frame_sum;
}

uint8_t RS485_ProcessFrame(const uint8_t *frame, uint16_t len, RS485_QueueMsg *msg)
{
    // Implementation for processing RS485 frame
    if (len < RS485_FRAME_MIN_LEN)
        return HAL_ERROR;
    if (frame[RS485_FRAME_INDEX_START] != RS485_FRAME_START || frame[RS485_FRAME_INDEX_TYPE] != RS485_FRAME_TYPE || (frame[len - 1] != RS485_CheckFrameSum(frame, len)))
    {
        return HAL_ERROR;
    }
    msg->cmd = frame[RS485_FRAME_INDEX_CMD];
    if (frame[RS485_FRAME_INDEX_DATASIZE] == (len - RS485_FRAME_OVERHEAD_LEN))
    {
        memcpy(msg->data, frame + RS485_FRAME_HEAD_LEN, frame[RS485_FRAME_INDEX_DATASIZE]);
    }
    return HAL_OK;
}


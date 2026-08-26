#include "RS485.h"
#include <string.h>

void RS485_Init(void)
{
    HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RS485_EN2_GPIO_Port, RS485_EN2_Pin, GPIO_PIN_RESET);
}

void RS485_Receive_To_Transmit(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RS485_EN1_Pin)
        HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_SET);
    else if (GPIO_Pin == RS485_EN2_Pin)
        HAL_GPIO_WritePin(RS485_EN2_GPIO_Port, RS485_EN2_Pin, GPIO_PIN_SET);
}

void RS485_Transmit_To_Receive(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RS485_EN1_Pin)
        HAL_GPIO_WritePin(RS485_EN1_GPIO_Port, RS485_EN1_Pin, GPIO_PIN_RESET);
    else if (GPIO_Pin == RS485_EN2_Pin)
        HAL_GPIO_WritePin(RS485_EN2_GPIO_Port, RS485_EN2_Pin, GPIO_PIN_RESET);
}

uint8_t RS485_CheckFrameSum(const uint8_t *frame, uint16_t len)
{
    uint8_t rs485_frame_sum = 0;
    for (uint8_t i = 0; i < len - 1; i++)
    {
        rs485_frame_sum += frame[i];
    }
    return rs485_frame_sum;
}

bool RS485_ProcessFrame(const uint8_t *frame, uint16_t len, RS485_QueueMsg *msg)
{
    // Implementation for processing RS485 frame
    if (len < RS485_FRAME_MIN_LEN)
        return false;
    if (frame[RS485_FRAME_INDEX_START] != RS485_FRAME_START || frame[RS485_FRAME_INDEX_TYPE] != RS485_FRAME_TYPE || (frame[len - 1] != RS485_CheckFrameSum(frame, len)))
    {
        return false;
    }
    msg->cmd = frame[RS485_FRAME_INDEX_CMD];
    if (frame[RS485_FRAME_INDEX_DATASIZE] == (len - RS485_FRAME_OVERHEAD_LEN))
    {
        memcpy(msg->data, frame + RS485_FRAME_HEAD_LEN, frame[RS485_FRAME_INDEX_DATASIZE]);
        //msg->data[0] = frame[4];
    }
    return true;
}

bool RS485_TransmitFrame(uint8_t *frame, uint16_t len)
{
    if (HAL_UART_Transmit(&huart2, frame, len, HAL_MAX_DELAY) == HAL_OK)
    {
        return true;
    }
    return false;
}

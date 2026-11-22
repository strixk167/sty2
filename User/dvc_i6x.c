#include "dvc_i6x.h"
#include "drv_uart.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

/* 默认链接到 AXI SRAM；不使用临时栈数组作为持续 DMA 接收缓冲。 */
static uint8_t I6X_RxBuffer[I6X_DMA_RX_BYTES];
static I6X_Receiver *I6X_UARTReceiver;

/** @brief 检查单个配置通道编号。 */
static bool I6X_ChannelValid(uint32_t channel)
{
    return channel >= 1U && channel <= I6X_CHANNEL_COUNT;
}

/** @brief 将摇杆原始值映射到 -1～1，死区以外重新缩放，保留两端满量程。 */
static float I6X_Normalize(uint16_t raw)
{
    int32_t delta = (int32_t)raw - (int32_t)I6X_RAW_CENTER;
    float result;
    if (delta > (int32_t)I6X_DEADZONE_RAW)
    {
        result = (float)(delta - (int32_t)I6X_DEADZONE_RAW) /
                 (float)(I6X_RAW_MAX - I6X_RAW_CENTER - I6X_DEADZONE_RAW);
    }
    else if (delta < -(int32_t)I6X_DEADZONE_RAW)
    {
        result = (float)(delta + (int32_t)I6X_DEADZONE_RAW) /
                 (float)(I6X_RAW_CENTER - I6X_RAW_MIN - I6X_DEADZONE_RAW);
    }
    else
    {
        return 0.0f;
    }
    if (result > 1.0f) return 1.0f;
    if (result < -1.0f) return -1.0f;
    return result;
}

bool I6X_Init(I6X_Receiver *receiver)
{
    if (receiver == NULL) return false;
    memset(receiver, 0, sizeof(*receiver));
    if (I6X_RAW_MIN >= I6X_RAW_CENTER || I6X_RAW_CENTER >= I6X_RAW_MAX ||
        I6X_RAW_MAX > 2047U ||
        I6X_DEADZONE_RAW >= I6X_RAW_CENTER - I6X_RAW_MIN ||
        I6X_DEADZONE_RAW >= I6X_RAW_MAX - I6X_RAW_CENTER ||
        I6X_TIMEOUT_MS == 0U || I6X_TIMEOUT_MS >= 0x80000000UL ||
        I6X_SWITCH_LOW >= I6X_SWITCH_HIGH || I6X_SWITCH_HIGH > 2047U ||
        !isfinite(I6X_ARM_CENTER_LIMIT) || I6X_ARM_CENTER_LIMIT < 0.0f ||
        I6X_ARM_CENTER_LIMIT >= 1.0f ||
        !I6X_ChannelValid(I6X_CHANNEL_VX) || !I6X_ChannelValid(I6X_CHANNEL_VY) ||
        !I6X_ChannelValid(I6X_CHANNEL_WZ) || !I6X_ChannelValid(I6X_CHANNEL_ARM) ||
        !I6X_ChannelValid(I6X_CHANNEL_SPIN) ||
        (I6X_VX_SIGN != 1 && I6X_VX_SIGN != -1) ||
        (I6X_VY_SIGN != 1 && I6X_VY_SIGN != -1) ||
        (I6X_WZ_SIGN != 1 && I6X_WZ_SIGN != -1))
    {
        return false;
    }
    receiver->initialized = true;
    return true;
}

void I6X_NotifyError(I6X_Receiver *receiver)
{
    if (receiver != NULL && receiver->initialized)
    {
        receiver->used = 0U;
        ++receiver->snapshot.fault_count;
        receiver->snapshot.failsafe = true;
    }
}

/** @brief 解包一个已验证头尾的 25 字节帧；16 个通道每个占连续 11 位。 */
static void I6X_Decode(I6X_Receiver *receiver, uint32_t now_ms)
{
    uint32_t channel;
    uint32_t bit;
    I6X_Snapshot *snapshot = &receiver->snapshot;
    const uint8_t *frame = receiver->frame;
    if (snapshot->received &&
        (uint32_t)(now_ms - snapshot->last_frame_ms) > I6X_TIMEOUT_MS)
    {
        ++snapshot->fault_count;
    }
    for (channel = 0U; channel < I6X_CHANNEL_COUNT; ++channel)
    {
        uint16_t value = 0U;
        for (bit = 0U; bit < 11U; ++bit)
        {
            uint32_t offset = channel * 11U + bit;
            value |= (uint16_t)(((frame[1U + offset / 8U] >>
                                 (offset % 8U)) & 1U) << bit);
        }
        snapshot->raw[channel] = value;
    }
    snapshot->digital17 = (frame[23] & 0x01U) != 0U;
    snapshot->digital18 = (frame[23] & 0x02U) != 0U;
    snapshot->frame_lost = (frame[23] & 0x04U) != 0U;
    snapshot->failsafe = (frame[23] & 0x08U) != 0U;
    if (snapshot->frame_lost || snapshot->failsafe)
    {
        ++snapshot->fault_count;
    }
    snapshot->last_frame_ms = now_ms;
    ++snapshot->frame_count;
    snapshot->received = true;
}

void I6X_Feed(I6X_Receiver *receiver, const uint8_t *data,
              uint16_t length, uint32_t now_ms)
{
    uint16_t index;
    if (receiver == NULL || !receiver->initialized || data == NULL) return;
    /* 跨越失联时间的半帧不能和恢复后的数据拼接。 */
    if (receiver->snapshot.received &&
        (uint32_t)(now_ms - receiver->snapshot.last_frame_ms) > I6X_TIMEOUT_MS)
    {
        receiver->used = 0U;
    }
    for (index = 0U; index < length; ++index)
    {
        if (receiver->used == 0U && data[index] != 0x0FU) continue;
        receiver->frame[receiver->used++] = data[index];
        if (receiver->used == I6X_FRAME_BYTES)
        {
            /* 本驱动按所给 iA6B 文档的标准 SBUS：0F ... flags 00。 */
            if (receiver->frame[0] == 0x0FU &&
                receiver->frame[24] == 0x00U &&
                (receiver->frame[23] & 0xF0U) == 0U)
            {
                I6X_Decode(receiver, now_ms);
                receiver->used = 0U;
            }
            else
            {
                uint8_t next;
                ++receiver->snapshot.fault_count;
                /* 从候选帧中查找下一个头，避免一次错位后永久错位。 */
                for (next = 1U; next < I6X_FRAME_BYTES; ++next)
                {
                    if (receiver->frame[next] == 0x0FU) break;
                }
                receiver->used = (uint8_t)(I6X_FRAME_BYTES - next);
                memmove(receiver->frame, receiver->frame + next, receiver->used);
            }
        }
    }
}

bool I6X_GetSnapshot(const I6X_Receiver *receiver, I6X_Snapshot *snapshot)
{
    uint32_t primask;
    if (receiver == NULL || snapshot == NULL || !receiver->initialized) return false;
    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = receiver->snapshot;
    __set_PRIMASK(primask);
    return snapshot->received;
}

bool I6X_GetControl(I6X_Receiver *receiver, uint32_t now_ms,
                    I6X_Control *control)
{
    I6X_Snapshot snapshot;
    bool fault;
    bool high;
    float vx, vy, wz;
    if (control == NULL) return false;
    memset(control, 0, sizeof(*control));
    if (receiver == NULL || !I6X_GetSnapshot(receiver, &snapshot)) return false;
    fault = snapshot.fault_count != receiver->observed_faults;
    receiver->observed_faults = snapshot.fault_count;
    if (fault || snapshot.frame_lost || snapshot.failsafe ||
        (uint32_t)(now_ms - snapshot.last_frame_ms) > I6X_TIMEOUT_MS)
    {
        receiver->armed = false;
        receiver->off_seen = false;
        receiver->previous_arm_high = false;
        receiver->observed_frames = snapshot.frame_count;
        return false;
    }
    control->online = true;
    vx = I6X_Normalize(snapshot.raw[I6X_CHANNEL_VX - 1U]) * I6X_VX_SIGN;
    vy = I6X_Normalize(snapshot.raw[I6X_CHANNEL_VY - 1U]) * I6X_VY_SIGN;
    wz = I6X_Normalize(snapshot.raw[I6X_CHANNEL_WZ - 1U]) * I6X_WZ_SIGN;
    high = snapshot.raw[I6X_CHANNEL_ARM - 1U] >= I6X_SWITCH_HIGH;
    /* 同一帧被重复轮询不能产生新的开关动作。 */
    if (snapshot.frame_count != receiver->observed_frames)
    {
        receiver->observed_frames = snapshot.frame_count;
        if (!high)
        {
            receiver->armed = false;
            receiver->off_seen =
                snapshot.raw[I6X_CHANNEL_ARM - 1U] <= I6X_SWITCH_LOW;
        }
        else if (!receiver->previous_arm_high && receiver->off_seen &&
                 fabsf(vx) <= I6X_ARM_CENTER_LIMIT &&
                 fabsf(vy) <= I6X_ARM_CENTER_LIMIT &&
                 fabsf(wz) <= I6X_ARM_CENTER_LIMIT)
        {
            receiver->armed = true;
        }
        receiver->previous_arm_high = high;
    }
    control->armed = receiver->armed;
    if (control->armed)
    {
        control->vx = vx;
        control->vy = vy;
        control->wz = wz;
        control->spin = snapshot.raw[I6X_CHANNEL_SPIN - 1U] >= I6X_SWITCH_HIGH;
    }
    return true;
}

/** @brief UART5 数据分块适配，不把中断内临时数据地址保存到对象中。 */
static void I6X_UARTReceive(uint8_t *data, uint16_t length)
{
    I6X_Feed(I6X_UARTReceiver, data, length, HAL_GetTick());
}

/** @brief 错误帧即使紧接着恢复，也要通知控制端撤销使能。 */
static void I6X_UARTError(uint32_t error)
{
    (void)error;
    I6X_NotifyError(I6X_UARTReceiver);
}

HAL_StatusTypeDef I6X_AttachUART5(I6X_Receiver *receiver,
                                 UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;
    if (receiver == NULL || !receiver->initialized || huart == NULL ||
        huart->Instance != UART5 || huart->Init.BaudRate != 100000U ||
        huart->Init.WordLength != UART_WORDLENGTH_9B ||
        huart->Init.Parity != UART_PARITY_EVEN ||
        huart->Init.StopBits != UART_STOPBITS_2 ||
        (huart->Init.Mode & UART_MODE_RX) == 0U ||
        (huart->Instance->CR2 & USART_CR2_RXINV) != 0U)
    {
        return HAL_ERROR;
    }
    if (I6X_UARTReceiver != NULL) return HAL_BUSY;
    /* 先建立回调目标，再启动 DMA。正常情况只在启动阶段调用一次。 */
    I6X_UARTReceiver = receiver;
    status = UART_Set_Error_Callback(huart, I6X_UARTError);
    if (status == HAL_OK)
    {
        status = UART_Init(huart, I6X_RxBuffer, sizeof(I6X_RxBuffer), I6X_UARTReceive);
    }
    if (status != HAL_OK)
    {
        (void)UART_Set_Error_Callback(huart, NULL);
        I6X_UARTReceiver = NULL;
    }
    return status;
}

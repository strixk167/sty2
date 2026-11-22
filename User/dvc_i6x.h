#ifndef DVC_I6X_H
#define DVC_I6X_H

/*================ 用户配置：通道编号从 1 开始 ================*/
#define I6X_TIMEOUT_MS             100U
#define I6X_RAW_MIN                240U
#define I6X_RAW_CENTER             1024U
#define I6X_RAW_MAX                1807U
#define I6X_DEADZONE_RAW           35U
#define I6X_ARM_CENTER_LIMIT       0.08f
#define I6X_SWITCH_LOW             700U
#define I6X_SWITCH_HIGH            1300U
#define I6X_CHANNEL_VX             2U
#define I6X_CHANNEL_VY             1U
#define I6X_CHANNEL_WZ             4U
#define I6X_CHANNEL_ARM            7U  /* SwA：低值停止，高值使能。 */
#define I6X_CHANNEL_SPIN           9U  /* SwC：高值小陀螺，其余普通模式。 */
#define I6X_VX_SIGN                (+1)
#define I6X_VY_SIGN                (-1)
#define I6X_WZ_SIGN                (-1)

#define I6X_CHANNEL_COUNT          16U
#define I6X_FRAME_BYTES            25U
#define I6X_DMA_RX_BYTES           128U

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/** @brief 接收快照。raw[] 下标从零开始，对应遥控器 CH1～CH16。 */
typedef struct
{
    uint16_t raw[I6X_CHANNEL_COUNT];
    bool digital17;
    bool digital18;
    bool frame_lost;
    bool failsafe;
    bool received;
    uint32_t last_frame_ms;
    uint32_t frame_count;
    uint32_t fault_count; /**< 异常累计；即使随后恢复正常帧，也能通知控制端撤销使能。 */
} I6X_Snapshot;

/** @brief 独立接收对象。Feed 只由一个接收执行流调用；GetControl 在主循环周期调用。 */
typedef struct
{
    I6X_Snapshot snapshot;
    uint8_t frame[I6X_FRAME_BYTES];
    uint8_t used;
    bool initialized;
    /* 以下状态仅由 GetControl 的调用者维护，不由 RX 中断写入。 */
    bool armed;
    bool off_seen;
    bool previous_arm_high;
    uint32_t observed_faults;
    uint32_t observed_frames;
} I6X_Receiver;

/** @brief 给底盘应用的归一化指令，未使能时所有速度均为零。 */
typedef struct
{
    bool online;
    bool armed;
    bool spin;
    float vx; /**< -1～1，车头为正。 */
    float vy; /**< -1～1，车左为正。 */
    float wz; /**< -1～1，逆时针为正。 */
} I6X_Control;

/**
 * @brief 初始化接收对象，检查配置并清空使能状态。
 * @param receiver 长期有效的对象。必须在启动接收前调用。
 * @return true 配置合法；false 空指针或宏配置非法。
 */
bool I6X_Init(I6X_Receiver *receiver);

/**
 * @brief 将任意长度的 SBUS 字节流送入解析器，支持拆包、粘包和错位重同步。
 * @param receiver 已初始化对象。
 * @param data 本次字节块。
 * @param length 字节数量。
 * @param now_ms 与控制端一致的毫秒时基。
 * @note 不在此处修改电机、PID 或底盘。SBUS 无校验和，不能检测所有位错误。
 */
void I6X_Feed(I6X_Receiver *receiver, const uint8_t *data,
              uint16_t length, uint32_t now_ms);

/** @brief 通知 UART 错误，清空半帧并记录故障，恢复后要求重新使能。 */
void I6X_NotifyError(I6X_Receiver *receiver);

/** @brief 使用短临界区复制一致的原始快照，供调试、绘图使用。 */
bool I6X_GetSnapshot(const I6X_Receiver *receiver, I6X_Snapshot *snapshot);

/**
 * @brief 周期生成归一化控制量及使能状态。
 * @param receiver 已初始化接收对象。
 * @param now_ms 当前时间。应在每个控制周期调用，不只在收到新帧时调用。
 * @param control 输出结果。
 * @return true 接收在线且无丢帧/失控标志；false 无有效控制链路。
 * @note 上电或异常恢复后：先收到开关低值，再在三轴回中时拨到高值，才使能。
 *       同一对象只能由一个控制执行流调用本函数。
 */
bool I6X_GetControl(I6X_Receiver *receiver, uint32_t now_ms,
                    I6X_Control *control);

/**
 * @brief 绑定板载 SBUS 口 UART5 并启动 Normal DMA 接收。
 * @param receiver 已初始化、长期有效的接收对象。
 * @param huart CubeMX 生成的 &huart5。
 * @return HAL_OK 启动成功，其他返回值表示配置或启动失败。
 * @note 先生成并调用 MX_UART5_Init()。要求 100000、9B+偶校验、2停止位、RX。
 *       板载反相电路后 UART RXINV 必须关闭。仅支持绑定一个接收机。
 *       使用现有 drv_uart，不占用 USART1 或修改其绘图回调。
 */
HAL_StatusTypeDef I6X_AttachUART5(I6X_Receiver *receiver,
                                 UART_HandleTypeDef *huart);

#endif /* DVC_I6X_H */

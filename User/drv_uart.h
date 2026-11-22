#ifndef __DRV_UART_H__
#define __DRV_UART_H__

//----------includes---------------------

#include "stm32h7xx_hal.h"
#include <stdint.h>

//-------结构体--------------------

/**
 * @brief  UART通信接收回调函数数据类型
 * @note   只要函数无返回值，参数是 (uint8_t*, uint16_t)，就可以赋值给这个UART回调指针
 */

typedef void (*UART_Callback)(uint8_t *Buffer, uint16_t Length);
/** @brief 可选 UART 错误通知，在 IRQ 中调用，不得阻塞。 */
typedef void (*UART_Error_Callback)(uint32_t error_code);

/**
 * @brief UART通信处理结构体
 * @param UART_HandleTypeDef *uart_handle;
 * @param uint8_t *uart_rx_buffer;
 * @param uint16_t uart_rx_size;
 * @param UART_Callback callback_function;
 */
typedef struct
{
  UART_HandleTypeDef *uart_handle;
  uint8_t *uart_rx_buffer;
  uint16_t uart_rx_size;
  UART_Callback callback_function;
  volatile HAL_StatusTypeDef last_rx_status; /* 启动/恢复接收结果，供调试检查。 */
  UART_Error_Callback error_callback; /* 可选；不影响原来的数据回调。 */
} Struct_UART_Manage_Obj;

extern Struct_UART_Manage_Obj UART1_Manage_Object;
extern Struct_UART_Manage_Obj UART2_Manage_Object;
extern Struct_UART_Manage_Obj UART3_Manage_Object;
extern Struct_UART_Manage_Obj UART4_Manage_Object;
extern Struct_UART_Manage_Obj UART5_Manage_Object;
extern Struct_UART_Manage_Obj UART6_Manage_Object;

extern uint8_t UART1_Tx_Data[256];
extern uint8_t UART2_Tx_Data[256];
extern uint8_t UART3_Tx_Data[256];
extern uint8_t UART4_Tx_Data[256];
extern uint8_t UART5_Tx_Data[256];
extern uint8_t UART6_Tx_Data[256];
//-----------函数-----------------------
/**
 * @brief  初始化UART
 * @param  uart_rx_buffer 接收缓冲区
 * @param  uart_rx_size 接收缓冲区大小
 * @param  uart_call_back 接收回调函数
 * @retval
 */
/* 当前使用 Normal DMA。缓冲必须长期有效，位于 AXI SRAM。
 * 本工程已通过 sty2_dma.sct 将默认 RW/ZI、堆栈放在 AXI SRAM。
 * 当前适配以 D-Cache 关闭为前提；开启后须先设计 DMA 缓存一致性。
 * 原有调用可继续忽略返回值，推荐检查 HAL_OK。
 */
HAL_StatusTypeDef UART_Init(UART_HandleTypeDef *huart, uint8_t *uart_rx_buffer, uint16_t uart_rx_size, UART_Callback uart_call_back);

/** @brief 注册/清除错误回调，启动 DMA 前调用；传 NULL 清除，不改变接收回调。 */
HAL_StatusTypeDef UART_Set_Error_Callback(UART_HandleTypeDef *huart,
                                         UART_Error_Callback callback);

/**
 * @brief  发送数据
 * @param  huart UART句柄
 * @param  pData 发送数据
 * @param  Size 发送数据长度
 * @retval
 */
uint8_t UART_Send(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
/* DMA 发送成功后，pData 必须保持有效且不修改，直到发送完成。
 * 同一串口的发送和绘图打包应由同一个任务/主循环串行调用。
 */

/**
 * @brief  UART 串口绘图发送
 * @param  Size 要发送的字节数（由 Serialplot_Output() 返回）
 * @note   发送前必须先调用 Serialplot_Output() 打包数据
 */
void TIM_UART_PeriodElapsedCallback(uint16_t Size);

#endif

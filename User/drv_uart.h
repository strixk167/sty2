#ifndef __DRV_UART_H__
#define __DRV_UART_H__

//----------includes---------------------

#include "stm32h7xx_hal.h"

//-------结构体--------------------

/**
 * @brief  UART通信接收回调函数数据类型
 * @note   只要函数无返回值，参数是 (uint8_t*, uint16_t)，就可以赋值给这个UART回调指针
 */

typedef void (*UART_Callback)(uint8_t *Buffer, uint16_t Length);

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
void UART_Init(UART_HandleTypeDef *huart, uint8_t *uart_rx_buffer, uint16_t uart_rx_size, UART_Callback uart_call_back);

/**
 * @brief  发送数据
 * @param  huart UART句柄
 * @param  pData 发送数据
 * @param  Size 发送数据长度
 * @retval
 */
uint8_t UART_Send(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

/**
 * @brief  UART 串口绘图发送
 * @param  Size 要发送的字节数（由 Serialplot_Output() 返回）
 * @note   发送前必须先调用 Serialplot_Output() 打包数据
 */
void TIM_UART_PeriodElapsedCallback(uint16_t Size);

#endif

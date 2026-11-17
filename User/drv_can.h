#ifndef DRV_CAN_H
#define DRV_CAN_H

// Classic Mod下的CAN FD driver
//-----------includes--------------------

#include "stm32h7xx_hal.h"
#include <string.h>
#include "fdcan.h"

//-----------宏定义-----------------
#define CAN_CLASSIC_MAX_DATA_BYTES 8U

//-----------结构体-----------------
/**
 * @brief CAN接收的信息结构体
 *
 */
typedef struct
{
  FDCAN_RxHeaderTypeDef Header;
  uint8_t Data[CAN_CLASSIC_MAX_DATA_BYTES];
} Struct_CAN_Rx_Buffer;

/**
 * @brief CAN通信接收回调函数数据类型
 *@note 该函数指针类型用于定义CAN通信接收回调函数CAN_Call_Back的原型
 *@note 相当于说Call_Back是“void 函数 (Struct_FDCAN_Rx_Buffer *)”的新名字
 */
typedef void (*CAN_Call_Back)(const Struct_CAN_Rx_Buffer *header);

/**
 * @brief 总线状态对象，总线记录
 * @param receive 目前只这个回调处理函数地址
 */
typedef struct
{
  CAN_Call_Back receive; // 保存一个接收处理函数的地址
} CAN_BusState;
//-----------函数声明-------------

/**
 * @brief 通过句柄地址匹配相应总线状态对象
 *
 * @param hfdcan 句柄
 * @return CAN_BusState* 总线状态对象的地址
 */
static CAN_BusState *CAN_FindBus(const FDCAN_HandleTypeDef *hfdcan);

/**
 * @brief 配置标准ID接收规则、保存应用回调、启动FDCAN和开启接收中断通知的函数
 *
 * @param hfdcan FDCAN句柄
 * @param first_id 精确匹配时的对应ID 范围匹配时的下限，如：0x201
 * @param last_id 精确匹配时的对应ID 范围匹配时的上限，如：0x204
 * @param callback 回调函数
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef CAN_STD_StartRx(FDCAN_HandleTypeDef *hfdcan, uint16_t first_id, uint16_t last_id, CAN_Call_Back callback);

/**
 * @brief 单次发送
 *
 * @param hfdcan fdcan句柄
 * @param id 发送给的ID
 * @param data 发送的数据
 * @param length 数据长度
 * @return HAL_StatusTypeDef 状态
 */
HAL_StatusTypeDef CAN_STD_Send(FDCAN_HandleTypeDef *hfdcan, uint16_t id, const uint8_t *data, uint8_t length);

/**
 * @brief 接收中断函数
 *
 * @param hfdcan 句柄
 * @param events HAL直接传入的事件标志集合
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t events);

#endif

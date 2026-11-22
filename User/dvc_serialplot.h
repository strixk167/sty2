#ifndef __DVC_SERIALPLOT_H__
#define __DVC_SERIALPLOT_H__

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

//-----------宏定义-------------------------
// 串口绘图单条指令最大长度
#define SERIALPLOT_MAX_LEN 128

// 最多支持12个通道
#define SERIALPLOT_MAX_CH 12

//-----------枚举，结构体--------------------
/**
 * @brief  串口绘图传输数据类型
 * @note   用于区分发送的数据是什么类型
 */
typedef enum
{
  Serialplot_Data_Type_UINT8 = 0, // 因为第一个显式写了 = 0 ，后面会自动递增
  Serialplot_Data_Type_INT8,      // = 1
  Serialplot_Data_Type_UINT16,    // = ...
  Serialplot_Data_Type_INT16,
  Serialplot_Data_Type_UINT32,
  Serialplot_Data_Type_INT32,
  Serialplot_Data_Type_FLOAT,
  Serialplot_Data_Type_DOUBLE, // = 7
} Enum_Serialplot_Data_Type;

/**
 * @brief  串口绘图管理对象
 * @note   用于存储串口绘图的数据
 */
typedef struct
{
  UART_HandleTypeDef *huart;           // 串口句柄
  uint8_t *UART_Tx_Data;               // 串口发送缓冲区
  uint16_t UART_Tx_Buffer_Size;        // 串口发送缓冲区大小，由Init传入
  uint8_t UART_Rx_Variable_Num;        // 接收指令字典数量
  char **UART_Rx_Variable_List;        // 接收指令字典列表
  Enum_Serialplot_Data_Type Data_Type; // 数据类型
  uint8_t Frame_Header;                // 帧头
  const void *Data[SERIALPLOT_MAX_CH]; // 需要绘图的变量地址，最多12个通道 const标记这个变量已读，不可修改
  uint8_t Data_Num;                    // 当前发送的数据数量
  volatile int8_t Variable_Index;      // 接收指令字典索引，-1表示未找到
  volatile double Variable_Value;      // 接收到的变量值
  /* UART IDLE/DMA 分块不等于命令边界，跨回调累积到 '#' 再解析。 */
  char Rx_Command[SERIALPLOT_MAX_LEN];
  uint16_t Rx_Command_Length;
  bool Rx_Discarding; /* 超长命令丢弃到下一个 '#'，避免截断后误执行。 */
} Struct_Serialplot_Manage_Obj;

//-----------函数声明------------------------
/**
 * @brief  串口绘图初始化
 * @param  obj 串口绘图管理对象
 * @param  huart 串口句柄
 * @param  tx_buffer 发送缓冲区
 * @param  tx_buffer_size 发送缓冲区大小
 * @param  variable_num 变量数量
 * @param  variable_list 变量列表
 * @param  data_type 数据类型
 * @param  frame_header 帧头
 * @retval None
 */
void Serialplot_Init(Struct_Serialplot_Manage_Obj *obj, UART_HandleTypeDef *huart, uint8_t *tx_buffer, uint16_t tx_buffer_size, uint8_t variable_num, char **variable_list, Enum_Serialplot_Data_Type data_type, uint8_t frame_header);

/**
 * @brief  获取当前接收指令编号
 * @param  obj 串口绘图管理对象
 * @retval 当前接收指令编号
 */
int8_t Serialplot_Get_Variable_Index(Struct_Serialplot_Manage_Obj *obj);

/**
 * @brief  获取当前接收指令值
 * @param  obj 串口绘图管理对象
 * @retval 当前接收指令值
 */

double Serialplot_Get_Variable_Value(Struct_Serialplot_Manage_Obj *obj);

/* 原有两个 Getter 保留。新代码推荐本接口：原子读取索引和值并消费一次。
 * 当前仍为“最近一条命令”邮箱，不是队列；多条未消费命令只保留最后一条。
 * 取到命令后，在控制周期边界校验并应用 PID 配置，不在接收 IRQ 内改 PID。
 */
bool Serialplot_Take_Variable(Struct_Serialplot_Manage_Obj *obj,
                              int8_t *index, double *value);

/**
 * @brief  添加被发送的数据
 * @param  obj 串口绘图管理对象
 * @param  Number 数据数量
 * @param  ... 数据指针
 * @note   调用示例：
 *       Serialplot_Set_Data(&obj, 2, (const void *)&speed, (const void *)&current);
 * @retval None
 */
void Serialplot_Set_Data(Struct_Serialplot_Manage_Obj *obj, uint8_t Number, ...); //... 是可变参数

/**
 * @brief  打包串口绘图数据到发送缓冲区
 * @param  obj 串口绘图管理对象
 * @retval 打包后的数据长度，返回0表示打包失败或超长
 */
uint16_t Serialplot_Output(Struct_Serialplot_Manage_Obj *obj);

/**
 * @brief  定时器周期回调，打包并发送
 * @param  obj 串口绘图管理对象
 * @note   需要在定时器中调用
 * @retval None
 */
void Serialplot_TIM_PeriodElapsedCallback(Struct_Serialplot_Manage_Obj *obj);

/**
 * @brief UART 接收回调，解析 variable=value#
 * @param Rx_Data 接收数据
 * @param Length 接收长度
 */
void Serialplot_UART_Rx_CpltCallback(Struct_Serialplot_Manage_Obj *obj, uint8_t *Rx_Data, uint16_t Length);
#endif

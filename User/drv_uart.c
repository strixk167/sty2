#include "drv_uart.h"
#include "usart.h"

//-----------变量-----------------------------

Struct_UART_Manage_Obj UART1_Manage_Object = {0};
Struct_UART_Manage_Obj UART2_Manage_Object = {0};
Struct_UART_Manage_Obj UART3_Manage_Object = {0};
Struct_UART_Manage_Obj UART4_Manage_Object = {0};
Struct_UART_Manage_Obj UART5_Manage_Object = {0};
Struct_UART_Manage_Obj UART6_Manage_Object = {0};

uint8_t UART1_Tx_Data[256];
uint8_t UART2_Tx_Data[256];
uint8_t UART3_Tx_Data[256];
uint8_t UART4_Tx_Data[256];
uint8_t UART5_Tx_Data[256];
uint8_t UART6_Tx_Data[256];

/**
 * @brief  储存各个 UART 管理对象的地址
 */

static Struct_UART_Manage_Obj *UART_Manage_Objects[] =
    {
        &UART1_Manage_Object,
        &UART2_Manage_Object,
        &UART3_Manage_Object,
        &UART4_Manage_Object,
        &UART5_Manage_Object,
        &UART6_Manage_Object};

#define UART_Manage_Objects_Size 6

//-----------函数-----------------------------

/**
 * @brief  按Instance查找的查找函数
 * @param  USART_TypeDef *Instance : 指向USART硬件实例
 * @retval 各个 UART 管理对象的地址
 */
static Struct_UART_Manage_Obj *UART_Get_Object_By_Instance(USART_TypeDef *Instance)
{
    if (Instance == USART1)
        return &UART1_Manage_Object;
    if (Instance == USART2)
        return &UART2_Manage_Object;
    if (Instance == USART3)
        return &UART3_Manage_Object;
    if (Instance == UART4)
        return &UART4_Manage_Object;
    if (Instance == UART5)
        return &UART5_Manage_Object;
    if (Instance == USART6)
        return &UART6_Manage_Object;
    return NULL;
}
/**
 * @brief  按Instance查找的查找函数
 * @param  UART_HandleTypeDef *huart : 指向USART硬件实例
 * @retval 各个 UART 管理对象的地址
 */
static Struct_UART_Manage_Obj *UART_Get_Obj_by_Handle(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < UART_Manage_Objects_Size; i++)
    {
        if (UART_Manage_Objects[i]->uart_handle == huart)
        {
            return UART_Manage_Objects[i];
        }
    }
    return NULL;
}

/**
 * @brief  初始化UART
 * @param  uart_rx_buffer 接收缓冲区
 * @param  uart_rx_size 接收缓冲区大小
 * @param  uart_call_back 接收回调函数
 * @retval
 */

void UART_Init(UART_HandleTypeDef *huart, uint8_t *uart_rx_buffer, uint16_t uart_rx_size, UART_Callback uart_call_back)
{
    Struct_UART_Manage_Obj *obj = UART_Get_Object_By_Instance(huart->Instance);
    if (obj == NULL) // 如果查找管理对象失败
    {
        return; // 退出函数
    }
    obj->uart_handle = huart;
    obj->uart_rx_buffer = uart_rx_buffer;
    obj->uart_rx_size = uart_rx_size;
    obj->callback_function = uart_call_back;

    HAL_UARTEx_ReceiveToIdle_DMA(huart, uart_rx_buffer, uart_rx_size); // 开启DMA接收
}

/**
 * @brief  发送数据帧
 * @param  huart UART编号
 * @param  pData 数据帧
 * @param  Size 数据帧大小
 * @retval
 */

uint8_t UART_Send(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    if (huart == NULL || pData == NULL || Size == 0) // 检查参数是否合法
    {
        return HAL_ERROR; // 结束当前函数，并向上层返回一个状态值 HAL_ERROR，表示这次操作失败或参数不合法
    }

    return HAL_UART_Transmit_DMA(huart, pData, Size); // 交给HAL用DMA发送
}

/**
 * @brief  UART 串口绘图发送
 * @param  Size 要发送的字节数，由 Serialplot_Output() 返回
 * @note   必须先调用 Serialplot_Output() 打包。帧长要和上位机的 #Channels / Frame Size 一致
 */
void TIM_UART_PeriodElapsedCallback(uint16_t Size)
{
    UART_Send(&huart1, UART1_Tx_Data, Size);
}

/**
 * @brief  UART接收中断回调函数
 * @param  huart UART编号
 * @retval
 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    Struct_UART_Manage_Obj *obj = UART_Get_Obj_by_Handle(huart);
    if (obj == NULL) // 如果查找管理对象失败
    {
        return; // 退出函数
    }
    if (obj->callback_function != NULL)
    {
        obj->callback_function(obj->uart_rx_buffer, Size);
    }

    HAL_UARTEx_ReceiveToIdle_DMA(obj->uart_handle, obj->uart_rx_buffer, obj->uart_rx_size); // 重新开启接收
}

/**
 * @brief  UART错误回调函数
 * @param  huart UART编号
 * @retval
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Obj *obj = UART_Get_Obj_by_Handle(huart);
    if (obj != NULL)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(obj->uart_handle, obj->uart_rx_buffer, obj->uart_rx_size);
    }
}

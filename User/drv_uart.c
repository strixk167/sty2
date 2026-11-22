#include "drv_uart.h"
#include "usart.h"
#include <stdbool.h>
#include <stddef.h>

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

#define UART_Manage_Objects_Size (sizeof(UART_Manage_Objects) / sizeof(UART_Manage_Objects[0]))

/* H723 AXI SRAM：0x24000000～0x2404FFFF，DMA1/2 可访问。
 * 明确拒绝 DTCM 缓冲，避免 HAL 返回成功但 DMA 实际无法读写。
 * 当前链接配置把应用数据和堆栈都放在此区。
 */
static bool UART_IsDMABufferValid(const uint8_t *data, uint16_t size)
{
    uintptr_t address = (uintptr_t)data;
    return data != NULL && size != 0U &&
           address >= 0x24000000UL && address < 0x24050000UL &&
           (uintptr_t)size <= 0x24050000UL - address &&
           (SCB->CCR & SCB_CCR_DC_Msk) == 0U;
}

/* 每次重启接收都关闭 HT；Normal 模式仅处理 IDLE 和 TC。
 * UART 与 RX DMA 中断当前优先级相同，不会互相抢占。
 */
static HAL_StatusTypeDef UART_StartReceive(Struct_UART_Manage_Obj *obj)
{
    HAL_StatusTypeDef status;
    status = HAL_UARTEx_ReceiveToIdle_DMA(obj->uart_handle,
                                         obj->uart_rx_buffer,
                                         obj->uart_rx_size);
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(obj->uart_handle->hdmarx, DMA_IT_HT);
    }
    obj->last_rx_status = status;
    return status;
}

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
    if (huart == NULL)
    {
        return NULL;
    }
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

HAL_StatusTypeDef UART_Set_Error_Callback(UART_HandleTypeDef *huart,
                                         UART_Error_Callback callback)
{
    Struct_UART_Manage_Obj *obj;
    if (huart == NULL) return HAL_ERROR;
    obj = UART_Get_Object_By_Instance(huart->Instance);
    if (obj == NULL) return HAL_ERROR;
    obj->error_callback = callback;
    return HAL_OK;
}

HAL_StatusTypeDef UART_Init(UART_HandleTypeDef *huart, uint8_t *uart_rx_buffer, uint16_t uart_rx_size, UART_Callback uart_call_back)
{
    Struct_UART_Manage_Obj *obj;
    if (huart == NULL || huart->hdmarx == NULL ||
        huart->hdmarx->Init.Mode != DMA_NORMAL ||
        !UART_IsDMABufferValid(uart_rx_buffer, uart_rx_size))
    {
        return HAL_ERROR;
    }
    obj = UART_Get_Object_By_Instance(huart->Instance);
    if (obj == NULL) // 如果查找管理对象失败
    {
        return HAL_ERROR;
    }
    if (huart->RxState != HAL_UART_STATE_READY)
    {
        return HAL_BUSY;
    }
    obj->uart_handle = huart;
    obj->uart_rx_buffer = uart_rx_buffer;
    obj->uart_rx_size = uart_rx_size;
    obj->callback_function = uart_call_back;

    return UART_StartReceive(obj);
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
    if (huart == NULL || huart->hdmatx == NULL ||
        huart->hdmatx->Init.Mode != DMA_NORMAL ||
        !UART_IsDMABufferValid(pData, Size))
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
    if (Size <= sizeof(UART1_Tx_Data))
    {
        UART_Send(&huart1, UART1_Tx_Data, Size);
    }
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
    /* 防御性忽略 HT，不能解析前半段后又重复解析整个缓冲。 */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
    {
        return;
    }
    if (Size > 0U && Size <= obj->uart_rx_size &&
        obj->callback_function != NULL)
    {
        obj->callback_function(obj->uart_rx_buffer, Size);
    }

    /* 回调运行在 IRQ，不能阻塞；缓冲只保证在本次回调内稳定。 */
    UART_StartReceive(obj);
}

/**
 * @brief  UART错误回调函数
 * @param  huart UART编号
 * @retval
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Struct_UART_Manage_Obj *obj = UART_Get_Obj_by_Handle(huart);
    if (obj != NULL && obj->error_callback != NULL)
    {
        obj->error_callback(huart->ErrorCode);
    }
    if (obj != NULL && huart->RxState == HAL_UART_STATE_READY)
    {
        UART_StartReceive(obj);
    }
}

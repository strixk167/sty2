#include "dvc_serialplot.h"
#include "drv_uart.h"

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

//-----------内部函数---------------------
/**
 * @brief  获取数据类型对应的字节数
 * @param  data_type 数据类型
 * @retval 对应数据类型所占字节
 */
static uint8_t Serialplot_Get_Data_Type_Size(Enum_Serialplot_Data_Type data_type)
{
    switch (data_type)
    {
    case Serialplot_Data_Type_INT8:
    case Serialplot_Data_Type_UINT8:
        return 1;
    case Serialplot_Data_Type_INT16:
    case Serialplot_Data_Type_UINT16:
        return 2;
    case Serialplot_Data_Type_INT32:
    case Serialplot_Data_Type_UINT32:
    case Serialplot_Data_Type_FLOAT:
        return 4;
    case Serialplot_Data_Type_DOUBLE:
        return 8;
    default:
        return 0;
    }
}

//------------函数--------------

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
void Serialplot_Init(Struct_Serialplot_Manage_Obj *obj, UART_HandleTypeDef *huart, uint8_t *tx_buffer, uint16_t tx_buffer_size, uint8_t variable_num, char **variable_list, Enum_Serialplot_Data_Type data_type, uint8_t frame_header)
{
    if (obj == NULL)
    {
        return;
    }

    obj->huart = huart;
    obj->UART_Tx_Data = tx_buffer;
    obj->UART_Tx_Buffer_Size = tx_buffer_size;
    obj->UART_Rx_Variable_Num = variable_num;
    obj->UART_Rx_Variable_List = variable_list;
    obj->Data_Type = data_type;
    obj->Frame_Header = frame_header;

    obj->Data_Num = 0;
    obj->Variable_Index = -1;
    obj->Variable_Value = 0;

    for (uint8_t i = 0; i < SERIALPLOT_MAX_CH; i++)
    {
        obj->Data[i] = NULL;
    }

    if (obj->UART_Tx_Data != NULL && obj->UART_Tx_Buffer_Size > 0)
    {
        memset(obj->UART_Tx_Data, 0, obj->UART_Tx_Buffer_Size); // 清空发送缓冲区
        obj->UART_Tx_Data[0] = frame_header;
    }
}

/**
 * @brief  获取当前接收指令编号
 * @param  obj 串口绘图管理对象
 * @retval 当前接收指令编号
 */
int8_t Serialplot_Get_Variable_Index(Struct_Serialplot_Manage_Obj *obj)
{
    if (obj == NULL)
    {
        return -1;
    }

    return obj->Variable_Index;
}

/**
 * @brief  获取当前接收指令值
 * @param  obj 串口绘图管理对象
 * @retval 当前接收指令值
 */

double Serialplot_Get_Variable_Value(Struct_Serialplot_Manage_Obj *obj)
{
    if (obj == NULL)
    {
        return 0.0;
    }

    return obj->Variable_Value;
}

/**
 * @brief  添加被发送的数据
 * @param  obj 串口绘图管理对象
 * @param  Number 数据数量
 * @param  ... 数据指针
 * @note   调用示例：
 *       Serialplot_Set_Data(&obj, 2, (const void *)&speed, (const void *)&current);
 * @retval None
 */
void Serialplot_Set_Data(Struct_Serialplot_Manage_Obj *obj, uint8_t Number, ...) //... 是可变参数
{
    if (obj == NULL)
    {
        return;
    }

    if (Number > SERIALPLOT_MAX_CH)
    {
        Number = SERIALPLOT_MAX_CH;
    }

    va_list data_ptr;           // 定义可变参数指针，用来记录可变参数在栈上的位置，相当于遍历参数列表的游标
    va_start(data_ptr, Number); // 初始化游标data_ptr，让它指向第一个可变参数。Number是最后一个固定参数

    for (uint8_t i = 0; i < Number; i++)
    {
        obj->Data[i] = va_arg(data_ptr, const void *); // 取出当前data_ptr指向的参数，并且自动把它移动到下一个参数
    }

    va_end(data_ptr); // 收尾

    obj->Data_Num = Number;
}

/**
 * @brief  打包串口绘图数据到发送缓冲区
 * @param  obj 串口绘图管理对象
 * @retval 打包后的数据长度，返回0表示打包失败或超长
 */
uint16_t Serialplot_Output(Struct_Serialplot_Manage_Obj *obj)
{
    if (obj == NULL || obj->UART_Tx_Data == NULL || obj->UART_Tx_Buffer_Size == 0) // 任意一个成立
    {
        return 0;
    }

    uint8_t type_size = Serialplot_Get_Data_Type_Size(obj->Data_Type);
    uint16_t len = 1 + obj->Data_Num * type_size;

    if (len > obj->UART_Tx_Buffer_Size)
    {
        return 0;
    }

    obj->UART_Tx_Data[0] = obj->Frame_Header; // 第0位放帧头

    uint16_t offset = 1;
    for (uint8_t i = 0; i < obj->Data_Num; i++)
    {
        if (obj->Data[i] == NULL)
        {
            memset(&obj->UART_Tx_Data[offset], 0, type_size);
        }
        else
        {
            memcpy(&obj->UART_Tx_Data[offset], obj->Data[i], type_size);
        }
        offset += type_size;
    }

    return len;
}

/**
 * @brief  定时器周期回调，打包并发送
 * @param  obj 串口绘图管理对象
 * @note   需要在定时器中调用
 * @retval None
 */
void Serialplot_TIM_PeriodElapsedCallback(Struct_Serialplot_Manage_Obj *obj)
{
    if (obj == NULL || obj->UART_Tx_Data == NULL || obj->UART_Tx_Buffer_Size == 0) // 任意一个成立
    {
        return;
    }

    if (obj->huart == NULL || obj->huart->gState != HAL_UART_STATE_READY) // 如果UART还在DMA发送中就跳过本次，避免覆盖发送缓冲区
    {
        return;
    }

    uint16_t len = Serialplot_Output(obj);

    if (len > 0)
    {
        UART_Send(obj->huart, obj->UART_Tx_Data, len);
    }
}

/**
 * @brief UART 接收回调，解析 variable=value#
 * @param Rx_Data 接收数据
 * @param Length 接收长度
 */
void Serialplot_UART_Rx_CpltCallback(Struct_Serialplot_Manage_Obj *obj, uint8_t *Rx_Data, uint16_t Length)
{
    if (obj == NULL || Rx_Data == NULL || Length == 0) // 任意一个成立
    {
        return;
    }

    // 查找“ = ”，也就是说到eq_pos之前是变量名
    uint16_t eq_pos = 0;
    while (eq_pos < Length && Rx_Data[eq_pos] != '=')
    {
        eq_pos++;
    }

    if (eq_pos >= Length)
    {
        return;
    }

    // 查找“ # ”，也就是说查找整条指令的结束位置
    uint16_t end_pos = eq_pos + 1;
    while (end_pos < Length && Rx_Data[end_pos] != '#')
    {
        end_pos++;
    }

    if (end_pos >= Length)
    {
        return;
    }

    if (eq_pos >= SERIALPLOT_MAX_LEN)
    {
        return;
    }

    // 提取变量名
    char tmp_variable_name[SERIALPLOT_MAX_LEN];
    memcpy(tmp_variable_name, Rx_Data, eq_pos);
    tmp_variable_name[eq_pos] = '\0'; // 给字符串手动加上结束符

    // 在字典中查找
    obj->Variable_Index = -1;

    for (uint8_t i = 0; i < obj->UART_Rx_Variable_Num; i++)
    {
        if (obj->UART_Rx_Variable_List != NULL && strcmp(tmp_variable_name, obj->UART_Rx_Variable_List[i]) == 0)
        {
            obj->Variable_Index = i;
            break;
        }
    }

    if (obj->Variable_Index == -1)
    {
        return;
    }

    // 提取数值字符串
    uint16_t value_len = end_pos - eq_pos - 1; // 计算数值长度

    char tmp_value[32];
    if (value_len >= sizeof(tmp_value))
    {
        value_len = sizeof(tmp_value) - 1; // 最后一个字节存\0
    }

    memcpy(tmp_value, &Rx_Data[eq_pos + 1], value_len); // 从Rx_Data把数据放入tmp_value
    tmp_value[value_len] = '\0';

    obj->Variable_Value = strtod(tmp_value, NULL); // 把类型转换为double类型
}

#include "dvc_motor.h"

//---------------内部函数-----------------

/**
 * @brief 检查控制方式是否合法
 *
 * @param method 控制方式，包括
    Control_Method_OPENLOOP = 0, // 开环控制
    Control_Method_TORQUE,       // 扭矩环控制
    Control_Method_OMEGA,        // 速度环控制
    Control_Method_ANGLE,        // 角度环控制
 * @return true
 * @return false
 */
static bool Motor_IsControlMethodValid(Enum_Control_Method method)
{
    return (unsigned int)method <= (unsigned int)Control_Method_ANGLE;
}

/**
 * @brief 检查句柄是否合法
 *
 * @param handle 句柄
 * @return true
 * @return false
 */
static bool Motor_IsHandleValid(const FDCAN_HandleTypeDef *handle)
{
    return handle == &hfdcan1 || handle == &hfdcan2 || handle == &hfdcan3;
}

/**
 * @brief 复位一个PID状态
 *
 * @param pid
 */
static void Motor_ResetPID(Struct_PID *pid)
{
    pid->Integral_Error = 0.0f;

    pid->Pre_Now = pid->Now;
    pid->Pre_Target = pid->Target;
    pid->Pre_Error = pid->Target - pid->Now;

    pid->Pre_Out = 0.0f;
    pid->Out = 0.0f;
    pid->Pre_F_Out = 0.0f;
}

//---------------初始化-------------------
/**
 * @brief GM6020初始化
 *
 * @param obj
 * @param hcan
 * @param Motor_ID
 * @param Control_Method
 * @param Encoder_Offset
 * @param Omega_Max
 */
void GM6020_Init(Struct_Motor_Manage_Obj *obj, CAN_HandleTypeDef *hcan, uint32_t Motor_ID, Enum_Control_Method Control_Method, int32_t Encoder_Offset, float Omega_Max)
{
    obj->Motor_Type = GM6020;
    obj->hcan = hcan;

    obj->Motor_ID = Motor_ID;
    obj->Control_Method = Control_Method;
    obj->Encoder_Offset = Encoder_Offset;
    obj->Omega_Max = Omega_Max;

    obj->Output_Max = Output_Max_6020;
    obj->Gearbox_Rate = 1.0f; // GM6020 直驱
    obj->Torque_Max = 1.2f;   // 仅开环扭矩模式用

    obj->Encoder_Num_Per_Round = Encoder_Num_Per_Round_6020; // 提前设好，避免首次回调时除数还是 0

    obj->Flag = 0;
    obj->Pre_Flag = 0;
    obj->Pre_Encoder = 0;
    obj->Rx_Encoder = 0;
    obj->Total_Round = 0;
    obj->Total_Encoder = 0;
    obj->Now_Angle = 0.0f;
    obj->Now_Omega = 0.0f;
    obj->Output = 0.0f;
    CAN_ID_Manage(obj);
    // 初始化CAN发送列表（.ID 必须在 CAN_ID_Manage 之后设，漏了就发成 ID=0x000 的帧）
    obj->CAN_Tx_Buffer[0].ID = obj->CAN_Send_ID;
    obj->CAN_Tx_Buffer[0].IDE = CAN_ID_STD;
    obj->CAN_Tx_Buffer[0].RTR = CAN_RTR_DATA;
    obj->CAN_Tx_Buffer[0].DLC = 8;
    obj->CAN_Tx_Buffer[0].period_counter = 0;
    obj->CAN_Tx_Buffer[0].period_limit = 1; // 1 = 每次调用都发
    memset(obj->CAN_Tx_Buffer[0].Data, 0, 8);
    CAN_Filter_Mask_Config(obj->hcan, CAN_FILTER(13) | CAN_FIFO1 | CAN_STDID | CAN_DATA_TYPE, obj->CAN_Receive_ID, 0x7FF); // 掩码初始化
    // 注册CAN发送列表
    CAN_RegisterTxList(obj->hcan, obj->CAN_Tx_Buffer, sizeof(obj->CAN_Tx_Buffer) / sizeof(obj->CAN_Tx_Buffer[0]));
}
/**
 * @brief C6X0电调支持的电机的初始化
 *
 * @param obj
 * @param hcan
 * @param Motor_ID
 * @param Motor_Type
 * @param Control_Method
 * @param Gearbox_Rate
 * @param Torque_Max
 */
void Motor_C6X0_Init(Struct_Motor_Manage_Obj *obj, CAN_HandleTypeDef *hcan, uint32_t Motor_ID, Enum_Motor_Type Motor_Type, Enum_Control_Method Control_Method, float Gearbox_Rate, float Torque_Max)
{
    switch (Motor_Type)
    {
    case (C610_M2006):
    {
        obj->Motor_Type = C610_M2006;
    }
    break;
    case (C620_M3508):
    {
        obj->Motor_Type = C620_M3508;
    }
    break;
    default:
        break;
    }
    obj->hcan = hcan;
    obj->Motor_ID = Motor_ID;
    obj->Control_Method = Control_Method;
    obj->Gearbox_Rate = Gearbox_Rate;
    obj->Torque_Max = Torque_Max;

    // 按电机类型定 Output_Max / Encoder_Num_Per_Round
    if (obj->Motor_Type == C610_M2006)
    {
        obj->Output_Max = Output_Max_2006;
        obj->Encoder_Num_Per_Round = Encoder_Num_Per_Round_2006;
    }
    else
    {
        obj->Output_Max = Output_Max_3508;
        obj->Encoder_Num_Per_Round = Encoder_Num_Per_Round_3508;
    }

    obj->Flag = 0;
    obj->Pre_Flag = 0;
    obj->Pre_Encoder = 0;
    obj->Rx_Encoder = 0;
    obj->Total_Round = 0;
    obj->Total_Encoder = 0;
    obj->Output = 0.0f;

    CAN_ID_Manage(obj);

    // 初始化CAN发送列表
    obj->CAN_Tx_Buffer[0].ID = obj->CAN_Send_ID;
    obj->CAN_Tx_Buffer[0].IDE = CAN_ID_STD;
    obj->CAN_Tx_Buffer[0].RTR = CAN_RTR_DATA;
    obj->CAN_Tx_Buffer[0].DLC = 8;
    obj->CAN_Tx_Buffer[0].period_counter = 0;
    obj->CAN_Tx_Buffer[0].period_limit = 1;
    memset(obj->CAN_Tx_Buffer[0].Data, 0, obj->CAN_Tx_Buffer[0].DLC);
    CAN_Filter_Mask_Config(obj->hcan, CAN_FILTER(13) | CAN_FIFO1 | CAN_STDID | CAN_DATA_TYPE, obj->CAN_Receive_ID, 0x7FF); // 掩码初始化
    // 注册CAN发送列表
    CAN_RegisterTxList(obj->hcan, obj->CAN_Tx_Buffer, sizeof(obj->CAN_Tx_Buffer) / sizeof(obj->CAN_Tx_Buffer[0]));
}

//-------------参数写入CAN发送列表-------------------
/**
 * @brief 把 obj->Output 写到本电机 ID 对应的那 2 个字节
 * @note  0x1FF/0x2FF 是广播帧：ID1~4 依次占 Data[0..7]，只写自己那 2 字节
 */
void Output(Struct_Motor_Manage_Obj *obj)
{
    uint8_t offset;
    int32_t out;
    uint16_t raw;

    switch (obj->Motor_ID)
    {
    case 1:
    case 5:
        offset = 0;
        break;
    case 2:
    case 6:
        offset = 2;
        break;
    case 3:
    case 7:
        offset = 4;
        break;
    case 4:
    case 8:
        offset = 6;
        break;
    default:
        return;
    }

    out = (int32_t)obj->Output; // 先转有符号整型，负数才正确

    // 输出限幅
    if (out > 30000)
    {
        out = 30000;
    }
    if (out < -30000)
    {
        out = -30000;
    }

    raw = (uint16_t)(int16_t)out; // int16 的位模式

    // 必须先移位再截断，写成 (uint8_t)(obj->Output) >> 8 高字节会恒为 0
    obj->CAN_Tx_Buffer[0].Data[offset] = (uint8_t)(raw >> 8);
    obj->CAN_Tx_Buffer[0].Data[offset + 1] = (uint8_t)(raw & 0xFF);
}

//-------------参数读取-------------------
/**
 * @brief 获取最大输出电压
 *
 * @param obj
 * @return uint16_t 最大输出电压
 */
uint16_t Get_Output_Max(Struct_Motor_Manage_Obj *obj)
{
    return obj->Output_Max;
}

/**
 * @brief 获取电机状态
 *
 * @param obj
 * @return Enum_CAN_Motor_Status 电机状态
 */
Enum_CAN_Motor_Status Get_CAN_Motor_Status(Struct_Motor_Manage_Obj *obj)
{
    return obj->CAN_Motor_Status;
}

/**
 * @brief 获取当前角度 rad
 *
 * @param obj
 * @return float 当前的角度
 */
float Get_Now_Angle(Struct_Motor_Manage_Obj *obj)
{
    return obj->Now_Angle;
}

/**
 * @brief 获取当前速度 rad/s
 *
 * @param obj
 * @return float 当前的速度
 */
float Get_Now_Omega(Struct_Motor_Manage_Obj *obj)
{
    return obj->Now_Omega;
}

float Get_Now_Torque(Struct_Motor_Manage_Obj *obj)
{
    return obj->Now_Torque;
}

uint8_t Get_Now_Temperature(Struct_Motor_Manage_Obj *obj)
{
    return obj->Now_Temperature;
}

/**
 * @brief 获取电机控制方式
 *
 * @param obj
 * @return Enum_Control_Method 电机控制方式
 */
Enum_Control_Method Get_Control_Method(Struct_Motor_Manage_Obj *obj)
{
    return obj->Control_Method;
}

/**
 * @brief 获取目标的角度 rad
 *
 * @param obj
 * @return float 目标的角度
 */
float Get_Target_Angle(Struct_Motor_Manage_Obj *obj)
{
    return obj->Target_Angle;
}

/**
 * @brief 获取目标的速度 rad/s
 *
 * @param obj
 * @return float 目标的速度
 */
float Get_Target_Omega(Struct_Motor_Manage_Obj *obj)
{
    return obj->Target_Omega;
}

/**
 * @brief 获取目标的扭矩
 *
 * @param obj
 * @return float 目标的扭矩
 */
float Get_Target_Torque(Struct_Motor_Manage_Obj *obj)
{
    return obj->Target_Torque;
}

/**
 * @brief 获取输出量
 *
 * @param obj
 * @return float 输出量
 */
float Get_Output(Struct_Motor_Manage_Obj *obj)
{
    return obj->Output;
}

/**
 * @brief 设置电机控制方式
 *
 * @param obj
 * @param Control_Method
 */
void Set_Control_Method(Struct_Motor_Manage_Obj *obj, Enum_Control_Method Control_Method)
{
    obj->Control_Method = Control_Method;
}

/**
 * @brief 设置目标的角度 rad
 *
 * @param obj
 * @param Target_Angle
 */
void Set_Target_Angle(Struct_Motor_Manage_Obj *obj, float Target_Angle)
{
    obj->Target_Angle = Target_Angle;
}

/**
 * @brief 设置目标的速度 rad/s
 *
 * @param obj
 * @param Target_Omega
 */
void Set_Target_Omega(Struct_Motor_Manage_Obj *obj, float Target_Omega)
{
    obj->Target_Omega = Target_Omega;
}

/**
 * @brief 设置目标的扭矩
 *
 * @param obj
 * @param Target_Torque
 */
void Set_Target_Torque(Struct_Motor_Manage_Obj *obj, float Target_Torque)
{
    obj->Target_Torque = Target_Torque;
}

/**
 * @brief 设置输出量
 *
 * @param obj
 * @param Output
 */
void Set_Out(Struct_Motor_Manage_Obj *obj, float Output)
{
    obj->Output = Output;
}

//---------------------------------------------------------------
/**
 * @brief 电机的CAN通信中断回调函数
 *
 * @param obj
 * @param Rx_Data
 */
void Motor_CAN_RxCpltCallback(Struct_Motor_Manage_Obj *obj, uint8_t *Rx_Data)
{
    int32_t delta_encoder;

    obj->Pre_Encoder = obj->Rx_Encoder;

    obj->Rx_Encoder = (uint16_t)((Rx_Data[0] << 8) | Rx_Data[1]);
    obj->Rx_Omega = (int16_t)((Rx_Data[2] << 8) | Rx_Data[3]); // 转速是有符号的，反转必须能表示为负数
    obj->Rx_Torque = (int16_t)((Rx_Data[4] << 8) | Rx_Data[5]);
    obj->Rx_Temperature = Rx_Data[6];

    // 第一帧：把当前位置设成软件零点（否则 Now_Angle 是编码器绝对角，角度环一上电就甩）
    if (obj->Flag == 0)
    {
        obj->Encoder_Offset = -(int32_t)obj->Rx_Encoder;
        obj->Flag += 1;
        return;
    }
    obj->Flag += 1;

    delta_encoder = (int32_t)obj->Rx_Encoder - (int32_t)obj->Pre_Encoder;
    if (delta_encoder < -4096)
    {
        // 正方向转过了一圈
        obj->Total_Round++;
    }
    else if (delta_encoder > 4096)
    {
        // 反方向转过了一圈
        obj->Total_Round--;
    }
    obj->Total_Encoder = obj->Total_Round * (int32_t)obj->Encoder_Num_Per_Round + (int32_t)obj->Rx_Encoder + obj->Encoder_Offset;
    switch (obj->Motor_Type)
    {
    case (GM6020):
    {
        obj->Encoder_Num_Per_Round = Encoder_Num_Per_Round_6020;
        obj->Now_Omega = (float)obj->Rx_Omega * RPM_TO_RADPS;
    }
    break;
    case (C610_M2006):
    {
        obj->Encoder_Num_Per_Round = Encoder_Num_Per_Round_2006;
        obj->Now_Omega = (float)obj->Rx_Omega * RPM_TO_RADPS / obj->Gearbox_Rate;
    }
    break;
    case (C620_M3508):
    {
        obj->Encoder_Num_Per_Round = Encoder_Num_Per_Round_3508;
        obj->Now_Omega = (float)obj->Rx_Omega * RPM_TO_RADPS / obj->Gearbox_Rate;
    }
    break;
    default:
        break;
    }
    obj->Now_Angle = (float)obj->Total_Encoder / (float)obj->Encoder_Num_Per_Round * 2.0f * PI;
    obj->Now_Torque = obj->Rx_Torque;
    obj->Now_Temperature = obj->Rx_Temperature;
}

/**
 * @brief TIM定时器中断电机检查回调函数
 *
 * @param obj
 */
void TIM_Status_PeriodElapsedCallback(Struct_Motor_Manage_Obj *obj)
{
    if (obj->Flag == obj->Pre_Flag) // 电机没有发来报文，电机可能寄了
    {
        obj->CAN_Motor_Status = CAN_Motor_Status_DISABLE;
        PID_Set_Integral_Error(&obj->PID_Angle, 0.0f);
        PID_Set_Integral_Error(&obj->PID_Omega, 0.0f);
        PID_Set_Integral_Error(&obj->PID_Torque, 0.0f);
    }
    else
    {
        obj->CAN_Motor_Status = CAN_Motor_Status_ENABLE;
    }
    obj->Pre_Flag = obj->Flag; // 更新当前的状态
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void TIM_PID_PeriodElapsedCallback(Struct_Motor_Manage_Obj *obj)
{
    // 没有收到电机报文时不允许控制：
    // Now_Angle / Now_Omega 初值都是 0，第一帧到来前它们是假反馈。
    // 此时若照常闭环，角度环会拿着"角度=0"去追目标，积分一路涨到饱和，
    // 等第一帧真实角度到了就已经晚了 —— 这就是上电瞬间飞转的窗口期。
    if (obj->CAN_Motor_Status != CAN_Motor_Status_ENABLE)
    {
        obj->Output = 0.0f;
        PID_Set_Integral_Error(&obj->PID_Angle, 0.0f);
        PID_Set_Integral_Error(&obj->PID_Omega, 0.0f);
        PID_Set_Integral_Error(&obj->PID_Torque, 0.0f);
        Output(obj); // 仍要发一帧 0 电压，让电调知道当前指令
        return;
    }

    switch (obj->Motor_Type)
    {
    case (GM6020):
    {
        switch (obj->Control_Method)
        {
        case (Control_Method_OPENLOOP):
        {
            obj->Output = obj->Target_Omega / obj->Omega_Max * (float)obj->Output_Max;
        }
        break;

        case (Control_Method_TORQUE):
        {
            PID_Set_Target(&obj->PID_Torque, obj->Target_Torque);
            PID_Set_Now(&obj->PID_Torque, obj->Now_Torque);
            PID_TIM_Adjust_PeriodElapsedCallback(&obj->PID_Torque);
            obj->Output = PID_Get_Out(&obj->PID_Torque);
        }
        break;

        case (Control_Method_OMEGA):
        {
            // 单环：速度环直接输出电压（调内环时用这个模式）
            PID_Set_Target(&obj->PID_Omega, obj->Target_Omega);
            PID_Set_Now(&obj->PID_Omega, obj->Now_Omega);
            PID_TIM_Adjust_PeriodElapsedCallback(&obj->PID_Omega);

            obj->Output = PID_Get_Out(&obj->PID_Omega);
        }
        break;

        case (Control_Method_ANGLE):
        {
            // 双环：角度环 -> 速度环 -> 电压
            PID_Set_Target(&obj->PID_Angle, obj->Target_Angle);
            PID_Set_Now(&obj->PID_Angle, obj->Now_Angle);
            PID_TIM_Adjust_PeriodElapsedCallback(&obj->PID_Angle);
            obj->Target_Omega = PID_Get_Out(&obj->PID_Angle);

            PID_Set_Target(&obj->PID_Omega, obj->Target_Omega);
            PID_Set_Now(&obj->PID_Omega, obj->Now_Omega);
            PID_TIM_Adjust_PeriodElapsedCallback(&obj->PID_Omega);
            obj->Output = PID_Get_Out(&obj->PID_Omega);
        }
        break;

        default:
        {
            obj->Output = 0.0f;
        }
        break;
        }
    }
    break;

    case (C610_M2006):
    case (C620_M3508):
    {
        switch (obj->Control_Method)
        {
        case (Control_Method_OPENLOOP):
        {
            obj->Output = obj->Target_Torque / obj->Torque_Max * (float)obj->Output_Max;
        }
        break;

        case (Control_Method_TORQUE):
        {
            obj->Output = obj->Target_Torque / obj->Torque_Max * (float)obj->Output_Max;
        }
        break;

        case (Control_Method_OMEGA):
        {
            PID_Set_Target(&obj->PID_Omega, obj->Target_Omega);
            PID_Set_Now(&obj->PID_Omega, obj->Now_Omega);
            PID_TIM_Adjust_PeriodElapsedCallback(&obj->PID_Omega);
            obj->Output = PID_Get_Out(&obj->PID_Omega);
        }
        break;

        case (Control_Method_ANGLE):
        {
            PID_Set_Target(&obj->PID_Angle, obj->Target_Angle);
            PID_Set_Now(&obj->PID_Angle, obj->Now_Angle);
            PID_TIM_Adjust_PeriodElapsedCallback(&obj->PID_Angle);

            obj->Target_Omega = PID_Get_Out(&obj->PID_Angle);

            PID_Set_Target(&obj->PID_Omega, obj->Target_Omega);
            PID_Set_Now(&obj->PID_Omega, obj->Now_Omega);
            PID_TIM_Adjust_PeriodElapsedCallback(&obj->PID_Omega);

            obj->Output = PID_Get_Out(&obj->PID_Omega);
        }
        break;

        default:
        {
            obj->Output = 0.0f;
        }
        break;
        }
    }
    break;

    default:
        break;
    }

    Output(obj);
}

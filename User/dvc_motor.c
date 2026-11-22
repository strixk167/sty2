#include "dvc_motor.h"

/* DJI 协议固定为四个两字节命令；型号参数只在这里集中定义。 */
#define MOTOR_COMMAND_BYTES 2U
typedef char Motor_Frame_Size_Check[
    (MOTOR_TX_GROUP_SLOTS == 4U &&
     MOTOR_TX_GROUP_SLOTS * MOTOR_COMMAND_BYTES == MOTOR_CAN_DATA_BYTES &&
     MOTOR_CAN_DATA_BYTES == CAN_CLASSIC_MAX_DATA_BYTES) ? 1 : -1];

typedef struct
{
    Enum_Motor_Type Type;
    uint32_t Max_ID;
    uint16_t Feedback_Base;
    uint16_t Command_First;
    uint16_t Command_Second;
    Enum_Motor_Encoder_Num_Per_Round Encoder_Counts;
    Enum_Motor_Output_Max Output_Limit;
} Motor_Protocol_Config;

static const Motor_Protocol_Config Motor_Protocols[] =
{
    {GM6020, 7U, 0x204U, 0x1FFU, 0x2FFU,
     Encoder_Num_Per_Round_6020, Output_Max_6020},
    {C610_M2006, 8U, 0x200U, 0x200U, 0x1FFU,
     Encoder_Num_Per_Round_2006, Output_Max_2006},
    {C620_M3508, 8U, 0x200U, 0x200U, 0x1FFU,
     Encoder_Num_Per_Round_3508, Output_Max_3508}
};

static const Motor_Protocol_Config *Motor_GetProtocol(Enum_Motor_Type type)
{
    size_t index;
    for (index = 0U; index < sizeof(Motor_Protocols) / sizeof(Motor_Protocols[0]); ++index)
    {
        if (Motor_Protocols[index].Type == type)
        {
            return &Motor_Protocols[index];
        }
    }
    return NULL;
}

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

/* 检查组的总线、命令 ID 和限幅；运行中组参数被误改也能被发现。 */
static bool Motor_IsGroupValid(const Struct_Motor_Tx_Group *group)
{
    const Motor_Protocol_Config *protocol;
    if (group == NULL || !Motor_IsHandleValid(group->Handle) ||
        group->Command_Limit < 0)
    {
        return false;
    }
    protocol = Motor_GetProtocol(group->Motor_Type);
    return protocol != NULL &&
           (group->Command_ID == protocol->Command_First ||
            group->Command_ID == protocol->Command_Second) &&
           (int32_t)group->Command_Limit <= (int32_t)protocol->Output_Limit;
}

/**
 * @brief 检查型号和电调编号，分配对应CAN ID。 仅在电机初始化时调用。 失败时不写入 ID。
 *
 * @param obj 电机管理结构体
 * @return true
 * @return false
 */
static bool CAN_ID_Manage(Struct_Motor_Manage_Obj *obj)
{
    const Motor_Protocol_Config *protocol;

    if (obj == NULL || obj->Motor_ID < 1U)
    {
        return false;
    }

    protocol = Motor_GetProtocol(obj->Motor_Type);
    if (protocol == NULL || obj->Motor_ID > protocol->Max_ID)
    {
        return false;
    }

    obj->CAN_Send_ID = obj->Motor_ID <= MOTOR_TX_GROUP_SLOTS
                           ? protocol->Command_First : protocol->Command_Second;
    obj->CAN_Receive_ID = protocol->Feedback_Base + obj->Motor_ID;

    return true;
}

/**
 * @brief 将单台电机的输出指令写入共享CAN数据帧的对应槽位
 *
 * @param obj 电机管理对象
 * @param data 共享CAN发送数据数组组
 * @param command_limit 本组每台电机允许的指令幅值上限
 * @return true
 * @return false
 */
static bool Output(const Struct_Motor_Manage_Obj *obj, uint8_t data[MOTOR_CAN_DATA_BYTES], int16_t command_limit)
{
    const Motor_Protocol_Config *protocol;
    uint32_t offset;
    float command = 0.0f;
    float limit;
    uint16_t raw;

    if (obj == NULL || data == NULL || command_limit < 0)
    {
        return false;
    }

    protocol = Motor_GetProtocol(obj->Motor_Type);
    if (protocol == NULL || obj->Motor_ID == 0U ||
        obj->Motor_ID > protocol->Max_ID ||
        (int32_t)obj->Output_Max <= 0 ||
        (int32_t)obj->Output_Max > (int32_t)protocol->Output_Limit)
    {
        return false;
    }
    offset = ((obj->Motor_ID - 1U) % MOTOR_TX_GROUP_SLOTS) * MOTOR_COMMAND_BYTES;

    limit = (float)command_limit;

    if (limit > (float)obj->Output_Max) // 限幅
    {
        limit = (float)obj->Output_Max;
    }

    if (obj->Command_Valid && obj->Has_Feedback &&
        obj->CAN_Motor_Status == CAN_Motor_Status_ENABLE && isfinite(obj->Output))
    {
        command = obj->Output;
    }

    if (command > limit)
    {
        command = limit;
    }
    else if (command < -limit)
    {
        command = -limit;
    }

    raw = (uint16_t)(int16_t)command;

    data[offset] = (uint8_t)(raw >> 8U);
    data[offset + 1U] = (uint8_t)raw;

    return true;
}
//---------------初始化-------------------
/**
 * @brief 6020初始化
 *
 * @param obj
 * @param hcan
 * @param Motor_ID
 * @param Control_Method
 * @param Encoder_Offset
 * @param Omega_Max
 * @return true
 * @return false
 */
bool GM6020_Init(Struct_Motor_Manage_Obj *obj, FDCAN_HandleTypeDef *hcan, uint32_t Motor_ID, Enum_Control_Method Control_Method, int32_t Encoder_Offset, float Omega_Max)
{
    const Motor_Protocol_Config *protocol = Motor_GetProtocol(GM6020);
    Struct_Motor_Manage_Obj candidate = {0};

    // 检查参数是否合法
    if (obj == NULL || !Motor_IsHandleValid(hcan) || !Motor_IsControlMethodValid(Control_Method) || !isfinite(Omega_Max) || Omega_Max <= 0.0f)
    {
        return false;
    }

    candidate.Motor_Type = GM6020;
    candidate.hcan = hcan;
    candidate.Motor_ID = Motor_ID;
    candidate.Control_Method = Control_Method;

    candidate.Encoder_Offset = Encoder_Offset;
    candidate.Omega_Max = Omega_Max;
    candidate.Gearbox_Rate = GEARBOX_GATE_6020; // 减速比，需要确认！！！
    candidate.Torque_Max = TORQUE_MAX_6020;     // 最大扭矩

    if (protocol == NULL || !isfinite(candidate.Gearbox_Rate) ||
        candidate.Gearbox_Rate <= 0.0f)
    {
        return false;
    }
    candidate.Output_Max = protocol->Output_Limit;
    candidate.Encoder_Num_Per_Round = protocol->Encoder_Counts;

    candidate.CAN_Motor_Status = CAN_Motor_Status_DISABLE;
    candidate.Last_Control_Status = PID_STATUS_OK;

    if (!CAN_ID_Manage(&candidate))
    {
        return false;
    }

    *obj = candidate; // 初始化成功才替换对象。

    return true;
}

/**
 * @brief 用C6X0系电调的初始化
 *
 * @param obj
 * @param hcan
 * @param Motor_ID
 * @param Motor_Type
 * @param Control_Method
 * @param Gearbox_Rate 减速比
 * @param Torque_Max 最大扭矩
 * @return true
 * @return false
 */
bool Motor_C6X0_Init(Struct_Motor_Manage_Obj *obj, FDCAN_HandleTypeDef *hcan, uint32_t Motor_ID, Enum_Motor_Type Motor_Type, Enum_Control_Method Control_Method, float Gearbox_Rate, float Torque_Max)
{
    const Motor_Protocol_Config *protocol = Motor_GetProtocol(Motor_Type);
    Struct_Motor_Manage_Obj candidate = {0};

    if (obj == NULL || !Motor_IsHandleValid(hcan) || (Motor_Type != C610_M2006 && Motor_Type != C620_M3508) || !Motor_IsControlMethodValid(Control_Method) || !isfinite(Gearbox_Rate) || Gearbox_Rate <= 0.0f || !isfinite(Torque_Max) || Torque_Max <= 0.0f)
    {
        return false;
    }

    candidate.Motor_Type = Motor_Type;
    candidate.hcan = hcan;
    candidate.Motor_ID = Motor_ID;
    candidate.Control_Method = Control_Method;

    candidate.Gearbox_Rate = Gearbox_Rate;
    candidate.Torque_Max = Torque_Max;

    if (protocol == NULL)
    {
        return false;
    }
    candidate.Output_Max = protocol->Output_Limit;
    candidate.Encoder_Num_Per_Round = protocol->Encoder_Counts;

    candidate.CAN_Motor_Status = CAN_Motor_Status_DISABLE;
    candidate.Last_Control_Status = PID_STATUS_OK;

    if (!CAN_ID_Manage(&candidate))
    {
        return false;
    }

    *obj = candidate;

    return true;
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
    if (obj == NULL || !Motor_IsControlMethodValid(Control_Method))
    {
        return;
    }
    if (obj->Control_Method != Control_Method)
    {
        /* 新模式不能继承旧模式的积分历史和有效命令。 */
        obj->Output = 0.0f;
        obj->Command_Valid = false;
        PID_Reset(&obj->PID_Angle);
        PID_Reset(&obj->PID_Omega);
        PID_Reset(&obj->PID_Torque);
        obj->Control_Method = Control_Method;
    }
}

/**
 * @brief 设置目标的角度 rad
 *
 * @param obj
 * @param Target_Angle
 */
void Set_Target_Angle(Struct_Motor_Manage_Obj *obj, float Target_Angle)
{
    if (obj != NULL)
    {
        obj->Target_Angle = Target_Angle;
    }
}

/**
 * @brief 设置目标的速度 rad/s
 *
 * @param obj
 * @param Target_Omega
 */
void Set_Target_Omega(Struct_Motor_Manage_Obj *obj, float Target_Omega)
{
    if (obj != NULL)
    {
        obj->Target_Omega = Target_Omega;
    }
}

/**
 * @brief 设置目标的扭矩
 *
 * @param obj
 * @param Target_Torque
 */
void Set_Target_Torque(Struct_Motor_Manage_Obj *obj, float Target_Torque)
{
    if (obj != NULL)
    {
        obj->Target_Torque = Target_Torque;
    }
}

/**
 * @brief 设置输出量
 *
 * @param obj
 * @param Output
 */
void Set_Out(Struct_Motor_Manage_Obj *obj, float Output)
{
    if (obj == NULL)
    {
        return;
    }
    obj->Command_Valid = isfinite(Output) && obj->Has_Feedback &&
                        obj->CAN_Motor_Status == CAN_Motor_Status_ENABLE;
    obj->Output = obj->Command_Valid ? Output : 0.0f;
    obj->Last_Control_Status = isfinite(Output)
                                  ? PID_STATUS_OK : PID_STATUS_INVALID_ARGUMENT;
}

//--------------------在线检查和计算-------------

/**
 * @brief TIM定时器中断电机检查回调函数
 *
 * @param obj 电机管理结构体
 * @param now_ms 当前时间
 * @param timeout_ms 允许连续收不到反馈的最长时间
 */
void TIM_Status_PeriodElapsedCallback(Struct_Motor_Manage_Obj *obj, uint32_t now_ms, uint32_t timeout_ms)
{
    bool online;

    if (obj == NULL)
    {
        return;
    }

    /*
     * now_ms 与 Last_Rx_Ms 使用同一毫秒时基。
     * 无符号差值允许计时器正常回绕。
     */
    online =
        obj->Has_Feedback &&
        (uint32_t)(now_ms - obj->Last_Rx_Ms) <= timeout_ms;

    obj->CAN_Motor_Status = online ? CAN_Motor_Status_ENABLE : CAN_Motor_Status_DISABLE;

    if (!online)
    {
        obj->Output = 0.0f;
        obj->Command_Valid = false;

        PID_Reset(&obj->PID_Angle);
        PID_Reset(&obj->PID_Omega);
        PID_Reset(&obj->PID_Torque);
    }
}

/**
 * @brief 单电机控制计算
 *
 * @param obj
 * @param enabled
 * @param dt_s
 * @return PID_Status
 */
PID_Status TIM_PID_PeriodElapsedCallback(Struct_Motor_Manage_Obj *obj, bool enabled, float dt_s)
{
    PID_Status status = PID_STATUS_OK;
    float speed_target;

    if (obj == NULL)
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    obj->Output = 0.0f;         // 每次计算先撤销旧命令
    obj->Command_Valid = false; // 只有当前周期成功计算后，Command_Valid 才置 true。

    if (!enabled ||
        !obj->Has_Feedback ||
        obj->CAN_Motor_Status != CAN_Motor_Status_ENABLE)
    {
        PID_Reset(&obj->PID_Angle);
        PID_Reset(&obj->PID_Omega);
        PID_Reset(&obj->PID_Torque);

        obj->Last_Control_Status = PID_STATUS_OK;
        return PID_STATUS_OK;
    }

    if (!Motor_IsControlMethodValid(obj->Control_Method) ||
        Motor_GetProtocol(obj->Motor_Type) == NULL)
    {
        status = PID_STATUS_INVALID_ARGUMENT;
    }
    else if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        status = PID_STATUS_INVALID_DT;
    }
    else if ((int32_t)obj->Output_Max <= 0 ||
             (int32_t)obj->Output_Max >
                 (int32_t)Motor_GetProtocol(obj->Motor_Type)->Output_Limit)
    {
        status = PID_STATUS_INVALID_CONFIG;
    }
    else
    {
        switch (obj->Control_Method)
        {
        case Control_Method_OPENLOOP:
            if (obj->Motor_Type == GM6020)
            {
                if (!isfinite(obj->Target_Omega) ||
                    !isfinite(obj->Omega_Max) ||
                    obj->Omega_Max <= 0.0f)
                {
                    status = PID_STATUS_INVALID_ARGUMENT;
                    break;
                }

                obj->Output =
                    obj->Target_Omega / obj->Omega_Max *
                    (float)obj->Output_Max;
            }
            else
            {
                if (!isfinite(obj->Target_Torque) ||
                    !isfinite(obj->Torque_Max) ||
                    obj->Torque_Max <= 0.0f)
                {
                    status = PID_STATUS_INVALID_ARGUMENT;
                    break;
                }

                obj->Output =
                    obj->Target_Torque / obj->Torque_Max *
                    (float)obj->Output_Max;
            }
            break;

        case Control_Method_TORQUE:
            if (obj->Motor_Type == GM6020)
            {
                status = PID_Update(
                    &obj->PID_Torque,
                    obj->Target_Torque,
                    obj->Now_Torque,
                    dt_s,
                    &obj->Output);
            }
            else
            {
                /*
                 * 保留原 C6X0 的比例映射。
                 * 电调内部执行电流控制。
                 */
                if (!isfinite(obj->Target_Torque) || !isfinite(obj->Torque_Max) || obj->Torque_Max <= 0.0f)
                {
                    status = PID_STATUS_INVALID_ARGUMENT;
                    break;
                }

                obj->Output =
                    obj->Target_Torque / obj->Torque_Max *
                    (float)obj->Output_Max;
            }
            break;

        case Control_Method_OMEGA:
            status = PID_Update(
                &obj->PID_Omega,
                obj->Target_Omega,
                obj->Now_Omega,
                dt_s,
                &obj->Output);
            break;

        case Control_Method_ANGLE:
            /*
             * 角度环输出是目标速度 rad/s。
             * 角度 PID 的输出限幅也应使用速度单位。
             */
            status = PID_Update(
                &obj->PID_Angle,
                obj->Target_Angle,
                obj->Now_Angle,
                dt_s,
                &speed_target);

            if (status != PID_STATUS_OK)
            {
                break;
            }

            obj->Target_Omega = speed_target;

            status = PID_Update(
                &obj->PID_Omega,
                obj->Target_Omega,
                obj->Now_Omega,
                dt_s,
                &obj->Output);
            break;

        default:
            status = PID_STATUS_INVALID_ARGUMENT;
            break;
        }
    }

    if (status == PID_STATUS_OK && !isfinite(obj->Output))
    {
        status = PID_STATUS_NUM_ERROR;
    }

    if (status != PID_STATUS_OK)
    {
        obj->Output = 0.0f;

        PID_Reset(&obj->PID_Angle);
        PID_Reset(&obj->PID_Omega);
        PID_Reset(&obj->PID_Torque);
    }
    else
    {
        obj->Command_Valid = true;
    }

    obj->Last_Control_Status = status;

    return status;
}

//---------------发送------------------------------------
/**
 * @brief 发送组初始化
 *
 * @param group 发送组
 * @param hcan fdcan句柄
 * @param type  电机种类
 * @param command_id  发送报文的 ID
 * @param command_limit 主动设置的使用上限
 * @return true
 * @return false
 */
bool Motor_TxGroup_Init(Struct_Motor_Tx_Group *group, FDCAN_HandleTypeDef *hcan, Enum_Motor_Type type, uint16_t command_id, int16_t command_limit)
{
    Struct_Motor_Tx_Group candidate = {0};

    if (group == NULL ||
        !Motor_IsHandleValid(hcan) ||
        command_limit < 0)
    {
        return false;
    }

    candidate.Handle = hcan;
    candidate.Motor_Type = type;
    candidate.Command_ID = command_id;
    candidate.Command_Limit = command_limit;
    if (!Motor_IsGroupValid(&candidate))
    {
        return false;
    }

    *group = candidate;

    return true;
}

/**
 * @brief 把一台电机加入发送组，并放到它该占有的槽位上
 *
 * @param group 要加入的发送组
 * @param motor 要加入的电机对象
 * @return true
 * @return false
 */
bool Motor_TxGroup_Bind(Struct_Motor_Tx_Group *group, Struct_Motor_Manage_Obj *motor)
{
    const Motor_Protocol_Config *protocol;
    uint32_t slot;

    if (group == NULL ||
        motor == NULL ||
        !Motor_IsGroupValid(group) ||
        motor->hcan != group->Handle ||
        motor->Motor_Type != group->Motor_Type ||
        motor->CAN_Send_ID != group->Command_ID ||
        motor->Motor_ID < 1U)
    {
        return false;
    }

    protocol = Motor_GetProtocol(motor->Motor_Type);
    if (motor->Motor_ID > protocol->Max_ID ||
        motor->CAN_Send_ID !=
            (motor->Motor_ID <= MOTOR_TX_GROUP_SLOTS
                 ? protocol->Command_First : protocol->Command_Second) ||
        motor->CAN_Receive_ID != protocol->Feedback_Base + motor->Motor_ID)
    {
        return false;
    }

    /*
     * ID 1/5 -> 槽位 0
     * ID 2/6 -> 槽位 1
     * ID 3/7 -> 槽位 2
     * ID 4/8 -> 槽位 3
     */
    slot = (motor->Motor_ID - 1U) % MOTOR_TX_GROUP_SLOTS;

    if (group->Motors[slot] != NULL && group->Motors[slot] != motor)
    {
        return false;
    }

    group->Motors[slot] = motor;

    return true;
}

/**
 * @brief 把发送组内各台电机的指令汇总成一帧，再通过CAN发出去
 *
 * @param group 发送组
 * @param enabled 是否允许发送电机控制指令
 * @return HAL_StatusTypeDef 返回发送结果，例如HAL_OK或HAL_ERROR
 */
HAL_StatusTypeDef Motor_TxGroup_Send(const Struct_Motor_Tx_Group *group, bool enabled)
{
    const Motor_Protocol_Config *protocol;
    uint8_t data[MOTOR_CAN_DATA_BYTES] = {0};

    if (!Motor_IsGroupValid(group))
    {
        return HAL_ERROR;
    }
    protocol = Motor_GetProtocol(group->Motor_Type);

    if (enabled)
    {
        for (uint32_t slot = 0U; slot < MOTOR_TX_GROUP_SLOTS; ++slot)
        {
            if (group->Motors[slot] == NULL)
            {
                continue;
            }

            /* 绑定后对象若被重新初始化或改动，不得把命令发送到错误总线/槽位。 */
            const Struct_Motor_Manage_Obj *motor = group->Motors[slot];
            if (motor->hcan != group->Handle ||
                motor->Motor_Type != group->Motor_Type ||
                motor->Motor_ID == 0U || motor->Motor_ID > protocol->Max_ID ||
                motor->CAN_Send_ID != group->Command_ID ||
                motor->CAN_Send_ID !=
                    (motor->Motor_ID <= MOTOR_TX_GROUP_SLOTS
                         ? protocol->Command_First : protocol->Command_Second) ||
                motor->CAN_Receive_ID != protocol->Feedback_Base + motor->Motor_ID ||
                (motor->Motor_ID - 1U) % MOTOR_TX_GROUP_SLOTS != slot)
            {
                return HAL_ERROR;
            }

            if (!Output(
                    group->Motors[slot],
                    data,
                    group->Command_Limit))
            {
                return HAL_ERROR;
            }
        }
    }

    /*
     * 同组电机全部完成组帧后，只提交一次。
     * 失能或未绑定槽位保持零。
     */
    return CAN_STD_Send(
        group->Handle,
        group->Command_ID,
        data,
        sizeof(data));
}

//-----------------------反馈----------------------------------------
/**
 * @brief  解析电机反馈，并更新电机状态
 *
 * @param  obj          电机管理对象
 * @param  Rx_Data      CAN 接收数据，指向当前帧的数据区
 * @param  length       当前帧的数据长度，单位：字节
 * @param  rx_time_ms   当前帧接收时刻，单位：毫秒
 *
 * @retval true         反馈解析成功
 * @retval false        参数、配置或反馈数据不合法

 */
bool Motor_CAN_RxCpltCallback(Struct_Motor_Manage_Obj *obj, const uint8_t *Rx_Data, uint8_t length, uint32_t rx_time_ms)
{
    uint16_t encoder;
    uint16_t speed_raw;
    uint16_t current_raw;

    int16_t speed_rpm;
    int16_t current_feedback;

    int32_t delta_encoder;

    uint32_t counts_per_round;
    int32_t half_counts_per_round;

    if (obj == NULL || Rx_Data == NULL)
    {
        return false;
    }

    if (length != MOTOR_CAN_DATA_BYTES ||
        !Motor_IsHandleValid(obj->hcan) ||
        Motor_GetProtocol(obj->Motor_Type) == NULL)
    {
        return false;
    }

    counts_per_round = (uint32_t)obj->Encoder_Num_Per_Round;

    if (counts_per_round < 2U ||
        counts_per_round > (uint32_t)UINT16_MAX + 1U)
    {
        return false;
    }

    if (!isfinite(obj->Gearbox_Rate) ||
        obj->Gearbox_Rate <= 0.0f)
    {
        return false;
    }

    half_counts_per_round = (int32_t)(counts_per_round / 2U);

    encoder = (uint16_t)(((uint16_t)Rx_Data[0] << 8U) | (uint16_t)Rx_Data[1]);

    speed_raw = (uint16_t)(((uint16_t)Rx_Data[2] << 8U) | (uint16_t)Rx_Data[3]);

    current_raw = (uint16_t)(((uint16_t)Rx_Data[4] << 8U) | (uint16_t)Rx_Data[5]);

    if ((uint32_t)encoder >= counts_per_round)
    {
        return false;
    }

    speed_rpm = (int16_t)(speed_raw <= (uint16_t)INT16_MAX ? (int32_t)speed_raw : (int32_t)speed_raw - ((int32_t)UINT16_MAX + 1));

    current_feedback = (int16_t)(current_raw <= (uint16_t)INT16_MAX ? (int32_t)current_raw : (int32_t)current_raw - ((int32_t)UINT16_MAX + 1));

    // 建立第一帧基准，或者执行跨圈累计。
    if (!obj->Has_Feedback)
    {
        // 第一帧没有上一帧位置可供比较，从当前编码器位置开始建立历史。

        obj->Total_Round = 0;
        obj->Pre_Encoder = encoder;

        if (obj->Motor_Type == C610_M2006 || obj->Motor_Type == C620_M3508)
        {
            obj->Encoder_Offset = -(int32_t)encoder;
        }
    }
    else
    {
        /* 保存上一帧；Rx_Encoder 在本帧解码完成后才更新。 */
        obj->Pre_Encoder = obj->Rx_Encoder;
        // 必须先转换为有符号整数再相减，否则负的位置差可能被当作无符号数处理。
        delta_encoder = (int32_t)encoder - (int32_t)obj->Pre_Encoder;

        if (delta_encoder < -half_counts_per_round)
        {
            /*
             * 上一帧靠近最大值，当前帧靠近零：
             * 按正方向越过了编码器零点。
             */
            obj->Total_Round++;
        }
        else if (delta_encoder > half_counts_per_round)
        {
            /*
             * 上一帧靠近零，当前帧靠近最大值：
             * 按负方向越过了编码器零点。
             */
            obj->Total_Round--;
        }
    }

    obj->Rx_Encoder = encoder;
    obj->Rx_Omega = speed_rpm;
    obj->Rx_Torque = current_feedback;
    obj->Rx_Temperature = Rx_Data[6];

    obj->Total_Encoder = obj->Total_Round * (int64_t)counts_per_round + (int64_t)encoder + (int64_t)obj->Encoder_Offset;

    obj->Now_Angle = ((float)obj->Total_Encoder / (float)counts_per_round) * (2.0f * PI) / obj->Gearbox_Rate;

    obj->Now_Omega = (float)speed_rpm * RPM_TO_RADPS / obj->Gearbox_Rate; // 输出轴角速度

    obj->Now_Torque = (float)current_feedback; // 保存电流反馈原始值

    obj->Now_Temperature = obj->Rx_Temperature;

    // 更新反馈时间和有效标记。
    obj->Last_Rx_Ms = rx_time_ms;
    obj->Has_Feedback = true;

    return true;
}

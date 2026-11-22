#if !defined(DVC_MOTOR_H)
#define DVC_MOTOR_H

//-----------includes-------------------

#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "alg_pid.h"
#include "drv_can.h"

//-----------宏定义---------------------

#define RPM_TO_RADPS (2.0f * PI / 60.0f)

#define MOTOR_TX_GROUP_SLOTS 4U // 一组电机命令的槽位数量-一帧最多有几台电机槽位
#define MOTOR_CAN_DATA_BYTES 8U // 一帧电机CAN数据的字节数量

#define GEARBOX_GATE_6020 1.0f // 6020减速比
#define TORQUE_MAX_6020 1.2f   // 6020扭矩最大值

//-----------数组----------------
/**
 * @brief 电机状态
 *
 */
typedef enum
{
    CAN_Motor_Status_DISABLE = 0,
    CAN_Motor_Status_ENABLE,
} Enum_CAN_Motor_Status;

/**
 * @brief 电机类型
 *
 */
typedef enum
{
    GM6020 = 0,
    C610_M2006,
    C620_M3508,
} Enum_Motor_Type;

/**
 * @brief 一圈编码器计数
 *
 */
typedef enum
{
    Encoder_Num_Per_Round_6020 = 8192,
    Encoder_Num_Per_Round_2006 = 8192,
    Encoder_Num_Per_Round_3508 = 8192,
} Enum_Motor_Encoder_Num_Per_Round;

/**
 * @brief 最大输出电压
 *
 */
typedef enum
{
    Output_Max_6020 = 30000,
    Output_Max_2006 = 10000,
    Output_Max_3508 = 16384,
} Enum_Motor_Output_Max;

/**
 * @brief CAN电机的ID枚举类型
 *
 */
typedef enum
{
    CAN_Motor_ID_UNDEFINED = 0,
    CAN_Motor_ID_0x201,
    CAN_Motor_ID_0x202,
    CAN_Motor_ID_0x203,
    CAN_Motor_ID_0x204,
    CAN_Motor_ID_0x205,
    CAN_Motor_ID_0x206,
    CAN_Motor_ID_0x207,
    CAN_Motor_ID_0x208,
    CAN_Motor_ID_0x209,
    CAN_Motor_ID_0x20A,
    CAN_Motor_ID_0x20B,
} Enum_CAN_Motor_ID;

/**
 * @brief CAN电机的ID分配情况
 *
 */
typedef enum
{
    CAN_Motor_ID_Status_FREE = 0,
    CAN_Motor_ID_Status_ALLOCATED,
} Enum_CAN_Motor_ID_Status;

/**
 * @brief 电机控制方式
 *
 */
typedef enum
{
    Control_Method_OPENLOOP = 0, // 开环控制
    Control_Method_TORQUE,       // 扭矩环控制
    Control_Method_OMEGA,        // 速度环控制
    Control_Method_ANGLE,        // 角度环控制
} Enum_Control_Method;

/**
 * @brief 选择要配置的控制环。
 */
typedef enum
{
    MOTOR_PID_ANGLE = 0,
    MOTOR_PID_OMEGA,
    MOTOR_PID_TORQUE,

} Motor_PID_Loop;
//-----------结构体-------------------------

/**
 * @brief 电机管理结构体
 *
 * @note 存放电机管理相关参数和变量
 */
typedef struct
{
    Enum_Motor_Type Motor_Type;

    FDCAN_HandleTypeDef *hcan;

    PID_Controller PID_Angle;
    PID_Controller PID_Omega;
    PID_Controller PID_Torque;

    // 初始化相关变量
    int32_t Encoder_Offset; // 编码器漂移。给编码器读数加一个固定修正量，让软件定义的零点和实际机械零位对齐。可为负
    uint32_t Motor_ID;      // 电调编号（1、2……），不是反馈 CAN ID（0x201 等）
    uint32_t CAN_Send_ID;
    uint32_t CAN_Receive_ID;
    float Gearbox_Rate;                                     // 减速比
    float Omega_Max;                                        // 最大速度, 需根据不同负载测量后赋值, 6020开环输出使用
    float Torque_Max;                                       // 最大扭矩, 需根据不同负载测量后赋值, 2006和3508开环输出使用
    Enum_Motor_Encoder_Num_Per_Round Encoder_Num_Per_Round; // 一圈编码器刻度
    Enum_Motor_Output_Max Output_Max;                       // 协议命令幅值上限，可设置更低的使用上限

    // 状态
    Enum_CAN_Motor_Status CAN_Motor_Status;

    // flag
    bool Has_Feedback;   // 是否至少收到过1帧有效反馈
    uint32_t Last_Rx_Ms; // 最后一帧合法反馈的接收时间

    // 接收
    uint16_t Rx_Encoder;    // 接收位置，范围 0～Encoder_Num_Per_Round-1
    int16_t Rx_Omega;       // 接收的转速，单位 rpm，有符号（电机反转时为负）
    int16_t Rx_Torque;      // 接收的实际转矩电流，有符号
    uint8_t Rx_Temperature; // 接收的温度，单位摄氏度

    // 编码器
    uint16_t Pre_Encoder;  // 上一帧位置；首帧与 Rx_Encoder 相同
    int64_t Total_Encoder; // 总编码器位置（多圈累计，有符号）
    int64_t Total_Round;   // 总圈数（有符号，可以小于 0）

    // 电机控制方式
    Enum_Control_Method Control_Method;

    // 当前
    float Now_Angle;
    float Now_Omega;
    float Now_Torque;
    uint8_t Now_Temperature;

    // 目标
    float Target_Angle;
    float Target_Omega;
    float Target_Torque;

    // 输出
    float Output;

    bool Command_Valid; // 本周期命令有效标志

    PID_Status Last_Control_Status;

} Struct_Motor_Manage_Obj;

/**
 * @brief 一条电机组命令对应的软件对象
 *
 */
typedef struct
{
    FDCAN_HandleTypeDef *Handle;
    uint16_t Command_ID; // 发送报文的控制帧ID
    Enum_Motor_Type Motor_Type;

    Struct_Motor_Manage_Obj *Motors[MOTOR_TX_GROUP_SLOTS]; // 槽位分别对应哪个电机对象

    int16_t Command_Limit; // 限制值
} Struct_Motor_Tx_Group;

/**
 * @brief 串口绘图用的统一 float 快照。
 *
 * 12 个字段可以分别作为 12 条 FLOAT 通道。
 * 从同一个控制周期采集，再交给串口任务发送。
 */
typedef struct
{
    float target_angle;
    float measured_angle;

    float target_speed;
    float measured_speed;

    float command;
    float error;

    float p_output;
    float i_output;
    float d_output;
    float ff_output;

    float feedback_current_raw;
    float temperature;

} Motor_DebugSnapshot;

//-----------函数声明-----------------------

/* 初始化成功会清空整个电机对象；先初始化电机，再调用 PID_Init 配置各环。
 * 初始化/绑定在启动周期任务前完成，绑定后不要改变型号、编号或总线。
 */
bool GM6020_Init(Struct_Motor_Manage_Obj *obj, FDCAN_HandleTypeDef *hcan,
                 uint32_t Motor_ID, Enum_Control_Method Control_Method,
                 int32_t Encoder_Offset, float Omega_Max);
bool Motor_C6X0_Init(Struct_Motor_Manage_Obj *obj, FDCAN_HandleTypeDef *hcan,
                     uint32_t Motor_ID, Enum_Motor_Type Motor_Type,
                     Enum_Control_Method Control_Method,
                     float Gearbox_Rate, float Torque_Max);

/* 调用方按总线及反馈 ID 分发；反馈与控制读写必须由应用层串行化。
 * 相邻有效反馈之间的转子位移须小于半圈，否则无法正确累计整圈。
 */
bool Motor_CAN_RxCpltCallback(Struct_Motor_Manage_Obj *obj,
                              const uint8_t *Rx_Data, uint8_t length,
                              uint32_t rx_time_ms);

/* 周期顺序：更新在线状态 → 设置目标 → 控制计算 → 整组发送。
 * now_ms 与接收时间使用相同时基；dt_s 单位秒。
 * Command_Valid 没有命令超时功能，上层负责调度和命令看门狗。
 */
void TIM_Status_PeriodElapsedCallback(Struct_Motor_Manage_Obj *obj,
                                      uint32_t now_ms, uint32_t timeout_ms);
PID_Status TIM_PID_PeriodElapsedCallback(Struct_Motor_Manage_Obj *obj,
                                         bool enabled, float dt_s);

/* 一条总线的同一个命令 ID 只建立一个组，避免不同组互相覆盖命令。
 * Init 清空绑定；Bind 保存指针，对象必须长期有效。
 * 组限幅低于 PID 输出范围时，应同步调整 PID 配置，避免后级饱和积分。
 */
bool Motor_TxGroup_Init(Struct_Motor_Tx_Group *group, FDCAN_HandleTypeDef *hcan,
                        Enum_Motor_Type type, uint16_t command_id,
                        int16_t command_limit);
bool Motor_TxGroup_Bind(Struct_Motor_Tx_Group *group,
                        Struct_Motor_Manage_Obj *motor);
/* enabled=false 发送零帧；HAL_OK 仅代表入队。调用方必须处理错误返回。 */
HAL_StatusTypeDef Motor_TxGroup_Send(const Struct_Motor_Tx_Group *group,
                                     bool enabled);

/* 读取接口要求 obj 指向有效对象；角度 rad，角速度 rad/s。
 * Now_Torque 为电流反馈原始量，尚未换算为 N·m。
 * Get_Output 是计算值，最终发送还会经过组限幅。
 */
uint16_t Get_Output_Max(Struct_Motor_Manage_Obj *obj);
Enum_CAN_Motor_Status Get_CAN_Motor_Status(Struct_Motor_Manage_Obj *obj);
float Get_Now_Angle(Struct_Motor_Manage_Obj *obj);
float Get_Now_Omega(Struct_Motor_Manage_Obj *obj);
float Get_Now_Torque(Struct_Motor_Manage_Obj *obj);
uint8_t Get_Now_Temperature(Struct_Motor_Manage_Obj *obj);
Enum_Control_Method Get_Control_Method(Struct_Motor_Manage_Obj *obj);
float Get_Target_Angle(Struct_Motor_Manage_Obj *obj);
float Get_Target_Omega(Struct_Motor_Manage_Obj *obj);
float Get_Target_Torque(Struct_Motor_Manage_Obj *obj);
float Get_Output(Struct_Motor_Manage_Obj *obj);

/* 模式改变时清零命令并复位 PID 历史，保留目标和 PID 配置。 */
void Set_Control_Method(Struct_Motor_Manage_Obj *obj,
                         Enum_Control_Method Control_Method);
void Set_Target_Angle(Struct_Motor_Manage_Obj *obj, float Target_Angle);
void Set_Target_Omega(Struct_Motor_Manage_Obj *obj, float Target_Omega);
void Set_Target_Torque(Struct_Motor_Manage_Obj *obj, float Target_Torque);
/* 直接输出会被后续 PID 计算覆盖；仍受在线状态和发送组使能约束。 */
void Set_Out(Struct_Motor_Manage_Obj *obj, float Output);

/* Output() 为 .c 私有的组帧函数，不对外声明。
 * 绘图可继续使用 Motor_DebugSnapshot，以及 PID_Controller.pid_state 的诊断量。
 */

#endif // DVC_MOTOR_H

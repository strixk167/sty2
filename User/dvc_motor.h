#if !defined(DVC_MOTOR_H)
#define DVC_MOTOR_H

//-----------includes-------------------

#include "stm32h7xx_hal.h"
#include <string.h>
#include <math.h>
#include <stddef.h>

#include "alg_pid.h"
#include "drv_can.h"
#include "dvc_serialplot.h"

//---------------变量--------------------
int64_t Total_Round;   // 累计跨过编码器零点的次数
int64_t Total_Encoder; // 累计编码器计数，包括零点偏移

//-----------宏定义---------------------

#define RPM_TO_RADPS (2.0f * PI / 60.0f)

#define MOTOR_TX_GROUP_SLOTS 4U // 一组电机命令的槽位数量

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

//-----------结构体-------------------------

/**
 * @brief 电机管理结构体
 *
 * @note 存放电机管理相关参数和变量
 */
typedef struct
{
    Enum_Motor_Type Motor_Type;

    Struct_PID PID_Angle;
    Struct_PID PID_Omega;
    Struct_PID PID_Torque;

    // 初始化相关变量
    int32_t Encoder_Offset; // 编码器漂移。给编码器读数加一个固定修正量，让软件定义的零点和实际机械零位对齐。可为负
    uint32_t Motor_ID;      // Motor ID
    uint32_t CAN_Send_ID;
    uint32_t CAN_Receive_ID;
    float Gearbox_Rate;                                     // 减速比
    float Omega_Max;                                        // 最大速度, 需根据不同负载测量后赋值, 6020开环输出使用
    float Torque_Max;                                       // 最大扭矩, 需根据不同负载测量后赋值, 2006和3508开环输出使用
    Enum_Motor_Encoder_Num_Per_Round Encoder_Num_Per_Round; // 一圈编码器刻度
    Enum_Motor_Output_Max Output_Max;                       // 最大输出电压

    // 状态
    Enum_CAN_Motor_Status CAN_Motor_Status;

    // flag
    bool Has_Feedback;   // 是否至少收到过1帧有效反馈
    uint32_t Last_Rx_Ms; // 最后一帧合法反馈的接收时间

    // 接收
    uint16_t Rx_Encoder;    // 接收的编码器位置/机械角度，0~8191，无符号
    int16_t Rx_Omega;       // 接收的转速，单位 rpm，有符号（电机反转时为负）
    int16_t Rx_Torque;      // 接收的实际转矩电流，有符号
    uint8_t Rx_Temperature; // 接收的温度，单位摄氏度

    // 编码器
    uint16_t Pre_Encoder;  // 之前的编码器位置
    int32_t Total_Encoder; // 总编码器位置（多圈累计，有符号）
    int32_t Total_Round;   // 总圈数（有符号，可以小于 0）

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

} Struct_Motor_Manage_Obj;

/**
 * @brief 一条电机组命令对应的软件对象
 *
 */
typedef struct
{
    FDCAN_HandleTypeDef *Handle;
    uint16_t Command_ID;
    Enum_Motor_Type Motor_Tyoe;

    Struct_Motor_Manage_Obj *Motors[MOTOR_TX_GROUP_SLOTS];

    int16_t Command_Limit; // 限制值
};

//-----------函数声明-----------------------

/**
 * @brief GM6020初始化
 *
 * @param obj 电机对象
 * @param hcan CAN句柄
 * @param Motor_ID 电机ID
 * @param Control_Method 控制方式,
 *                       Control_Method_OPENLOOP开环控制,
 *                       Control_Method_TORQUE扭矩环控制,
 *                       Control_Method_OMEGA速度环控制,
 *                       Control_Method_ANGLE角度环控制
 * @param Encoder_Offset 编码器偏移
 * @param Omega_Max 最大速度
 */
void GM6020_Init(Struct_Motor_Manage_Obj *obj, CAN_HandleTypeDef *hcan, uint32_t Motor_ID, Enum_Control_Method Control_Method, int32_t Encoder_Offset, float Omega_Max);

/**
 * @brief C6X0电调支持的电机初始化
 *
 * @param obj 电机对象
 * @param hcan CAN句柄
 * @param Motor_ID 电机ID
 * @param Motor_Type 电机类型
 * @param Control_Method 控制方式,
 *                       Control_Method_OPENLOOP开环控制,
 *                       Control_Method_TORQUE扭矩环控制,
 *                       Control_Method_OMEGA速度环控制,
 *                       Control_Method_ANGLE角度环控制
 * @param Gearbox_Rate 减速比
 * @param Torque_Max 最大扭矩
 */
void Motor_C6X0_Init(Struct_Motor_Manage_Obj *obj, CAN_HandleTypeDef *hcan, uint32_t Motor_ID, Enum_Motor_Type Motor_Type, Enum_Control_Method Control_Method, float Gearbox_Rate, float Torque_Max);

/**
 * @brief 参数写入CAN发送列表
 *
 * @param obj 电机对象
 */
void Output(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取最大输出电压
 *
 * @param obj 电机对象
 * @return uint16_t 最大输出电压
 */
uint16_t Get_Output_Max(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取电机状态
 *
 * @param obj 电机对象
 * @return Enum_CAN_Motor_Status 电机状态
 */
Enum_CAN_Motor_Status Get_CAN_Motor_Status(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取当前角度 rad
 *
 * @param obj 电机对象
 * @return float 当前角度
 */
float Get_Now_Angle(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取当前速度 rad/s
 *
 * @param obj 电机对象
 * @return float 当前速度
 */
float Get_Now_Omega(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取当前扭矩
 *
 * @param obj 电机对象
 * @return float 当前扭矩
 */
float Get_Now_Torque(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取当前温度
 *
 * @param obj 电机对象
 * @return uint8_t 当前温度
 */
uint8_t Get_Now_Temperature(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取电机控制方式
 *
 * @param obj 电机对象
 * @return Enum_Control_Method 控制方式
 */
Enum_Control_Method Get_Control_Method(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取目标角度 rad
 *
 * @param obj 电机对象
 * @return float 目标角度
 */
float Get_Target_Angle(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取目标速度 rad/s
 *
 * @param obj 电机对象
 * @return float 目标速度
 */
float Get_Target_Omega(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取目标扭矩
 *
 * @param obj 电机对象
 * @return float 目标扭矩
 */
float Get_Target_Torque(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 获取输出量
 *
 * @param obj 电机对象
 * @return float 输出量
 */
float Get_Output(Struct_Motor_Manage_Obj *obj);

/**
 * @brief 设置电机控制方式
 *
 * @param obj 电机对象
 * @param Control_Method 控制方式
 */
void Set_Control_Method(Struct_Motor_Manage_Obj *obj, Enum_Control_Method Control_Method);

/**
 * @brief 设置目标角度 rad
 *
 * @param obj 电机对象
 * @param Target_Angle 目标角度
 */
void Set_Target_Angle(Struct_Motor_Manage_Obj *obj, float Target_Angle);

/**
 * @brief 设置目标速度 rad/s
 *
 * @param obj 电机对象
 * @param Target_Omega 目标速度
 */
void Set_Target_Omega(Struct_Motor_Manage_Obj *obj, float Target_Omega);

/**
 * @brief 设置目标扭矩
 *
 * @param obj 电机对象
 * @param Target_Torque 目标扭矩
 */
void Set_Target_Torque(Struct_Motor_Manage_Obj *obj, float Target_Torque);

/**
 * @brief 设置输出量
 *
 * @param obj 电机对象
 * @param Output 输出量
 */
void Set_Out(Struct_Motor_Manage_Obj *obj, float Output);

/**
 * @brief 电机CAN通信中断回调函数
 *
 * @param obj 电机对象
 * @param Rx_Data 接收数据
 */
void Motor_CAN_RxCpltCallback(Struct_Motor_Manage_Obj *obj, uint8_t *Rx_Data);

/**
 * @brief TIM定时器中断电机检查回调函数
 *
 * @param obj 电机对象
 */
void TIM_Status_PeriodElapsedCallback(Struct_Motor_Manage_Obj *obj);

/**
 * @brief TIM定时器中断计算回调函数
 *
 * @param obj 电机对象
 */
void TIM_PID_PeriodElapsedCallback(Struct_Motor_Manage_Obj *obj);

#endif // DVC_MOTOR_H

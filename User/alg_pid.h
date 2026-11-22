#if !defined(__ALG_PID_H__)
#define __ALG_PID_H__

//-----------Includes--------------
#include <stdbool.h>
#include "drv_math.h"

//-----------宏定义-----------------------
/**
 * @brief   前馈低通滤波系数默认值
 * @note    F_Filter 是"每个周期向新值靠拢的比例"，越大滤波越弱
 * @note    0.5 约等于控制周期 0.002s、截止频率 80Hz；目标值是方波时会削掉前馈尖峰
 */
#define PID_F_FILTER_DEFAULT (0.5f)

//-----------枚举，结构体-----------------

// 操作结构
/**
 * @brief PID操作结果
 *
 */
typedef enum
{
    PID_STATUS_OK = 0,

    PID_STATUS_INVALID_ARGUMENT, // 空指针或非法目标
    PID_STATUS_INVALID_CONFIG,   // 配置不合法
    PID_STATUS_NOT_INIT,         // 对象尚未初始化
    PID_STATUS_INVALID_DT,       // 时间间隔
    PID_STATUS_NUM_ERROR,        // 中间计算产生非有限数值
} PID_Status;

// 功能选项
/**
 * @brief D的模式
 *
 */
typedef enum
{
    PID_D_ON_ERROR = 0,   // 对误差求导，目标跳变会进入微分项，适合目标值平滑变化的
    PID_D_ON_MEASUREMENT, // 对测量值求导，适合目标值可能会阶跃变化的
} PID_D_Mode;

/**
 * @brief 死区模式
 *
 */
typedef enum
{
    PID_DEADZONE_ZERO_ERROR = 0, // 死区内仅将控制误差置0，已有积分、测量微分和前馈仍可能产生输出
    PID_DEADZONE_ZERO_OUTPUT,    // 死区内输出归零，同时清楚积分、微分滤波和前馈滤波状态
} PID_Deadzone_Mode;

/**
 * @brief 积分分离模式
 *
 */
typedef enum
{
    PID_I_SEPARATION_HOLD = 0, // 超过积分分离阈值时，保留积分但听组织新增积分
    PID_I_SEPARATION_CLEAR,    // 超过积分分离阈值时，清除积分
} PID_I_Separation_Mode;

// 配置
/**
 * @brief PID配置结构体
 *
 */
typedef struct
{
    float Kp;         // 比例项
    float Ki;         // 积分项
    float Kd;         // 微分项
    float Delta_Time; // PID计时周期，单位 s

    // 最终输出范围
    float Output_min;
    float Output_max;

    // 积分项贡献的输出范围
    float I_Min;
    float I_Max;

    // 死区
    float Deadzone; // 死区，误差小于该值时不输出
    PID_Deadzone_Mode Deadzone_Mode;

    // 变速积分
    // |error| <= full_error       全速积分
    // full_error < |error| < stop_error 积分倍率线性下降
    // |error| >= stop_error       停止新增积分
    bool Variable_I_Enable; // 是否开启
    float I_Full_Error;     // 误差不超过时积分按正常速度累加的值
    float I_Stop_Error;     // 误差达到时积分停止累计的值

    // 积分分离
    float I_Sep_Error; // 积分分离阈值
    PID_I_Separation_Mode I_Sep_Mode;

    // 微分先行选项开关
    bool D_First; // 0关1开

    // 条件积分抗饱和开关
    bool anti_windup; // 0关1开

    // 微分项模式配置
    PID_D_Mode D_Mode;

    // 前馈输出
    float feedforward_gain; // 按目标速度提供基础命令，目标稳定后可以持续输出
    float target_rate_gain; // 按目标加速度提供加速、减速补偿，目标稳定后不输出

    // 低通滤波参数，单位秒
    float measurement_filter_tau; // 滤反馈测量值，单位秒
    float d_filter_tau;           // 滤微分变化率，单位秒
    float feedforward_filter_tau; // 滤前馈输出，单位秒
    float output_filter_tau;      // 滤限幅后总输出，单位秒

    // 本次计算允许的时间间隔范围，固定时间就填一样的值
    float dt_min; // 下限，单位秒
    float dt_max; // 上限，单位秒
} PID_Config;

/**
 * @brief PID动态状态
 *
 */
typedef struct
{
    bool has_history; // 是否有有效的历史数据，标记控制器是否已经跑过至少1个有效周期

    float filtered_measurement; // 滤波后的反馈值
    float pre_error;            // 上一周期误差
    float pre_target;           // 上一周期目标设定值

    float i_out; // 积分项输出，已经乘过i

    float filtered_d;           // 滤波后的积分项
    float filtered_feedforward; // 滤波后的前馈量
    float last_out;             // 上一周期PID总输出

    // 诊断量，用于遥测和调试
    float error;  // 当前周期误差
    float p_out;  // 比例项输出
    float d_out;  // 微分项输出
    float ff_out; // 前馈项输出FF
} PID_State;

/**
 * @brief PID管理结构体
 *
 */
typedef struct
{
    PID_Config pid_config;
    PID_State pid_state;
    bool init_ok;
} PID_Controller;

//-----------函数声明---------------------

/**
 * @brief 校验并复制配置，清空动态状态。
 * @return PID_STATUS_OK 表示初始化成功。
 */
PID_Status PID_Init(PID_Controller *pid, const PID_Config *config);

/**
 * @brief 清空积分、差分和滤波历史，保留配置。
 */
void PID_Reset(PID_Controller *pid);

/**
 * @brief 使用本周期目标、反馈和时间间隔计算输出。
 * @param output 指向控制器对象之外的有效输出变量。
 * @return PID状态码，计算失败时有效的output写入零。
 */
PID_Status PID_Update(PID_Controller *pid, float target, float measurement,
                      float dt, float *output);

#endif // __ALG_PID_H__

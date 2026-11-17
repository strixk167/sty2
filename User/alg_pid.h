#if !defined(__ALG_PID_H__)
#define __ALG_PID_H__

//-----------Includes--------------
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
    float I_Sep_Error;
    PID_I_Separation_Mode I_Sep_Mode;

    // 微分先行选项开关
    bool D_First; // 0关1开

    // 条件积分抗饱和开关
    bool anti_windup; // 0关1开

    // 微分项模式配置
    PID_D_Mode D_Mode;

    // 前馈输出
    float feedforward_gain;   // 按目标速度提供基础命令，目标稳定后可以持续输出
    float fetpoint_rate_gain; // 按目标加速度提供加速、减速补偿，目标稳定后不输出

    // 低通滤波参数，单位秒
    float measurement_filter_tau; // 滤反馈测量值，单位秒
    float derivative_filter_tau;  // 滤微分变化率，单位秒
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
    float pre_setpoint;         // 上一周期目标设定值

    float intergral_out; // 积分项输出，已经乘过i

    float filtered_derivative;  // 滤波后的积分项
    float filtered_feedforward; // 滤波后的前馈量
    float last_out;             // 上一周期PID总输出

    // 诊断量，用于遥测和调试
    float error;  // 当前周期误差
    float p_out;  // 比例项输出
    float d_out;  // 微分项输出
    float ff_out; // 前馈项输出FF
} PID_State;

typedef struct
{
    PID_Config pid_config;
    PID_State pid_state;
    bool init_ok;
} PID_Controller;

//-----------函数声明---------------------
/**
 * @brief   PID 初始化
 * @param   pid  PID 对象
 * @param   Kp  P 参数
 * @param   Ki  I 参数
 * @param   Kd  D 参数
 * @param   Delta_Time  PID计时周期，单位 s
 * @retval  None
 */
void PID_Init(Struct_PID *pid, float Kp, float Ki, float Kd, float Delta_Time);

/**
 * @brief   获取积分值
 * @param   pid PID对象
 * @retval  float 积分值，误差对时间的积分
 */
float PID_Get_Integral_Error(Struct_PID *pid);

/**
 * @brief   获取输出值
 * @param   pid PID 对象
 * @retval  float 输出值
 */
float PID_Get_Out(Struct_PID *pid);

/**
 * @brief   设置 PID 的 P
 * @param   pid PID 对象
 * @param   Kp P 参数
 * @retval  None
 */
void PID_Set_Kp(Struct_PID *pid, float Kp);

/**
 * @brief   设置 PID 的 I
 * @param   pid PID 对象
 * @param   Ki I 参数
 * @retval  None
 */
void PID_Set_Ki(Struct_PID *pid, float Ki);

/**
 * @brief   设置 PID 的 D
 * @param   pid PID 对象
 * @param   Kd D 参数
 * @retval  None
 */
void PID_Set_Kd(Struct_PID *pid, float Kd);

/**
 * @brief   设置前馈
 * @param   pid PID 对象
 * @param   K_F 前馈参数
 * @retval  None
 */
void PID_Set_K_F(Struct_PID *pid, float K_F);

/**
 * @brief   获取前馈参数
 * @param   pid PID 对象
 * @retval  float 前馈参数
 */
float PID_Get_K_F(Struct_PID *pid);

/**
 * @brief   设置前馈低通滤波系数
 * @param   pid PID 对象
 * @param   F_Filter 滤波系数，0~1。0 为不滤波，越大滤波越弱
 * @retval  None
 */
void PID_Set_F_Filter(Struct_PID *pid, float F_Filter);

/**
 * @brief   设置积分限幅，0 为不限制
 * @param   pid PID 对象
 * @param   I_Out_Max 积分限幅
 * @retval  None
 */
void PID_Set_I_Out_Max(Struct_PID *pid, float I_Out_Max);

/**
 * @brief   设置输出限幅，0 为不限制
 * @param   pid PID 对象
 * @param   Out_Max 输出限幅
 * @retval  None
 */
void PID_Set_Out_Max(Struct_PID *pid, float Out_Max);

/**
 * @brief   设置变速积分定速内段阈值，0 为不限制
 * @param   pid PID 对象
 * @param   I_Variable_Speed_A 定速内段阈值
 * @retval  None
 */
void PID_Set_I_Variable_Speed_A(Struct_PID *pid, float I_Variable_Speed_A);

/**
 * @brief   设置变速积分变速区间，0 为不限制
 * @param   pid PID 对象
 * @param   I_Variable_Speed_B 变速区间
 * @retval  None
 */
void PID_Set_I_Variable_Speed_B(Struct_PID *pid, float I_Variable_Speed_B);

/**
 * @brief   设置积分分离阈值，需为正数，0 为不限制
 * @param   pid PID 对象
 * @param   I_Separate_Threshold 积分分离阈值
 * @retval  None
 */
void PID_Set_I_Separate_Threshold(Struct_PID *pid, float I_Separate_Threshold);

/**
 * @brief   设置微分先行选项
 * @param   pid PID 对象
 * @param   D_First 微分先行选项
 * @retval  None
 */
void PID_Set_D_First(Struct_PID *pid, Enum_PID_D_First D_First);

/**
 * @brief   设置死区
 * @param   pid PID 对象
 * @param   Dead_Zone 死区大小
 * @retval  None
 */
void PID_Set_Dead_Zone(Struct_PID *pid, float Dead_Zone);

/**
 * @brief   设置目标值
 * @param   pid PID 对象
 * @param   Target 目标值
 * @retval  None
 */
void PID_Set_Target(Struct_PID *pid, float Target);

/**
 * @brief   设置当前值
 * @param   pid PID 对象
 * @param   Now 当前值
 * @retval  None
 */
void PID_Set_Now(Struct_PID *pid, float Now);

/**
 * @brief   设置积分值，一般用于积分清零
 * @param   pid PID 对象
 * @param   Integral_Error 积分值
 * @retval  None
 */
void PID_Set_Integral_Error(Struct_PID *pid, float Integral_Error);

/**
 * @brief   PID 周期调整回调
 * @param   pid PID 对象
 * @retval  无
 * @note    用于计算PID
 * @note    需在固定周期中断或主循环中周期性调用
 */
void PID_TIM_Adjust_PeriodElapsedCallback(Struct_PID *pid);

#endif // __ALG_PID_H__

#include "alg_pid.h"
#include <string.h>

//-----------函数------------

// 内部函数
/**
 * @brief 限值
 *
 * @param value 输入值
 * @param min 下限
 * @param max 上限
 * @return float 限制后输出结果
 */
static float PID_Clamp(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    return value;
}

/**
 * @brief 一阶低通
 * @note
 *      alpha = dt / (tau + dt)
 *      tau 为零时直接返回输入。
 *      调用方保证 tau >= 0、dt > 0。
 * @param pre 上一次滤波后的值
 * @param input 本次刚测到或算出的原始值
 * @param tau 滤波时间常数
 * @param dt 这次更新与上次更新之间的时间间隔
 * @return float 滤波后的新数值
 */
static float PID_LowPass(float pre, float input, float tau, float dt)
{
    float denominator; // 分母
    float pre_weight;
    float input_weight;

    if (tau == 0.0f)
    {
        return input;
    }

    denominator = tau + dt;

    if (!isfinite(denominator)) // 判断是不是一个正常的有限数
    {
        return NAN; // Not A Number
    }

    pre_weight = tau / denominator;
    input_weight = dt / denominator;

    return pre_weight * pre + input_weight * input;
}

/**
 * @brief 审查参数是否合法
 *
 * @param config PID_config结构体
 * @return true
 * @return false
 */
static bool PID_IsConfigValid(const PID_Config *config)
{
    if (config == NULL)
    {
        return false;
    }

    // 检查参数是否合法
    if (!isfinite(config->Kp) || !isfinite(config->Ki) || !isfinite(config->Kd) || config->Kp < 0.0f || config->Ki < 0.0f || config->Kd < 0.0f)
    {
        return false;
    }

    if (!isfinite(config->Output_min) || !isfinite(config->Output_max) || config->Output_min >= config->Output_max)
    {
        return false;
    }

    if (!isfinite(config->I_Min) || !isfinite(config->I_Max) || config->I_Min > 0.0f || config->I_Max > 0.0f || config->I_Min >= config->I_Max)
    {
        return false;
    }

    if (!isfinite(config->Deadzone) || config->Deadzone < 0.0f || (config->Deadzone_Mode != PID_DEADZONE_ZERO_ERROR && config->Deadzone_Mode != PID_DEADZONE_ZERO_OUTPUT))
    {
        return false;
    }

    // 强制零输出模式(触发某个条件时PID直接把本次输出设为0)要求输出范围包括0
    if (config->Deadzone_Mode == PID_DEADZONE_ZERO_OUTPUT && (config->Output_min > 0.0f || config->Output_max < 0.0f))
    {
        return false;
    }

    if (!isfinite(config->I_Full_Error) || !isfinite(config->I_Stop_Error) || config->I_Full_Error < 0.0f || config->I_Stop_Error < 0.0f)
    {
        return false;
    }

    if (config->Variable_I_Enable && config->I_Stop_Error <= config->I_Full_Error)
    {
        return false;
    }

    if (!isfinite(config->I_Sep_Error) || config->I_Sep_Error < 0.0f || (config->I_Sep_Mode != PID_I_SEPARATION_HOLD && config->I_Sep_Mode != PID_I_SEPARATION_CLEAR))
    {
        return false;
    }

    if (config->D_Mode != PID_D_ON_ERROR && config->D_Mode != PID_D_ON_MEASUREMENT)
    {
        return false;
    }

    if (!isfinite(config->measurement_filter_tau) || !isfinite(config->derivative_filter_tau) || !isfinite(config->feedforward_filter_tau) || !isfinite(config->output_filter_tau) || config->measurement_filter_tau < 0.0f || config->derivative_filter_tau < 0.0f || config->feedforward_filter_tau < 0.0f || config->output_filter_tau < 0.0f)
    {
        return false;
    }

    if (!isfinite(config->dt_min) || !isfinite(config->dt_max) || config->dt_min <= 0.0f || config->dt_max < config->dt_min)
    {
        return false;
    }

    return true; // 所有参数合法！
}

/**
 * @brief 计算本周期变速积分倍率
 *
 * @param config PID_Config 结构体
 * @param abs_error 误差的绝对值
 * @return float
 */
static float PID_I_Ratio(const PID_Config *config, float abs_error)
{
    if (!config->Variable_I_Enable) // 没有启用变速积分
    {
        return 1; // 积分倍率为100%
    }

    if (abs_error <= config->I_Full_Error) // 误差处于积分全速区间
    {
        return 1.0f;
    }

    if (abs_error >= config->I_Stop_Error) // 误差大于停止积分累加
    {
        return 0.0f;
    }

    return (config->I_Stop_Error - abs_error) / (config->I_Stop_Error - config->I_Full_Error);
}

// 函数
PID_Status PID_Init(PID_Controller *pid, const PID_Config *config)
{
    PID_Controller candidate = {0}; // 先用临时对象，等到核验没问题了再赋值

    if (pid == NULL || config == NULL)
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    if (!PID_IsConfigValid(config))
    {
        return PID_STATUS_INVALID_CONFIG;
    }

    // 核验完成开始赋值
}

/**
 * @brief   获取积分值
 * @param   pid PID对象
 * @retval  float 积分值，误差对时间的积分
 */
float PID_Get_Integral_Error(Struct_PID *pid)
{
    return (pid->Integral_Error);
}

/**
 * @brief   获取输出值
 * @param   pid PID 对象
 * @retval  float 输出值
 */
float PID_Get_Out(Struct_PID *pid)
{
    return (pid->Out);
}

/**
 * @brief   设置 PID 的 P
 * @param   pid PID 对象
 * @param   Kp P 参数
 * @retval  None
 */
void PID_Set_Kp(Struct_PID *pid, float Kp)
{
    pid->Kp = Kp;
}

/**
 * @brief   设置 PID 的 I
 * @param   pid PID 对象
 * @param   Ki I 参数
 * @retval  None
 */
void PID_Set_Ki(Struct_PID *pid, float Ki)
{
    pid->Ki = Ki;
}

/**
 * @brief   设置 PID 的 D
 * @param   pid PID 对象
 * @param   Kd D 参数
 * @retval  None
 */
void PID_Set_Kd(Struct_PID *pid, float Kd)
{
    pid->Kd = Kd;
}

/**
 * @brief   设置前馈
 * @param   pid PID 对象
 * @param   K_F 前馈参数
 * @retval  None
 */
void PID_Set_K_F(Struct_PID *pid, float K_F)
{
    pid->K_F = K_F;
}

/**
 * @brief   获取前馈参数
 * @param   pid PID 对象
 * @retval  float 前馈参数
 */
float PID_Get_K_F(Struct_PID *pid)
{
    return (pid->K_F);
}

/**
 * @brief   设置前馈低通滤波系数
 * @param   pid PID 对象
 * @param   F_Filter 滤波系数，0~1。0 为不滤波，越大滤波越弱
 * @retval  None
 */
void PID_Set_F_Filter(Struct_PID *pid, float F_Filter)
{
    pid->F_Filter = F_Filter;
}

/**
 * @brief   设置积分限幅，0 为不限制
 * @param   pid PID 对象
 * @param   I_Out_Max 积分限幅
 * @retval  None
 */
void PID_Set_I_Out_Max(Struct_PID *pid, float I_Out_Max)
{
    pid->I_Out_Max = I_Out_Max;
}

/**
 * @brief   设置输出限幅，0 为不限制
 * @param   pid PID 对象
 * @param   Out_Max 输出限幅
 * @retval  None
 */
void PID_Set_Out_Max(Struct_PID *pid, float Out_Max)
{
    pid->Out_Max = Out_Max;
}

/**
 * @brief   设置变速积分定速内段阈值，0 为不限制
 * @param   pid PID 对象
 * @param   I_Variable_Speed_A 定速内段阈值
 * @retval  None
 */
void PID_Set_I_Variable_Speed_A(Struct_PID *pid, float I_Variable_Speed_A)
{
    pid->I_Variable_Speed_A = I_Variable_Speed_A;
}

/**
 * @brief   设置变速积分变速区间，0 为不限制
 * @param   pid PID 对象
 * @param   I_Variable_Speed_B 变速区间
 * @retval  None
 */
void PID_Set_I_Variable_Speed_B(Struct_PID *pid, float I_Variable_Speed_B)
{
    pid->I_Variable_Speed_B = I_Variable_Speed_B;
}

/**
 * @brief   设置积分分离阈值，需为正数，0 为不限制
 * @param   pid PID 对象
 * @param   I_Separate_Threshold 积分分离阈值
 * @retval  None
 */
void PID_Set_I_Separate_Threshold(Struct_PID *pid, float I_Separate_Threshold)
{
    pid->I_Separate_Threshold = I_Separate_Threshold;
}

/**
 * @brief   设置微分先行选项
 * @param   pid PID 对象
 * @param   D_First 微分先行选项
 * @retval  None
 */
void PID_Set_D_First(Struct_PID *pid, Enum_PID_D_First D_First)
{
    pid->D_First = D_First;
}

/**
 * @brief   设置死区
 * @param   pid PID 对象
 * @param   Dead_Zone 死区大小
 * @retval  None
 */
void PID_Set_Dead_Zone(Struct_PID *pid, float Dead_Zone)
{
    pid->Dead_Zone = Dead_Zone;
}

/**
 * @brief   设置目标值
 * @param   pid PID 对象
 * @param   Target 目标值
 * @retval  None
 */
void PID_Set_Target(Struct_PID *pid, float Target)
{
    pid->Target = Target;
}

/**
 * @brief   设置当前值
 * @param   pid 一个结构体，里面有PID计算相关参数
 * @param   Now 当前值
 * @retval  None
 */
void PID_Set_Now(Struct_PID *pid, float Now)
{
    pid->Now = Now;
}

/**
 * @brief   设置积分值，一般用于积分清零
 * @param   pid PID 对象
 * @param   Integral_Error 积分值
 * @retval  None
 */
void PID_Set_Integral_Error(Struct_PID *pid, float Integral_Error)
{
    pid->Integral_Error = Integral_Error;
}

/**
 * @brief   PID 周期调整回调
 * @param   pid PID 对象
 * @retval  无
 * @note    用于计算PID
 * @note    需在固定周期中断或主循环中周期性调用
 */
void PID_TIM_Adjust_PeriodElapsedCallback(Struct_PID *pid)
{
    float p_out = 0.0f;
    float i_out = 0.0f;
    float d_out = 0.0f;
    float f_out = 0.0f; // 前馈项输出
    float error;        // 误差
    float abs_error;    // 误差的绝对值
    float speed_ratio;  // 倍数，用于线性变速积分

    error = pid->Target - pid->Now;
    abs_error = Math_Abs(error);

    if (abs_error < pid->Dead_Zone) // 判断死区。当误差绝对值小于死区阈值时，直接把目标值强制等于当前实际值，误差清零，让 PID 停止输出，避免电机在目标附近来回抖动
    {
        pid->Target = pid->Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    p_out = pid->Kp * error; // 计算p项

    // 计算I项
    // 线性变速积分
    if (pid->I_Variable_Speed_A == 0.0f && pid->I_Variable_Speed_B == 0.0f) // 不限
    {
        speed_ratio = 1.0f; // 非变速积分
    }
    else
    {
        // 变速积分
        if (abs_error <= pid->I_Variable_Speed_A)
        {
            speed_ratio = 1.0f;
        }
        else if (abs_error < pid->I_Variable_Speed_A + pid->I_Variable_Speed_B)
        {
            speed_ratio = 1.0f - (abs_error - pid->I_Variable_Speed_A) / pid->I_Variable_Speed_B;
        }
        else
        {
            speed_ratio = 0.0f;
        }
    }
    // 积分限幅
    // 注意必须判 Ki：I_Out_Max / Ki 在 Ki=0 时是除零，会得到 inf/nan 并污染整个输出
    if (pid->I_Out_Max != 0.0f && pid->Ki != 0.0f) // 有限幅且 Ki 非零
    {
        Math_Constrain(&pid->Integral_Error, -pid->I_Out_Max / pid->Ki, pid->I_Out_Max / pid->Ki);
    }
    // 积分分离：当误差很大时，先把积分项停掉，避免积分值积累过多；等误差变小了，再重新开启积分，用来消除稳态误差
    if (pid->I_Separate_Threshold == 0.0f)
    {
        // 没有积分分离
        pid->Integral_Error += speed_ratio * pid->Delta_Time * error;
        i_out = pid->Ki * pid->Integral_Error;
    }
    else
    {
        // 有积分分离
        if (abs_error < pid->I_Separate_Threshold)
        {
            pid->Integral_Error += speed_ratio * pid->Delta_Time * error;
            i_out = pid->Ki * pid->Integral_Error;
        }
        else
        {
            pid->Integral_Error = 0.0f; // 停掉积分项
            i_out = 0.0f;
        }
    }

    // 计算D项
    if (pid->D_First == PID_D_First_DISABLE) // 没有微分先行，对误差求导
    {
        d_out = pid->Kd * (error - pid->Pre_Error) / pid->Delta_Time;
    }
    else
    {
        // 有微分先行，对当前值求导
        d_out = -pid->Kd * (pid->Now - pid->Pre_Now) / pid->Delta_Time;
    }

    // 计算前馈：不等误差出现，直接按目标值的"变化速率"提前给输出
    // 注意必须除以 Delta_Time：目标速度 = (本次目标 - 上次目标) / 控制周期
    // 少了这个除法，前馈量纲是"每周期角度"而不是"角速度"，会随控制频率漂移
    // 再经过低通滤波，避免目标跳变时前馈出尖峰
    {
        float target_vel = (pid->Target - pid->Pre_Target) / pid->Delta_Time; // 目标速度
        float f_raw = pid->K_F * target_vel;                                  // 未滤波的前馈输出

        if (pid->F_Filter <= 0.0f) // 系数为 0 表示不滤波
        {
            pid->Pre_F_Out = f_raw;
        }
        else
        {
            pid->Pre_F_Out += pid->F_Filter * (f_raw - pid->Pre_F_Out);
        }
        f_out = pid->Pre_F_Out;
    }

    // 计算总输出
    pid->Out = p_out + i_out + d_out + f_out;

    // 输出限幅
    if (pid->Out_Max != 0.0f)
    {
        Math_Constrain(&pid->Out, -pid->Out_Max, pid->Out_Max);
    }

    // 更新历史值
    pid->Pre_Now = pid->Now;
    pid->Pre_Target = pid->Target;
    pid->Pre_Out = pid->Out;
    pid->Pre_Error = error;
}

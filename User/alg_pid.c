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

    if (!isfinite(config->I_Min) || !isfinite(config->I_Max) || config->I_Min > 0.0f || config->I_Max < 0.0f || config->I_Min > config->I_Max)
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

    if (!isfinite(config->feedforward_gain) || !isfinite(config->target_rate_gain))
    {
        return false;
    }

    if (!isfinite(config->measurement_filter_tau) || !isfinite(config->d_filter_tau) || !isfinite(config->feedforward_filter_tau) || !isfinite(config->output_filter_tau) || config->measurement_filter_tau < 0.0f || config->d_filter_tau < 0.0f || config->feedforward_filter_tau < 0.0f || config->output_filter_tau < 0.0f)
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
 * @return float 计算结果
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
/**
 * @brief PID初始化
 *
 * @param pid PID管理结构体
 * @param config PID配置
 * @return PID_Status 初始化情况
 */
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
    candidate.pid_config = *config;
    candidate.init_ok = true;

    *pid = candidate;
    return PID_STATUS_OK;
}

/**
 * @brief 复位函数
 *
 * @param pid PID控制结构体
 */
void PID_Reset(PID_Controller *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->pid_state = (PID_State){0}; // 只清运行在状态，增益、限幅、死区、滤波和初始化标志保持有效
}

/**
 * @brief PID更新
 *
 * @param pid PID控制结构体
 * @param target 目标值
 * @param measurement 当前反馈值
 * @param dt 本次PID计算与上一次计算之间的时间间隔，单位是秒
 * @param output 输出结果(的地址)。PID计算完成后，函数通过它把结果写回调用者
 * @return PID_Status
 */
PID_Status PID_Update(PID_Controller *pid, float target, float measurement, float dt, float *output)
{
    const PID_Config *config;   // 指向当前控制器的配置，只读
    PID_State next_state;       // 下一周期状态
    float measurement_filtered; // 滤波后的测量值
    float error_raw;            // 原始误差
    float error;                // 实际用于PID计算的误差，可选死区处理
    float error_abs;            // 误差绝对值

    float p_output; // 比例项输出

    float d_raw;      // 原始微分输入
    float d_filtered; // 经过微分滤波后的微分输入
    float d_out;      // 微分项输出

    float target_rate; // 目标值变化率
    float ff_raw;      // 未滤波的前馈输出
    float ff_filtered; // 滤波后的前馈输出

    float i_base;      // 历史积分累加值
    float i_ratio;     // 当前误差下允许积分的比例
    float i_delta;     // 本周期理论上新增的积分量
    float i_candidate; // 暂时算出的积分结果，还可能没有经过积分限幅或抗饱和处理
    float i_change;    // 实际积分变化量

    float output_candidate; // 各项相加的初始输出，还没有经过输出限幅和输出滤波
    float output_raw;       // 准备进入输出处理链的原始控制量，是output_candidate经过抗饱和和修正正后的结果
    float output_limited;   // 经过输出上下限限制后的输出
    float output_filtered;  // 滤波后的结果

    bool in_deadzone; // 表示当前原始误差是否处于死区内，用于记录诊断状态
    bool i_sep;       // 表示当前是否触发积分分离

    // 输入和配置检查
    if (output == NULL) // 检查是否传入了空指针
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    *output = 0.0f;

    if (pid == NULL)
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    if (!pid->init_ok)
    {
        return PID_STATUS_NOT_INIT;
    }

    if (!isfinite(target) || !isfinite(measurement))
    {
        return PID_STATUS_INVALID_ARGUMENT;
    }

    config = &pid->pid_config;

    if (!PID_IsConfigValid(config))
    {
        return PID_STATUS_INVALID_CONFIG;
    }

    if (!isfinite(dt) ||
        dt < config->dt_min ||
        dt > config->dt_max)
    {
        return PID_STATUS_INVALID_DT;
    }

    next_state = pid->pid_state;

    // 测量滤波
    // 第一次调用直接用实际反馈值建立初始基准 后续调用上一周期滤波值 + 当前测量值=得到平滑反馈
    if (next_state.has_history)
    {
        measurement_filtered = PID_LowPass(next_state.filtered_measurement, measurement, config->measurement_filter_tau, dt);
    }
    else
    {
        measurement_filtered = measurement;
    }

    error_raw = target - measurement_filtered;

    if (!isfinite(measurement_filtered) || !isfinite(error_raw)) // 检查计算结果是否为有限的正常浮点数
    {
        return PID_STATUS_NUM_ERROR;
    }

    // 死区

    in_deadzone = config->Deadzone > 0.0f && fabsf(error_raw) <= config->Deadzone; // 判断当前误差是否进入死区
    error = in_deadzone ? 0.0f : error_raw;
    error_abs = fabsf(error);

    if (in_deadzone &&
        config->Deadzone_Mode == PID_DEADZONE_ZERO_OUTPUT)
    {
        next_state.has_history = true;

        next_state.filtered_measurement = measurement_filtered;
        next_state.pre_error = 0.0f;
        next_state.pre_target = target;

        next_state.i_out = 0.0f;
        next_state.filtered_d = 0.0f;
        next_state.filtered_feedforward = 0.0f;

        next_state.last_out = 0.0f;
        next_state.error = 0.0f;
        next_state.p_out = 0.0f;
        next_state.d_out = 0.0f;
        next_state.ff_out = 0.0f;

        pid->pid_state = next_state;

        return PID_STATUS_OK;
    }

    // 比例项
    p_output = config->Kp * error;

    if (!isfinite(p_output))
    {
        return PID_STATUS_NUM_ERROR;
    }

    // 微分与微分滤波
    d_raw = 0.0f;

    if (next_state.has_history && config->Kd > 0.0f)
    {
        if (config->D_Mode == PID_D_ON_MEASUREMENT)
        {
            d_raw = -(measurement_filtered - next_state.filtered_measurement) / dt;
        }
        else
        {
            d_raw = (error - next_state.pre_error) / dt;
        }
    }

    if (!isfinite(d_raw))
    {
        return PID_STATUS_NUM_ERROR;
    }

    d_filtered = PID_LowPass(next_state.filtered_d, d_raw, config->d_filter_tau, dt);

    d_out = config->Kd * d_filtered;

    if (!isfinite(d_filtered) || !isfinite(d_out))
    {
        return PID_STATUS_NUM_ERROR;
    }

    // 前馈与前馈滤波：保留目标，首次计算的目标变化率为零。
    target_rate = 0.0f;

    if (next_state.has_history && config->target_rate_gain != 0.0f)
    {
        target_rate = (target - next_state.pre_target) / dt;
    }

    ff_raw = config->feedforward_gain * target + config->target_rate_gain * target_rate;

    if (!isfinite(target_rate) || !isfinite(ff_raw))
    {
        return PID_STATUS_NUM_ERROR;
    }

    ff_filtered = PID_LowPass(next_state.filtered_feedforward, ff_raw, config->feedforward_filter_tau, dt);

    if (!isfinite(ff_filtered))
    {
        return PID_STATUS_NUM_ERROR;
    }

    // 积分分离和变速积分
    i_base = next_state.i_out;
    i_delta = 0.0f;

    i_sep = config->I_Sep_Error > 0.0f && error_abs >= config->I_Sep_Error;

    if (config->Ki == 0.0f)
    {
        i_base = 0.0f;
    }
    else if (i_sep)
    {
        if (config->I_Sep_Mode == PID_I_SEPARATION_CLEAR)
        {
            i_base = 0.0f; // CLEAR：一旦进入积分分离，直接把历史积分清 0。等误差回到阈值内，积分从零开始慢慢累积，冲击更小
        }
        // HOLD: 不清零，只是不再继续叠加新的积分，旧积分值原地保持住
    }
    else // 没有积分分离
    {
        i_ratio = PID_I_Ratio(config, error_abs);
        i_delta = config->Ki * error * dt * i_ratio;
    }

    i_candidate = i_base + i_delta;

    if (!isfinite(i_base) || !isfinite(i_delta) || !isfinite(i_candidate))
    {
        return PID_STATUS_NUM_ERROR;
    }

    // 积分限幅
    i_candidate = PID_Clamp(i_candidate, config->I_Min, config->I_Max);

    // 总输出条件积分抗饱和
    output_candidate = p_output + i_candidate + d_out + ff_filtered;

    i_change = i_candidate - i_base;

    if (!isfinite(output_candidate) || !isfinite(i_change))
    {
        return PID_STATUS_NUM_ERROR;
    }

    if (config->anti_windup && ((output_candidate > config->Output_max && i_change > 0.0f) || (output_candidate < config->Output_min && i_change < 0.0f)))
    {
        i_candidate = i_base; // 只拒绝本周期继续推动输出饱和的新增积分
    }

    // 总输出限幅
    output_raw = p_output + i_candidate + d_out + ff_filtered;

    if (!isfinite(output_raw))
    {
        return PID_STATUS_NUM_ERROR;
    }

    output_limited = PID_Clamp(output_raw, config->Output_min, config->Output_max);

    // 输出滤波
    output_filtered = PID_LowPass(next_state.last_out, output_limited, config->output_filter_tau, dt);

    if (!isfinite(output_filtered))
    {
        return PID_STATUS_NUM_ERROR;
    }

    output_filtered = PID_Clamp(output_filtered, config->Output_min, config->Output_max);

    // 完整计算成功后提交状态
    next_state.has_history = true;

    next_state.filtered_measurement = measurement_filtered;
    next_state.pre_error = error;
    next_state.pre_target = target;

    next_state.i_out = i_candidate;
    next_state.filtered_d = d_filtered;
    next_state.filtered_feedforward = ff_filtered;
    next_state.last_out = output_filtered;

    next_state.error = error;
    next_state.p_out = p_output;
    next_state.d_out = d_out;
    next_state.ff_out = ff_filtered;

    pid->pid_state = next_state; // 传入
    *output = output_filtered;

    return PID_STATUS_OK;
}

#include "alg_chassis.h"

//------------内部常量----------------------
/**
 * @brief sin45° = cos45° = 1/根号2
 *
 */
#define CHASSIS_INV_SQRT_TWO 0.7071067811865475f

/**
 * @brief Chassis_Wheels，四个轮子的几何参数表
 *
 * @note Chassis_Wheel_Geometry结构体数组，按对应顺序输入就可以直接赋值
 */
static const Chassis_Wheel_Geometry Chassis_Wheels[CHASSIS_WHEEL_COUNT] =
    {
        /* 左前 FL：位置前左，正驱动方向后左。 */
        {
            +CHASSIS_WHEEL_CENTER_SIDE_M * 0.5f, // x_m，轮心相对底盘中心的前后位置
            +CHASSIS_WHEEL_CENTER_SIDE_M * 0.5f, // y_m，轮心相对底盘中心的左右位置

            -CHASSIS_INV_SQRT_TWO, // tx，正驱动方向在 X 轴上的分量
            +CHASSIS_INV_SQRT_TWO, // yx，正驱动方向在 Y 轴上的分量

            (float)CHASSIS_FL_MOTOR_SIGN}, // motor_sign，电机方向修正：+1 或 -1

        /* 右前 FR：位置前右，正驱动方向前左。 */
        {
            +CHASSIS_WHEEL_CENTER_SIDE_M * 0.5f,
            -CHASSIS_WHEEL_CENTER_SIDE_M * 0.5f,

            +CHASSIS_INV_SQRT_TWO,
            +CHASSIS_INV_SQRT_TWO,

            (float)CHASSIS_FR_MOTOR_SIGN},

        /* 左后 RL：位置后左，正驱动方向后右。 */
        {
            -CHASSIS_WHEEL_CENTER_SIDE_M * 0.5f,
            +CHASSIS_WHEEL_CENTER_SIDE_M * 0.5f,

            -CHASSIS_INV_SQRT_TWO,
            -CHASSIS_INV_SQRT_TWO,

            (float)CHASSIS_RL_MOTOR_SIGN},

        /* 右后 RR：位置后右，正驱动方向前右。 */
        {
            -CHASSIS_WHEEL_CENTER_SIDE_M * 0.5f,
            -CHASSIS_WHEEL_CENTER_SIDE_M * 0.5f,

            +CHASSIS_INV_SQRT_TWO,
            -CHASSIS_INV_SQRT_TWO,

            (float)CHASSIS_RR_MOTOR_SIGN}};

//-----------内部函数----------------------

/**
 * @brief 对浮点数执行上下限限制后输出
 *
 * @param value 输入值
 * @param lower 下限
 * @param upper 上限
 * @return 限幅后的值
 * @note 调用方保证输入有限且上下限合法
 */
static float Chassis_Clamp(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }

    if (value > upper)
    {
        return upper;
    }

    return value;
}

/**
 * @brief 检查运行模式
 *
 * @param mode 待检查模式
 * @return true 表示合法
 */
static bool Chassis_IsModeValid(Chassis_Mode mode)
{
    switch (mode)
    {
    case CHASSIS_MODE_STOP:
    case CHASSIS_MODE_NORMAL:
    case CHASSIS_MODE_SPIN:
        return true; // 就这三个

    default:
        return false;
    }
}

/**
 * @brief 检查坐标系类型
 *
 * @param frame 待检查坐标系
 * @return true 表示合法
 */
static bool Chassis_IsFrameValid(Chassis_Frame frame)
{
    if (frame == CHASSIS_FRAME_BODY || frame == CHASSIS_FRAME_REFERENCE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief 检查顶部配置宏是否合法
 *
 * @return true 表示配置可用于解算
 */
static bool Chassis_IsConfigValid(void)
{
    if (!isfinite(CHASSIS_WHEEL_CENTER_SIDE_M) || CHASSIS_WHEEL_CENTER_SIDE_M <= 0.0f || !isfinite(CHASSIS_WHEEL_RADIUS_M) || CHASSIS_WHEEL_RADIUS_M <= 0.0f)
    {
        return false;
    }

    if (!isfinite(CHASSIS_MAX_LINEAR_SPEED_MPS) || CHASSIS_MAX_LINEAR_SPEED_MPS <= 0.0f || !isfinite(CHASSIS_MAX_YAW_RATE_RADPS) || CHASSIS_MAX_YAW_RATE_RADPS <= 0.0f || !isfinite(CHASSIS_MAX_WHEEL_SPEED_RADPS) || CHASSIS_MAX_WHEEL_SPEED_RADPS <= 0.0f)
    {
        return false;
    }

    if (!isfinite(CHASSIS_SPIN_DEFAULT_RADPS) || fabsf(CHASSIS_SPIN_DEFAULT_RADPS) > CHASSIS_MAX_YAW_RATE_RADPS) // fabsf是求float绝对值的函数
    {
        return false;
    }

    /*
     * 无符号时间差采用常规的短超时约定。
     * 上层必须周期调用，不能跨越整个毫秒计数周期后才检查。
     * 因为时间戳是 uint32_t，它会在2^32毫秒后回绕。用无符号减法判断“经过了多久”时，只能可靠地区分小于半个计数周期的时间间隔，所以边界选2^31，也就是 0x80000000UL
     */
    if (CHASSIS_COMMAND_TIMEOUT_MS == 0U ||
        CHASSIS_COMMAND_TIMEOUT_MS >= 0x80000000UL)
    {
        return false;
    }

    /*
     * 方向修正只能为 +1 或 -1。
     * 填成 0 会使该轮失去输出，填其他值会改变速度比例。
     */
    if ((CHASSIS_FL_MOTOR_SIGN != 1 &&
         CHASSIS_FL_MOTOR_SIGN != -1) ||
        (CHASSIS_FR_MOTOR_SIGN != 1 &&
         CHASSIS_FR_MOTOR_SIGN != -1) ||
        (CHASSIS_RL_MOTOR_SIGN != 1 &&
         CHASSIS_RL_MOTOR_SIGN != -1) ||
        (CHASSIS_RR_MOTOR_SIGN != 1 &&
         CHASSIS_RR_MOTOR_SIGN != -1))
    {
        return false;
    }

    return true;
}

/**
 * @brief 清零当前输出和解算诊断量
 *
 * @param chassis 底盘对象
 *
 * @note 不修改模式、指令内容和接收时间
 */
static void Chassis_ClearOutput(Chassis_Controller *chassis)
{
    uint32_t index;

    chassis->requested_body = (Chassis_Velocity){0};
    chassis->limited_body = (Chassis_Velocity){0};

    for (index = 0U; index < CHASSIS_WHEEL_COUNT; ++index)
    {
        chassis->wheel_target_radps[index] = 0.0f;
    }

    chassis->wheel_scale = 0.0f;
    chassis->output_valid = false;
}

/**
 * @brief 将指令平移速度转换到底盘坐标系
 *
 * @param command 输入指令
 * @param body 转换后的速度
 * @return Chassis_Status 运行结果，一个状态
 * @note
 *      theta = command->reference_yaw_rad，从底盘车头方向逆时针转多少角度才能对其参考坐标系的前方，一个夹角
 *      vx_body = cos(theta) * vx_reference
 *              - sin(theta) * vy_reference
 *
 *      vy_body = sin(theta) * vx_reference
 *              + cos(theta) * vy_reference
 */
static Chassis_Status Chassis_CommandToBody(const Chassis_Command *command, Chassis_Velocity *body)
{
    float cosine;
    float sine;
    float theta;

    *body = command->velocity; // 输入速度

    if (command->frame == CHASSIS_FRAME_BODY)
    {
        return CHASSIS_STATUS_OK; // 使用底盘自身方向定义，不需要旋转变换
    }

    theta = command->reference_yaw_rad;
    cosine = cosf(theta);
    sine = sinf(theta);

    body->vx_mps = cosine * command->velocity.vx_mps - sine * command->velocity.vy_mps;
    body->vy_mps = sine * command->velocity.vx_mps + cosine * command->velocity.vy_mps;

    if (!isfinite(body->vx_mps) || !isfinite(body->vy_mps))
    {
        return CHASSIS_STATUS_NUM_ERROR;
    }

    return CHASSIS_STATUS_OK;
}

/**
 * @brief 底盘限速
 *
 * @param body 底盘坐标系速度
 * @return Chassis_Status 限速结果状态
 */
static Chassis_Status Chassis_LimitBodyVelocity(Chassis_Velocity *body)
{
    float linear_speed; // 当前平移速度的大小
    float scale;        // 超速时的缩小比例

    // 平移限幅
    linear_speed = hypotf(body->vx_mps, body->vy_mps); // 根据前后左右两个方向的速度计算底盘实际的平移速度大小，hypotf就是根号下两数平方和

    if (!isfinite(linear_speed))
    {
        return CHASSIS_STATUS_NUM_ERROR;
    }

    if (linear_speed > CHASSIS_MAX_LINEAR_SPEED_MPS) // 判断合成速度是否超过宏定义的最大平移速度
    {
        scale = CHASSIS_MAX_LINEAR_SPEED_MPS / linear_speed; // 计算缩小比例，使得原来的速度乘上这个比例后，刚好等于允许的最大速度

        body->vx_mps *= scale;
        body->vy_mps *= scale;
    }

    // 转动限幅
    body->wz_radps = Chassis_Clamp(body->wz_radps, -CHASSIS_MAX_YAW_RATE_RADPS, +CHASSIS_MAX_YAW_RATE_RADPS);

    return CHASSIS_STATUS_OK;
}

/**
 * @brief 将底盘速度解算为四轮目标角速度
 *
 * @param chassis 底盘对象
 *
 * @return 解算结果状态
 *
 * @note
 * 刚体上轮心处的速度：
 * vx_wheel = vx - wz * y
 * vy_wheel = vy + wz * x
 *
 * 投影到轮子有效驱动方向：
 * wheel_omega =(tx * vx_wheel + ty * vy_wheel) / radius
 *
 */
static Chassis_Status Chassis_Jiesuan(Chassis_Controller *chassis)
{
    uint32_t index;

    float vx_at_wheel; // 轮心沿底盘X方向的速度
    float vy_at_wheel; // 轮心沿底盘Y方向的速度
    float wheel_speed; // 轮子的目标转速，角速度，radps
    float peak_speed = 0.0f;

    const Chassis_Wheel_Geometry *wheel; // 一个指针变量，每次循环让它指向不同轮子的参数

    for (index = 0; index < CHASSIS_WHEEL_COUNT; ++index)
    {
        wheel = &Chassis_Wheels[index];

        vx_at_wheel = chassis->requested_body.vx_mps + (-chassis->requested_body.wz_radps * wheel->y_m); // 前后平移速度加轮心旋转速度在前后方向上的分量(是负的因为逆时针为正)
        vy_at_wheel = chassis->requested_body.vy_mps + (-chassis->requested_body.wz_radps * wheel->x_m);

        wheel_speed = (wheel->tx * vx_at_wheel + wheel->ty * vy_at_wheel) / CHASSIS_WHEEL_RADIUS_M;

        wheel_speed *= wheel->motor_sign;

        if (!isfinite(wheel_speed))
        {
            return CHASSIS_STATUS_NUM_ERROR;
        }

        chassis->wheel_target_radps[index] = wheel_speed;

        if (fabsf(wheel_speed) > peak_speed)
        {
            peak_speed = fabsf(wheel_speed);
        }
    }

    /*
     * 四轮统一缩放。
     *
     * 若某轮超限，四轮同时乘同一比例，
     * 保留平移与旋转之间的原始比例关系。
     */
    chassis->wheel_scale = 1.0f;

    if (peak_speed > CHASSIS_MAX_WHEEL_SPEED_RADPS)
    {
        chassis->wheel_scale = CHASSIS_MAX_WHEEL_SPEED_RADPS / peak_speed;

        for (index = 0U; index < CHASSIS_WHEEL_COUNT; ++index)
        {
            chassis->wheel_target_radps[index] *= chassis->wheel_scale;
        }
    }

    /*
     * 根据相同缩放比例计算理论可执行的底盘速度。
     * 便于串口绘图时区分“请求”和“限速后目标”。
     */
    chassis->limited_body.vx_mps = chassis->requested_body.vx_mps * chassis->wheel_scale;

    chassis->limited_body.vy_mps = chassis->requested_body.vy_mps * chassis->wheel_scale;

    chassis->limited_body.wz_radps = chassis->requested_body.wz_radps * chassis->wheel_scale;

    return CHASSIS_STATUS_OK;
}

//-----------公共函数--------------------------
/**
 * @brief 初始化底盘解算对象
 *
 * @param chassis 底盘对象
 *
 * @return 初始化结果状态
 */
Chassis_Status Chassis_Init(Chassis_Controller *chassis)
{
    if (chassis == NULL)
    {
        return CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    *chassis = (Chassis_Controller){0};

    chassis->mode = CHASSIS_MODE_STOP;
    chassis->command.frame = CHASSIS_FRAME_BODY;

    if (!Chassis_IsConfigValid())
    {
        chassis->last_status =
            CHASSIS_STATUS_INVALID_CONFIG;

        return chassis->last_status;
    }

    chassis->spin_rate_radps =
        CHASSIS_SPIN_DEFAULT_RADPS;

    chassis->init = true;
    chassis->last_status = CHASSIS_STATUS_OK;

    return CHASSIS_STATUS_OK;
}

/**
 * @brief 设置运行模式
 *
 * @param chassis 底盘对象
 * @param mode    新模式
 *
 * @return 设置结果状态
 */
Chassis_Status Chassis_SetMode(Chassis_Controller *chassis, Chassis_Mode mode)
{
    if (chassis == NULL)
    {
        return CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    if (!chassis->init)
    {
        return CHASSIS_STATUS_NOT_INITIALIZED;
    }

    if (!Chassis_IsModeValid(mode))
    {
        // 非法模式请求也撤销运动，避免继续执行旧命令
        chassis->mode = CHASSIS_MODE_STOP;
        chassis->has_command = false;

        Chassis_ClearOutput(chassis);

        chassis->last_status = CHASSIS_STATUS_INVALID_ARGUMENT;

        return chassis->last_status;
    }

    if (chassis->mode != mode)
    {
        chassis->mode = mode;
        chassis->has_command = false;

        Chassis_ClearOutput(chassis);
    }

    chassis->last_status = CHASSIS_STATUS_OK;

    return CHASSIS_STATUS_OK;
}

/**
 * @brief 设置小陀螺旋转速度
 *
 * @param chassis 底盘对象
 * @param spin_radps 旋转速度，rad/s
 *
 * @return 设置结果状态
 */
Chassis_Status Chassis_SetSpinRate(Chassis_Controller *chassis, float spin_radps)
{
    if (chassis == NULL)
    {
        return CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    if (!chassis->init)
    {
        return CHASSIS_STATUS_NOT_INITIALIZED;
    }

    if (!isfinite(spin_radps))
    {
        chassis->has_command = false;
        Chassis_ClearOutput(chassis);

        chassis->last_status = CHASSIS_STATUS_INVALID_ARGUMENT;

        return chassis->last_status;
    }

    chassis->spin_rate_radps = Chassis_Clamp(spin_radps, -CHASSIS_MAX_YAW_RATE_RADPS, +CHASSIS_MAX_YAW_RATE_RADPS); // 限幅后输出

    /*
     * 变更旋转目标后，撤销之前已计算的输出。
     * 下一次 Update 根据新参数重新计算。
     */
    Chassis_ClearOutput(chassis);

    chassis->last_status = CHASSIS_STATUS_OK;

    return CHASSIS_STATUS_OK;
}

/**
 * @brief 提交运动指令并刷新有效时间
 *
 * @param chassis 底盘对象
 * @param command    完整指令
 * @param now_ms     当前毫秒时间
 *
 * @return 提交结果状态
 */
Chassis_Status Chassis_SetCommand(Chassis_Controller *chassis, const Chassis_Command *command, uint32_t now_ms)
{
    if (chassis == NULL)
    {
        return CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    if (!chassis->init)
    {
        return CHASSIS_STATUS_NOT_INITIALIZED;
    }

    if (command == NULL || !Chassis_IsFrameValid(command->frame))
    {
        chassis->has_command = false;
        Chassis_ClearOutput(chassis);

        chassis->last_status = CHASSIS_STATUS_INVALID_ARGUMENT;

        return chassis->last_status; // 无效就清掉旧指令和旧输出，再返回错误状态，避免底盘继续沿用上一条速度命令
    }

    if (!isfinite(command->velocity.vx_mps) || !isfinite(command->velocity.vy_mps) || !isfinite(command->velocity.wz_radps) || (command->frame == CHASSIS_FRAME_REFERENCE && !isfinite(command->reference_yaw_rad)))
    {
        chassis->has_command = false;
        Chassis_ClearOutput(chassis);

        chassis->last_status = CHASSIS_STATUS_INVALID_ARGUMENT;

        return chassis->last_status;
    }

    chassis->command = *command;

    chassis->last_command_ms = now_ms;
    chassis->has_command = true;

    /*
     * 新指令到达后，需要重新 Update 才发布对应轮速。
     */
    Chassis_ClearOutput(chassis);

    chassis->last_status = CHASSIS_STATUS_OK;

    return CHASSIS_STATUS_OK;
}

/**
 * @brief 周期执行模式处理、限速和四轮解算
 *
 * @param chassis 底盘对象
 * @param now_ms      当前毫秒时间
 *
 * @return 本次处理结果
 */
Chassis_Status Chassis_Update(Chassis_Controller *chassis, uint32_t now_ms)
{
    Chassis_Status status;
    Chassis_Velocity body;

    if (chassis == NULL)
    {
        return CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    Chassis_ClearOutput(chassis);

    if (!chassis->init)
    {
        chassis->last_status = CHASSIS_STATUS_NOT_INITIALIZED;

        return chassis->last_status;
    }

    if (!Chassis_IsModeValid(chassis->mode))
    {
        chassis->has_command = false;

        chassis->last_status = CHASSIS_STATUS_INVALID_ARGUMENT;

        return chassis->last_status;
    }

    if (chassis->mode == CHASSIS_MODE_STOP)
    {
        chassis->last_status = CHASSIS_STATUS_OK;
        return CHASSIS_STATUS_OK;
    }

    if (!chassis->has_command || (uint32_t)(now_ms - chassis->last_command_ms) > CHASSIS_COMMAND_TIMEOUT_MS)
    {
        // 超时后撤销指令只有重新提交合法指令，才能恢复运动
        chassis->has_command = false;

        chassis->last_status = CHASSIS_STATUS_COMMAND_TIMEOUT;

        return chassis->last_status;
    }

    status = Chassis_CommandToBody(&chassis->command, &body);

    if (status != CHASSIS_STATUS_OK)
    {
        chassis->last_status = status;
        return status;
    }

    // NORMAL模式使用输入指令中的wz，已经存入body
    if (chassis->mode == CHASSIS_MODE_SPIN) // SPIN模式平移仍由指令控制，旋转使用小陀螺速度设定覆盖角速度
    {
        body.wz_radps = chassis->spin_rate_radps;
    }

    status = Chassis_LimitBodyVelocity(&body);

    if (status != CHASSIS_STATUS_OK)
    {
        chassis->last_status = status;
        return status;
    }

    chassis->requested_body = body;

    status = Chassis_Jiesuan(chassis);

    if (status != CHASSIS_STATUS_OK)
    {
        // 解算中途失败时也必须清理部分轮速结果
        Chassis_ClearOutput(chassis);

        chassis->last_status = status;
        return status;
    }

    chassis->output_valid = true;
    chassis->last_status = CHASSIS_STATUS_OK;

    return CHASSIS_STATUS_OK;
}

/**
 * @brief 根据轮速反馈估算底盘速度
 *
 * @param wheel_feedback_radps 电机输出轴速度反馈
 * @param body_velocity       底盘速度估计值
 * @return 计算结果状态
 */
Chassis_Status Chassis_ForwardKinematics(const float wheel_feedback_radps[CHASSIS_WHEEL_COUNT], Chassis_Velocity *body_velocity)
{
    uint32_t index;

    float wheel_speed[CHASSIS_WHEEL_COUNT];

    float vx_sum = 0.0f; // 沿底盘前后方向的速度累加值
    float vy_sum = 0.0f; // 左右
    float wz_sum = 0.0f; // 角

    Chassis_Velocity result;

    if (body_velocity == NULL)
    {
        return CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    *body_velocity = (Chassis_Velocity){0};

    if (wheel_feedback_radps == NULL)
    {
        return CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    if (!Chassis_IsConfigValid())
    {
        return CHASSIS_STATUS_INVALID_CONFIG;
    }

    /*
     * 消除各电机安装方向差异 恢复成几何公式约定的轮子正方向
     */
    for (index = 0U; index < CHASSIS_WHEEL_COUNT; ++index)
    {
        if (!isfinite(wheel_feedback_radps[index]))
        {
            return CHASSIS_STATUS_INVALID_ARGUMENT;
        }

        wheel_speed[index] = wheel_feedback_radps[index] * Chassis_Wheels[index].motor_sign;
    }

    /*
     * 对当前对称四轮布局：
     *
     * vx = radius / 2 * Σ(tx_i * wheel_i)
     * vy = radius / 2 * Σ(ty_i * wheel_i)
     *
     * wz = radius / (4 * k) * Σ(wheel_i)
     *
     * 其中：
     * k = side / sqrt(2)
     */
    for (index = 0U; index < CHASSIS_WHEEL_COUNT; ++index)
    {
        vx_sum += Chassis_Wheels[index].tx * wheel_speed[index];

        vy_sum += Chassis_Wheels[index].ty * wheel_speed[index];

        wz_sum += wheel_speed[index];
    }

    result.vx_mps = CHASSIS_WHEEL_RADIUS_M * 0.5f * vx_sum;

    result.vy_mps = CHASSIS_WHEEL_RADIUS_M * 0.5f * vy_sum;

    result.wz_radps = CHASSIS_WHEEL_RADIUS_M * wz_sum / (4.0f * CHASSIS_WHEEL_CENTER_SIDE_M * CHASSIS_INV_SQRT_TWO);

    if (!isfinite(result.vx_mps) || !isfinite(result.vy_mps) || !isfinite(result.wz_radps))
    {
        return CHASSIS_STATUS_NUM_ERROR;
    }

    *body_velocity = result;

    return CHASSIS_STATUS_OK;
}

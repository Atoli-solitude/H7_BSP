//
// Created by Atoli on 2026/10/25.
//

#include "chassis.h"
#include "dmmotor.h"
#include "cmsis_os2.h"
#include "sys_timestamp.h"
#include <cmath>

// ==================== 底盘集中配置（轮序：左上、右上、左下、右下） ====================
// 尺寸暂为占位值，实车运动前必须测量修改；单位 m，减速比为电机转角/轮组转角。
static constexpr ChassisGeometry kGeometry{0.20f, 0.20f, 0.05f, 1.0f};
static constexpr float kSteerReductionRatio = 1.0f;
// !!! 禁止解锁：必须先确认实车 kp/kd、G6220 固件 MIT 模式、协议轴侧及 PMAX 范围 !!!
// 0 仅为未配置占位，不是推荐增益；确认尺寸、方向、零位、减速比及全部量程后才能置 true。
static constexpr bool kMotionConfigurationConfirmed = false;
static constexpr float kSteerKp = 0.0f;
static constexpr float kSteerKd = 0.0f;
static constexpr uint8_t kDriveId[CHASSIS_WHEEL_COUNT] = {1, 2, 3, 4};
static constexpr uint8_t kSteerId[CHASSIS_WHEEL_COUNT] = {5, 6, 7, 8};
// 全部使用 FDCAN1，Master ID 与 CAN ID 相同；驱动方向+1，转向电机正转为顺时针。
static constexpr float kDriveDirection[CHASSIS_WHEEL_COUNT] = {1, 1, 1, 1};
static constexpr float kSteerDirection[CHASSIS_WHEEL_COUNT] = {-1, -1, -1, -1};
// 轮组朝车体+X时的电机位置（rad）；此软件偏置不会向电机发送置零命令。
static constexpr float kSteerZero[CHASSIS_WHEEL_COUNT] = {0, 0, 0, 0};
// 以下量程沿用旧配置，尚未实车确认；转向同时用于 MIT 编码及反馈解码，必须与固件一致。
static constexpr float kDrivePmax = 12.5f, kDriveVmax = 45.0f, kDriveTmax = 10.0f;
static constexpr float kSteerPmax = 12.5f, kSteerVmax = 45.0f, kSteerTmax = 10.0f;
static constexpr uint64_t kFeedbackTimeoutUs = 100000U; // 100ms 无反馈则故障锁定
static constexpr uint64_t kStartupFeedbackGraceUs = 1000000U; // 使能提交后最多等待1s完整反馈
static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kZeroSpeed = 1.0e-6f;
// ==============================================================================

bool Chassis_Calculate(const ChassisGeometry &geometry, float vx, float vy, float wz,
                       ChassisWheelTarget (&targets)[CHASSIS_WHEEL_COUNT])
{
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) ||
        !std::isfinite(geometry.half_width) || geometry.half_width <= 0.0f ||
        !std::isfinite(geometry.half_length) || geometry.half_length <= 0.0f ||
        !std::isfinite(geometry.wheel_radius) || geometry.wheel_radius <= 0.0f ||
        !std::isfinite(geometry.drive_reduction_ratio) || geometry.drive_reduction_ratio <= 0.0f)
    {
        return false;
    }

    // 以底盘中心为原点，俯视 +X 向右、+Y 向上；这里是轮组的位置，不是速度。
    // 下标 i：       0 左上       1 右上       2 左下       3 右下
    // 坐标 (x,y)： (-半宽,+半长) (+半宽,+半长) (-半宽,-半长) (+半宽,-半长)
    const float x[CHASSIS_WHEEL_COUNT] = {
        -geometry.half_width, geometry.half_width,
        -geometry.half_width, geometry.half_width};
    const float y[CHASSIS_WHEEL_COUNT] = {
        geometry.half_length, geometry.half_length,
        -geometry.half_length, -geometry.half_length};
    // 先写入临时数组；四个轮都计算成功后才更新 targets，避免只更新一部分轮。
    ChassisWheelTarget result[CHASSIS_WHEEL_COUNT]{};
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        // 每个轮的目标速度 = 底盘中心平移速度 + 底盘旋转产生的切向速度。
        // 平面刚体公式：omega × (x,y,0) = (-wz*y, +wz*x, 0)。
        // vx、vy 和 wheel_vx、wheel_vy 单位 m/s；wz 单位 rad/s；x、y 单位 m。
        // 例如只逆时针旋转时，上方轮的 X 速度为负（向左），右侧轮的 Y 速度为正（向上）。
        const float wheel_vx = vx - wz * y[i];
        const float wheel_vy = vy + wz * x[i];
        // hypot(a,b) = sqrt(a*a+b*b)：由两个速度分量求合速度大小，单位 m/s。
        // 此处大小非负；后面的最短转向优化可能将速度反号，让轮子倒转。
        result[i].linear_speed = std::hypot(wheel_vx, wheel_vy);
        // v = omega*r，因此 v/轮半径得到轮子的角速度，再乘减速比得到指令角速度 rad/s。
        // 减速比须按协议指令轴与车轮的关系填写；协议若已按输出轴换算，不要重复计入内部减速比。
        result[i].drive_speed = result[i].linear_speed / geometry.wheel_radius *
                                geometry.drive_reduction_ratio;
        // atan2(y,x) 根据速度方向求舵角，单位 rad：朝 +X 为 0，朝 +Y 为 +pi/2。
        // “条件 ? A : B”表示条件成立取 A，否则取 B。
        // 速度大于 1e-6 m/s 才计算新角度；接近零速时没有确定方向，保持旧目标，避免停车回正。
        result[i].steer_angle = result[i].linear_speed > 1.0e-6f
            ? std::atan2(wheel_vy, wheel_vx) : targets[i].steer_angle;
        // isfinite 检查是否为有限数，排除 NaN 和无穷大；不是速度或角度限幅。
        if (!std::isfinite(result[i].linear_speed) ||
            !std::isfinite(result[i].drive_speed) || !std::isfinite(result[i].steer_angle))
        {
            return false;
        }
    }
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        targets[i] = result[i];
    }
    return true;
}

bool Chassis_CalculateGlobal(const ChassisGeometry &geometry,
                             float global_vx, float global_vy, float wz, float yaw,
                             ChassisWheelTarget (&targets)[CHASSIS_WHEEL_COUNT])
{
    if (!std::isfinite(yaw))
    {
        return false;
    }
    const float cos_yaw = std::cos(yaw);
    const float sin_yaw = std::sin(yaw);
    return Chassis_Calculate(geometry,
                             cos_yaw * global_vx + sin_yaw * global_vy,
                             -sin_yaw * global_vx + cos_yaw * global_vy,
                             wz, targets);
}

bool Chassis_Optimize(const float (&actual_angles)[CHASSIS_WHEEL_COUNT],
                      ChassisWheelTarget (&targets)[CHASSIS_WHEEL_COUNT])
{
    ChassisWheelTarget result[CHASSIS_WHEEL_COUNT];
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        result[i] = targets[i];
        if (!std::isfinite(actual_angles[i]) || !std::isfinite(result[i].steer_angle) ||
            !std::isfinite(result[i].drive_speed) || !std::isfinite(result[i].linear_speed))
        {
            return false;
        }
        if (std::fabs(result[i].linear_speed) <= kZeroSpeed)
        {
            result[i].linear_speed = 0.0f;
            result[i].drive_speed = 0.0f;
            continue; // 停车保留上一次目标，而不是跟随编码器抖动。
        }
        float delta = std::remainder(result[i].steer_angle - actual_angles[i], 2.0f * kPi);
        if (std::fabs(delta) > 0.5f * kPi)
        {
            delta = std::remainder(delta + kPi, 2.0f * kPi);
            result[i].linear_speed = -result[i].linear_speed;
            result[i].drive_speed = -result[i].drive_speed;
        }
        // 仅归一化角差；连续目标靠近实测位置，不折回单圈区间。
        result[i].steer_angle = actual_angles[i] + delta;
        if (!std::isfinite(result[i].steer_angle))
        {
            return false;
        }
    }
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        targets[i] = result[i];
    }
    return true;
}

static Class_DMMotor drive_motors[CHASSIS_WHEEL_COUNT];         //四个驱动电机对象
static Class_DMMotor steer_motors[CHASSIS_WHEEL_COUNT];         //四个转向电机对象
static ChassisWheelTarget wheel_targets[CHASSIS_WHEEL_COUNT];
static bool chassis_ready = false;
static bool init_attempted = false;
static bool angles_initialized = false;
static bool fault_latched = false;
static bool drive_initialized[CHASSIS_WHEEL_COUNT]{};
static bool steer_initialized[CHASSIS_WHEEL_COUNT]{};
static bool drive_disable_submitted[CHASSIS_WHEEL_COUNT]{};
static bool steer_disable_submitted[CHASSIS_WHEEL_COUNT]{};
static uint64_t enable_timestamp_us = 0;
static float command_vx = 0.0f, command_vy = 0.0f, command_wz = 0.0f;

static bool MotionConfigurationValid()
{
    // 上界来自当前 dmmotor MIT 编码范围，不代表实车推荐值；位置闭环要求 kp > 0。
    return kMotionConfigurationConfirmed &&
           std::isfinite(kSteerKp) && kSteerKp > 0.0f && kSteerKp <= 500.0f &&
           std::isfinite(kSteerKd) && kSteerKd >= 0.0f && kSteerKd <= 5.0f &&
           std::isfinite(kSteerReductionRatio) && kSteerReductionRatio > 0.0f &&
           std::isfinite(kSteerPmax) && kSteerPmax > 0.0f &&
           std::isfinite(kSteerVmax) && kSteerVmax > 0.0f &&
           std::isfinite(kSteerTmax) && kSteerTmax > 0.0f;
}

static void PublishZeroMotion()
{
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        if (drive_initialized[i])
        {
            drive_motors[i].SetSpeed(0.0f);
        }
        if (steer_initialized[i])
        {
            // 覆盖旧周期目标，不能仅插入 Disable 后留下旧 MIT 运动帧。
            steer_motors[i].SetMIT(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

static void StopAndLatchFault()
{
    fault_latched = true; // 只允许重启解除，通信恢复也不自动使能。
    chassis_ready = false;
    Chassis_SetBodyVelocity(0.0f, 0.0f, 0.0f);
    PublishZeroMotion();
    // 零转矩不等于制动；CAN 断开仍需电机自身通信超时保护。
    // 每轮每台最多尝试一次；入队成功的不再重复，硬件 FIFO 满由 CAN 任务重试。
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        if (drive_initialized[i] && !drive_disable_submitted[i])
        {
            drive_disable_submitted[i] = drive_motors[i].Disable();
        }
        if (steer_initialized[i] && !steer_disable_submitted[i])
        {
            steer_disable_submitted[i] = steer_motors[i].Disable();
        }
    }
}
//### 给底盘速度命令
//位置： 车体速度接口
bool Chassis_SetBodyVelocity(float vx, float vy, float wz)
{
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz))
    {
        return false;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if ((vx != 0.0f || vy != 0.0f || wz != 0.0f) &&
        (!MotionConfigurationValid() || !chassis_ready || !angles_initialized || fault_latched))
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return false;
    }
    command_vx = vx;
    command_vy = vy;
    command_wz = wz;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
    return true;
}

bool Chassis_Init()
{
    if (init_attempted)
    {
        return chassis_ready; // 不重复注册、使能或自动清错。
    }
    init_attempted = true;
    Chassis_SetBodyVelocity(0.0f, 0.0f, 0.0f);
    if (!MotionConfigurationValid())
    {
        StopAndLatchFault(); // 配置未确认时不注册、不使能任何电机。
        return false;
    }
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        // 方向换算由底盘完成；Init 只配置主控，G6220 固件须预先确认处于 MIT。
        drive_initialized[i] = drive_motors[i].Init(&hfdcan1, kDriveId[i], kDriveId[i],
                                  Enum_DMMotor_Mode::SPEED, false,
                                  kDrivePmax, kDriveVmax, kDriveTmax);
        steer_initialized[i] = steer_motors[i].Init(&hfdcan1, kSteerId[i], kSteerId[i],
                                  Enum_DMMotor_Mode::MIT, false,
                                  kSteerPmax, kSteerVmax, kSteerTmax);
        if (!drive_initialized[i] || !steer_initialized[i])
        {
            StopAndLatchFault();
            return false;
        }
    }
    PublishZeroMotion();
    osDelay(2U * osKernelGetTickFreq());
    bool submitted = true;
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        // 8帧小于插入队列深度16；硬件FIFO满由CAN任务留存重试，每台仅提交一次。
        const bool drive_ok = drive_motors[i].Enable();
        const bool steer_ok = steer_motors[i].Enable();
        submitted = submitted && drive_ok && steer_ok;
    }
    if (!submitted)
    {
        StopAndLatchFault(); // 已入队的 Enable 无法撤回，后续 Disable 按 FIFO 顺序撤销。
        return false;
    }
    enable_timestamp_us = SYS_Timestamp.Get_Now_Microsecond();
    chassis_ready = true; // 仅表示提交成功，运行仍要求8台真实且已使能的反馈。
    return true;
}

// 每次调用只处理一轮控制，不在这里等待电机到位，也不在这里执行电机内部 PID。
// 流程：检查反馈 -> 获取实际轮向 -> 读取车体命令 -> 解算并优化 -> 提交八台电机的目标。
void Chassis_Loop()
{
    // 1. 故障后只重试尚未入队的失能请求，绝不自动重新使能。
    if (fault_latched)
    {
        StopAndLatchFault();
        return;
    }
    // 本地初始化及使能提交未成功时不运行；ready 不等于八台电机实际在线。
    if (!chassis_ready)
    {
        return;
    }
    // 2. 读取四对电机已收到的 CAN 反馈快照，不是此刻直接向编码器发起读取。
    // actual_angles 是轮组的实际转向角（rad），不是驱动轮滚动角，也不是目标角。
    float actual_angles[CHASSIS_WHEEL_COUNT];
    bool feedback_ready = true;
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        Struct_DMMotor_Feedback drive, steer;
        const bool drive_ok = drive_motors[i].GetFeedback(drive, kFeedbackTimeoutUs);
        const bool steer_ok = steer_motors[i].GetFeedback(steer, kFeedbackTimeoutUs);
        // 反馈位置先减软件零位，再除以方向和传动比，换算成统一的轮组角度。
        // total_position 是驱动软件展开的连续位置；其轴侧定义及传动比仍须按电机协议核对。
        actual_angles[i] = (steer.total_position - kSteerZero[i]) /
                          (kSteerDirection[i] * kSteerReductionRatio);
        // 八台都要有 100ms 内的有效反馈、状态为 1（使能），且轮向计算有效。
        // 这里检查的是能否正常控制，不是检查转向是否已经到达目标角度。
        feedback_ready = feedback_ready && drive_ok && steer_ok &&
                         drive.state == 1U && steer.state == 1U &&
                         std::isfinite(actual_angles[i]);
    }
    // 3. 启动宽限只用于首次完整反馈；期间始终零运动，之后任何异常立即锁定。
    if (!feedback_ready)
    {
        if (angles_initialized ||
            SYS_Timestamp.Get_Now_Microsecond() - enable_timestamp_us >= kStartupFeedbackGraceUs)
        {
            StopAndLatchFault();
        }
        else
        {
            PublishZeroMotion();
            Chassis_SetBodyVelocity(0.0f, 0.0f, 0.0f);
        }
        return;
    }
    // 4. 首次收到完整有效反馈，以当前实际轮向作为初始目标。
    // 这样零速启动会保持当前位置，而不是因为默认目标为 0 就让四个轮突然回正。
    if (!angles_initialized)
    {
        for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
        {
            wheel_targets[i].steer_angle = actual_angles[i];
        }
        angles_initialized = true;
    }
    // 5. 一次性取得上层 Chassis_SetBodyVelocity 保存的三个车体速度分量。
    // 短暂关中断，防止读取过程中命令被更新，造成新旧分量混用。
    // PRIMASK 保存原中断屏蔽状态；DMB 保证内存访问顺序；原先允许中断才重新打开。
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const float vx = command_vx, vy = command_vy, wz = command_wz;
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
    // 6. Calculate 把车体速度变成四组“驱动速度 + 舵角”。
    // Optimize 根据实际轮向选择最短转向：角差超过 90° 时，目标翻转 180°、驱动速度反号。
    // || 会短路：解算失败就不再优化；任一步失败，整组停止并故障锁定。
    if (!Chassis_Calculate(kGeometry, vx, vy, wz, wheel_targets) ||
        !Chassis_Optimize(actual_angles, wheel_targets))
    {
        StopAndLatchFault();
        return;
    }
    // 7. 先检查全部连续目标，再提交任何一台，禁止依赖 SetMIT 静默限幅或擅自取模。
    // 转向位置 = 零位 + 方向 * 传动比 * 轮组目标角，是反馈角换算的逆运算。
    float steer_positions[CHASSIS_WHEEL_COUNT];
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        steer_positions[i] = kSteerZero[i] + kSteerDirection[i] *
                             kSteerReductionRatio * wheel_targets[i].steer_angle;
        if (!std::isfinite(steer_positions[i]) || std::fabs(steer_positions[i]) > kSteerPmax)
        {
            StopAndLatchFault();
            return;
        }
    }
    // 直接给最终位置，无速度斜坡；MIT 目标速度和转矩前馈均为0，驱动保持 SPEED。
    // 注意：当前同一轮循环就提交转向和驱动目标，没有“转向到位后才允许行进”的判断。
    for (unsigned i = 0; i < CHASSIS_WHEEL_COUNT; ++i)
    {
        steer_motors[i].SetMIT(steer_positions[i], 0.0f, kSteerKp, kSteerKd, 0.0f);
        drive_motors[i].SetSpeed(kDriveDirection[i] * wheel_targets[i].drive_speed);
    }
}
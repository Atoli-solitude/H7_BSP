//
// Created by Atoli on 2026/10/25.
//

#ifndef H7_BSP_CHASSIS_H
#define H7_BSP_CHASSIS_H

#pragma once

constexpr unsigned CHASSIS_WHEEL_COUNT = 4;

// 俯视：+X 向右，+Y 向上，角速度逆时针为正；轮序：左上、右上、左下、右下。
struct ChassisGeometry
{
    float half_width;             // 中心到左右轮的 X 距离，m
    float half_length;            // 中心到前后轮的 Y 距离，m
    float wheel_radius;           // 轮半径，m
    float drive_reduction_ratio;  // 减速比，正数
};

struct ChassisWheelTarget
{
    float linear_speed = 0.0f;  // 轮缘速度，m/s
    float drive_speed = 0.0f;   // 驱动电机角速度，rad/s，未叠加安装方向
    float steer_angle = 0.0f;   // 相对车体 +X 的舵角，rad，未叠加零位与安装方向
};

// vx、vy 为车体坐标速度 m/s，wz 为 rad/s；零速轮保留 targets 中原有舵角。
// 首次使用 targets 应初始化为 {}，或将舵角初始化为实际轮向。
// 参数无效返回 false，保持 targets 不变；本接口仅计算，不发送电机指令。
bool Chassis_Calculate(const ChassisGeometry &geometry, float vx, float vy, float wz,
                       ChassisWheelTarget (&targets)[CHASSIS_WHEEL_COUNT]);

// 全局速度转车体速度后解算；yaw 为车体 +X 相对全局 +X 的角度，rad，逆时针为正。
bool Chassis_CalculateGlobal(const ChassisGeometry &geometry,
                             float global_vx, float global_vy, float wz, float yaw,
                             ChassisWheelTarget (&targets)[CHASSIS_WHEEL_COUNT]);

// 在 Calculate/CalculateGlobal 后调用，actual_angles 必须来自真实反馈并已转为车体舵角。
// 将非零速目标选到实际角附近的连续角，超过90度则反转轮速；零速保留输入目标角。
// linear_speed 与 drive_speed 优化后均可为负；无效输入返回 false 且不修改 targets。
bool Chassis_Optimize(const float (&actual_angles)[CHASSIS_WHEEL_COUNT],
                      ChassisWheelTarget (&targets)[CHASSIS_WHEEL_COUNT]);

// 原子更新车体速度指令：vx/vy m/s、wz rad/s；非有限值拒绝并保留旧指令。
// 配置未确认、反馈未就绪或故障锁定时拒绝非零命令；故障只可重启解除。无额外速度限幅。
bool Chassis_SetBodyVelocity(float vx, float vy, float wz);
// RTOS任务中仅初始化一次；配置默认锁定，不使能8台；true仅表示使能已入队。
bool Chassis_Init();
// 单一控制任务持续周期调用（包括初始化失败后，以重试失能入队）。
// 使能后最多等待1s完整反馈，期间零运动；就绪后100ms反馈超时/状态异常立即锁定。
// 故障停止驱动、转向零增益零转矩并失能；零转矩不是制动，断CAN须电机通信超时保护。
void Chassis_Loop();

#endif // H7_BSP_CHASSIS_H

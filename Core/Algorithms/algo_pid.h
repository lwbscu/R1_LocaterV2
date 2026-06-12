#ifndef ALGO_PID_H
#define ALGO_PID_H

#include <stdint.h>
#include <math.h>
#define PID_MEDIAN_SIZE 5
// === 标准 PID (速度环) ===
typedef struct {
    float kp, ki, kd;
    float max_out;
    float max_int;
    // --- 微分项的中值滤波 Buffer ---
    float deriv_buf[PID_MEDIAN_SIZE];
    uint8_t deriv_idx;   // derivativeIndex
    uint8_t deriv_count; // derivativeCount

    // --- 输入项的低通滤波参数 (原版 alpha_meas) ---
    float alpha_meas;    // 如果 > 0 则启用内部低通
    float meas_lp;       // 内部滤波状态值
    // 状态
    float error_int;
    float error_last;
    float output;
} PID_t;

void Algo_PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_int, float alpha_meas);
void Algo_PID_Reset(PID_t *pid);
float Algo_PID_Compute(PID_t *pid, float target, float measure);

// === 增强型位置 PD (PosPD - 带阻尼/摩擦/死区) ===
// 对应原 PosPD 结构
typedef struct {
    float kp;       // 刚度
    float kv;       // 阻尼 (抑制速度)
    float kc;       // 库伦摩擦补偿
    float deadband; // 死区 (deg)
    float target_pos;
} PosPD_t;

void Algo_PosPD_Init(PosPD_t *pd, float kp, float kv, float kc, float deadband);
void Algo_PosPD_SetTarget(PosPD_t *pd, float target);
// 返回目标速度 (RPM)，用于串级控制
float Algo_PosPD_Compute_Vref(PosPD_t *pd, float measure_pos, float measure_rpm, float max_rpm);

#endif // ALGO_PID_H
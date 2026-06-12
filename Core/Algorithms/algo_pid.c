#include "algo_pid.h"
#include <string.h>
#include <stdlib.h>

static float f_clamp(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

static float f_sign(float val) {
    return (val > 0.0f) ? 1.0f : ((val < 0.0f) ? -1.0f : 0.0f);
}
// 比较函数 (用于 qsort)
static int cmpf(const void *a, const void *b) {
    float d = (*(const float*)a) - (*(const float*)b);
    return (d < 0) ? -1 : (d > 0);
}

// 中值滤波算法
static float calc_median(float arr[], int n) {
    if (n <= 0) return 0.0f;
    float tmp[PID_MEDIAN_SIZE];
    // 复制数据防止破坏原 buffer
    memcpy(tmp, arr, n * sizeof(float));
    // 排序
    qsort(tmp, n, sizeof(float), cmpf);
    int m = n / 2;
    // 偶数取平均，奇数取中间
    return (n % 2) ? tmp[m] : (tmp[m-1] + tmp[m]) * 0.5f;
}

// --- PID 实现 ---
void Algo_PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_int, float alpha_meas) {
    memset(pid, 0, sizeof(PID_t)); // 清空 buffer 和 索引
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max_out = max_out;
    pid->max_int = max_int;
    pid->alpha_meas = alpha_meas; // 原版 usr_pid.c 中的滤波系数

    Algo_PID_Reset(pid);
}

void Algo_PID_Reset(PID_t *pid) {
    pid->error_int = 0.0f;
    pid->error_last = 0.0f;
    pid->output = 0.0f;
    // 重置滤波状态
    pid->deriv_idx = 0;
    pid->deriv_count = 0;
    memset(pid->deriv_buf, 0, sizeof(pid->deriv_buf));
    pid->meas_lp = 0.0f;
}

float Algo_PID_Compute(PID_t *pid, float target, float measure) {
    float input_val = measure;

    // 1. 还原 usr_pid.c 的输入低通滤波逻辑
    if (pid->alpha_meas > 0.0f) {
        // 若是第一次运行(meas_lp为0)，直接赋值防止爬升延迟，或者接受一次延迟
        // 原版代码逻辑：
        // if (pid->alpha_meas <= 0.0f) ... else pid->meas_lp += ...
        // 这里为了严谨：
        pid->meas_lp += pid->alpha_meas * (input_val - pid->meas_lp);
        input_val = pid->meas_lp;
    }

    // 2. 误差计算
    float error = target - input_val;

    // 3. 积分 (Clamp 策略)
    pid->error_int += error;
    pid->error_int = f_clamp(pid->error_int, -pid->max_int, pid->max_int);

    // 4. 微分 + 中值滤波 (核心防抖逻辑)
    float deriv = error - pid->error_last;

    // 压入环形缓冲区
    pid->deriv_buf[pid->deriv_idx] = deriv;
    if (pid->deriv_count < PID_MEDIAN_SIZE) {
        pid->deriv_count++;
    }
    pid->deriv_idx++;
    if (pid->deriv_idx >= PID_MEDIAN_SIZE) {
        pid->deriv_idx = 0;
    }

    // 计算中值
    float deriv_med = calc_median(pid->deriv_buf, pid->deriv_count);

    pid->error_last = error;

    // 5. 输出计算 (使用中值滤波后的微分)
    float out = (pid->kp * error) + (pid->ki * pid->error_int) + (pid->kd * deriv_med);
    out = f_clamp(out, -pid->max_out, pid->max_out);

    pid->output = out;
    return out;
}

// --- PosPD 实现 (保持不变，因为原版 usr_pid.c 的 PosPD 很简单) ---
void Algo_PosPD_Init(PosPD_t *pd, float kp, float kv, float kc, float deadband) {
    pd->kp = kp;
    pd->kv = kv;
    pd->kc = kc;
    pd->deadband = deadband;
    pd->target_pos = 0.0f;
}

void Algo_PosPD_SetTarget(PosPD_t *pd, float target) {
    pd->target_pos = target;
}

float Algo_PosPD_Compute_Vref(PosPD_t *pd, float measure_pos, float measure_rpm, float max_rpm) {
    float error = pd->target_pos - measure_pos;

    if (fabsf(error) < pd->deadband) {
        error = 0.0f;
    }

    float omega_dps = measure_rpm * 6.0f;

    // 这里传入的 measure_rpm 应该是经过外部滤波的，对应原 app_freertos.c 的 rpm_filt
    float v_dps = (pd->kp * error) - (pd->kv * omega_dps) + (pd->kc * f_sign(error));

    float v_rpm = v_dps / 6.0f;
    v_rpm = f_clamp(v_rpm, -max_rpm, max_rpm);

    return v_rpm;
}
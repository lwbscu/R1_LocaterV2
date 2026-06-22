#include "task_locater.h"
#include "FreeRTOS.h"
#include "task.h"
#include "driver_encoder.h"
#include "driver_dt35.h"
#include "driver_h30mini.h"
#include "driver_lidar_pose.h"
#include "driver_uart.h"
#include "locater_config.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

static Locater_State_t s_locater_state;
static float s_encoder_x_cm = LOCATER_INITIAL_X_CM;
static float s_encoder_y_cm = LOCATER_INITIAL_Y_CM;
static float s_encoder_prev_yaw_deg = LOCATER_INITIAL_YAW_DEG;
static bool s_encoder_prev_yaw_ready;
static float s_h30_x_cm = LOCATER_INITIAL_X_CM;
static float s_h30_y_cm = LOCATER_INITIAL_Y_CM;
static float s_h30_vx_mps;
static float s_h30_vy_mps;
static uint32_t s_h30_last_packet_count;
static uint32_t s_h30_last_integrate_ms;
static float s_lidar_x_cm = LOCATER_INITIAL_X_CM;
static float s_lidar_y_cm = LOCATER_INITIAL_Y_CM;
static float s_lidar_yaw_deg = LOCATER_INITIAL_YAW_DEG;
static float s_pos_x_cm = LOCATER_INITIAL_X_CM;
static float s_pos_y_cm = LOCATER_INITIAL_Y_CM;
static float s_pos_yaw_deg = LOCATER_INITIAL_YAW_DEG;
static float s_lidar_anchor_x_cm = LOCATER_INITIAL_X_CM;
static float s_lidar_anchor_y_cm = LOCATER_INITIAL_Y_CM;
static float s_lidar_anchor_yaw_deg = LOCATER_INITIAL_YAW_DEG;
static float s_anchor_encoder_x_cm = LOCATER_INITIAL_X_CM;
static float s_anchor_encoder_y_cm = LOCATER_INITIAL_Y_CM;
static float s_anchor_h30_yaw_deg = LOCATER_INITIAL_YAW_DEG;
static uint32_t s_lidar_anchor_packet_count;
static bool s_lidar_anchor_ready;
static float s_yaw_zero_deg;
static bool s_yaw_zero_ready;
static volatile bool s_zero_requested;

static float angle_normal_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle <= -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

static float angle_diff_deg(float now, float before)
{
    return angle_normal_deg(now - before);
}

static float angle_to_rad(float angle)
{
    return angle * (LOCATER_PI / 180.0f);
}

static void get_start_pose(float *x_cm, float *y_cm, float *yaw_deg)
{
    if (x_cm == NULL || y_cm == NULL || yaw_deg == NULL) {
        return;
    }

#if LOCATER_HOST_START_SIDE == LOCATER_HOST_START_SIDE_BLUE
    *x_cm = LOCATER_HOST_BLUE_START_X_CM;
    *y_cm = LOCATER_HOST_BLUE_START_Y_CM;
    *yaw_deg = LOCATER_HOST_BLUE_START_YAW_DEG;
#else
    *x_cm = LOCATER_HOST_RED_START_X_CM;
    *y_cm = LOCATER_HOST_RED_START_Y_CM;
    *yaw_deg = LOCATER_HOST_RED_START_YAW_DEG;
#endif
}

static float apply_deadband(float value, float deadband)
{
    if (fabsf(value) <= deadband) {
        return 0.0f;
    }
    return value;
}

static float h30_corrected_yaw_deg(const H30Mini_Data_t *h30)
{
    const float raw_yaw = (h30 != NULL && h30->has_attitude)
                              ? h30->yaw_deg
                              : LOCATER_INITIAL_YAW_DEG;
    float yaw = angle_normal_deg(raw_yaw + LOCATER_H30_YAW_OFFSET_DEG);

    if (s_yaw_zero_ready) {
        yaw = angle_diff_deg(yaw, s_yaw_zero_deg);
    }

    return yaw;
}

static bool consume_zero_request(void)
{
    bool requested;

    taskENTER_CRITICAL();
    requested = s_zero_requested;
    s_zero_requested = false;
    taskEXIT_CRITICAL();

    return requested;
}

static void locater_debug_command_callback(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U) {
        return;
    }

    if (len == 1U && (data[0] == 'R' || data[0] == 'r')) {
        Locater_RequestZero();
    }
}

static bool lidar_to_chassis_abs_pose(const LidarPose_Data_t *lidar,
                                      float *x_cm,
                                      float *y_cm,
                                      float *yaw_deg)
{
    if (lidar == NULL || !lidar->valid || !lidar->has_pose ||
        x_cm == NULL || y_cm == NULL || yaw_deg == NULL) {
        return false;
    }

    const float yaw = lidar->yaw_deg;
    const float yaw_rad = angle_to_rad(yaw);
    const float sin_yaw = sinf(yaw_rad);
    const float cos_yaw = cosf(yaw_rad);

    *x_cm = lidar->sensor_x_cm -
            (LOCATER_LIDAR_OFFSET_X_CM * cos_yaw -
             LOCATER_LIDAR_OFFSET_Y_CM * sin_yaw) +
            LOCATER_LIDAR_OFFSET_X_CM;
    *y_cm = lidar->sensor_y_cm -
            (LOCATER_LIDAR_OFFSET_X_CM * sin_yaw +
             LOCATER_LIDAR_OFFSET_Y_CM * cos_yaw) +
            LOCATER_LIDAR_OFFSET_Y_CM;
    *yaw_deg = yaw;

#if LOCATER_LIDAR_POSE_IS_START_LOCAL
    float start_x_cm = 0.0f;
    float start_y_cm = 0.0f;
    float start_yaw_deg = 0.0f;
    const float local_x_cm = *x_cm;
    const float local_y_cm = *y_cm;

    get_start_pose(&start_x_cm, &start_y_cm, &start_yaw_deg);
    const float start_yaw_rad = angle_to_rad(start_yaw_deg);
    const float start_sin = sinf(start_yaw_rad);
    const float start_cos = cosf(start_yaw_rad);
    *x_cm = start_x_cm + local_x_cm * start_cos - local_y_cm * start_sin;
    *y_cm = start_y_cm + local_x_cm * start_sin + local_y_cm * start_cos;
    *yaw_deg = angle_normal_deg(*yaw_deg + start_yaw_deg);
#endif

    return true;
}

static bool lidar_is_online(const LidarPose_Data_t *lidar)
{
    if (lidar == NULL || !lidar->valid || !lidar->has_pose || lidar->last_update_ms == 0U) {
        return false;
    }

    return (HAL_GetTick() - lidar->last_update_ms) <= LOCATER_LIDAR_ONLINE_TIMEOUT_MS;
}

static void set_lidar_anchor(float lidar_x_cm, float lidar_y_cm, float lidar_yaw_deg,
                             uint32_t packet_count, float h30_yaw_deg)
{
    s_lidar_anchor_x_cm = lidar_x_cm;
    s_lidar_anchor_y_cm = lidar_y_cm;
    s_lidar_anchor_yaw_deg = lidar_yaw_deg;
    s_anchor_encoder_x_cm = s_encoder_x_cm;
    s_anchor_encoder_y_cm = s_encoder_y_cm;
    s_anchor_h30_yaw_deg = h30_yaw_deg;
    s_lidar_anchor_packet_count = packet_count;
    s_lidar_anchor_ready = true;
}

static void update_lidar_first_pose(float h30_yaw_deg,
                                    bool lidar_has_pose,
                                    uint32_t lidar_packet_count,
                                    float lidar_x_cm,
                                    float lidar_y_cm,
                                    float lidar_yaw_deg)
{
    if (lidar_has_pose) {
        s_lidar_x_cm = lidar_x_cm;
        s_lidar_y_cm = lidar_y_cm;
        s_lidar_yaw_deg = lidar_yaw_deg;

        if (!s_lidar_anchor_ready ||
            lidar_packet_count != s_lidar_anchor_packet_count) {
            set_lidar_anchor(lidar_x_cm,
                             lidar_y_cm,
                             lidar_yaw_deg,
                             lidar_packet_count,
                             h30_yaw_deg);
        }
    }

    if (s_lidar_anchor_ready) {
        s_pos_x_cm = s_lidar_anchor_x_cm + (s_encoder_x_cm - s_anchor_encoder_x_cm);
        s_pos_y_cm = s_lidar_anchor_y_cm + (s_encoder_y_cm - s_anchor_encoder_y_cm);
        s_pos_yaw_deg = s_lidar_anchor_yaw_deg +
                        angle_diff_deg(h30_yaw_deg, s_anchor_h30_yaw_deg);
    } else {
        s_pos_x_cm = s_encoder_x_cm;
        s_pos_y_cm = s_encoder_y_cm;
        s_pos_yaw_deg = h30_yaw_deg;
    }
}

static void reset_locater_origin(const H30Mini_Data_t *h30,
                                 bool lidar_has_pose,
                                 uint32_t lidar_packet_count,
                                 float lidar_x_cm,
                                 float lidar_y_cm,
                                 float lidar_yaw_deg)
{
    if (h30 != NULL && h30->has_attitude) {
        s_yaw_zero_deg = angle_normal_deg(h30->yaw_deg + LOCATER_H30_YAW_OFFSET_DEG);
        s_yaw_zero_ready = true;
    } else {
        s_yaw_zero_deg = 0.0f;
        s_yaw_zero_ready = false;
    }

    Driver_Encoder_Reset();

    s_encoder_x_cm = LOCATER_INITIAL_X_CM;
    s_encoder_y_cm = LOCATER_INITIAL_Y_CM;
    s_encoder_prev_yaw_deg = 0.0f;
    s_encoder_prev_yaw_ready = false;

    s_h30_x_cm = LOCATER_INITIAL_X_CM;
    s_h30_y_cm = LOCATER_INITIAL_Y_CM;
    s_h30_vx_mps = 0.0f;
    s_h30_vy_mps = 0.0f;
    s_h30_last_packet_count = (h30 != NULL) ? h30->packet_count : 0U;
    s_h30_last_integrate_ms = (h30 != NULL) ? h30->last_update_ms : 0U;

    const float h30_yaw_deg = h30_corrected_yaw_deg(h30);
    s_lidar_anchor_ready = false;
    s_lidar_anchor_packet_count = 0U;

    if (lidar_has_pose) {
        s_lidar_x_cm = lidar_x_cm;
        s_lidar_y_cm = lidar_y_cm;
        s_lidar_yaw_deg = lidar_yaw_deg;
        set_lidar_anchor(lidar_x_cm,
                         lidar_y_cm,
                         lidar_yaw_deg,
                         lidar_packet_count,
                         h30_yaw_deg);
    }

    update_lidar_first_pose(h30_yaw_deg,
                            lidar_has_pose,
                            lidar_packet_count,
                            lidar_x_cm,
                            lidar_y_cm,
                            lidar_yaw_deg);
}

static void update_h30_dead_reckoning(const H30Mini_Data_t *h30, float yaw_deg)
{
    if (h30 == NULL || !h30->valid || !h30->has_accel || h30->packet_count == s_h30_last_packet_count) {
        return;
    }

    const uint32_t now_ms = h30->last_update_ms;
    if (s_h30_last_integrate_ms == 0U) {
        s_h30_last_integrate_ms = now_ms;
        s_h30_last_packet_count = h30->packet_count;
        return;
    }

    uint32_t dt_ms = now_ms - s_h30_last_integrate_ms;
    s_h30_last_integrate_ms = now_ms;
    s_h30_last_packet_count = h30->packet_count;

    if (dt_ms == 0U || dt_ms > LOCATER_H30_MAX_INTEGRATE_DT_MS) {
        return;
    }

    float accel_x = (h30->accel_x_mps2 - LOCATER_H30_ACCEL_X_BIAS_MPS2) * LOCATER_H30_ACCEL_X_SCALE;
    float accel_y = (h30->accel_y_mps2 - LOCATER_H30_ACCEL_Y_BIAS_MPS2) * LOCATER_H30_ACCEL_Y_SCALE;

#if LOCATER_H30_ACCEL_X_REVERSED
    accel_x = -accel_x;
#endif
#if LOCATER_H30_ACCEL_Y_REVERSED
    accel_y = -accel_y;
#endif

    accel_x = apply_deadband(accel_x, LOCATER_H30_ACCEL_DEADBAND_MPS2);
    accel_y = apply_deadband(accel_y, LOCATER_H30_ACCEL_DEADBAND_MPS2);

    const float yaw_rad = angle_to_rad(yaw_deg);
    const float cos_yaw = cosf(yaw_rad);
    const float sin_yaw = sinf(yaw_rad);
    const float world_accel_x = accel_x * cos_yaw - accel_y * sin_yaw;
    const float world_accel_y = accel_x * sin_yaw + accel_y * cos_yaw;
    const float dt_s = (float)dt_ms * 0.001f;

    s_h30_vx_mps += world_accel_x * dt_s;
    s_h30_vy_mps += world_accel_y * dt_s;

    float decay = 1.0f - LOCATER_H30_VELOCITY_DECAY_PER_S * dt_s;
    if (decay < 0.0f) {
        decay = 0.0f;
    } else if (decay > 1.0f) {
        decay = 1.0f;
    }
    s_h30_vx_mps *= decay;
    s_h30_vy_mps *= decay;

    s_h30_x_cm += s_h30_vx_mps * dt_s * 100.0f;
    s_h30_y_cm += s_h30_vy_mps * dt_s * 100.0f;
}

static void update_encoder_odometry(const Encoder_Snapshot_t *encoder, float yaw_deg,
                                    float *dis_p_mm, float *dis_q_mm)
{
    if (encoder == NULL || dis_p_mm == NULL || dis_q_mm == NULL) {
        return;
    }

    const int32_t enc_p = encoder->x_delta_count;
    const int32_t enc_q = encoder->y_delta_count;
    const float dis_p_raw = (abs(enc_p) > LOCATER_ENCODER_NOISE_THRESHOLD)
                                ? (float)enc_p * LOCATER_ENCODER_SCALE_P_MM
                                : 0.0f;
    const float dis_q_raw = (abs(enc_q) > LOCATER_ENCODER_NOISE_THRESHOLD)
                                ? -(float)enc_q * LOCATER_ENCODER_SCALE_Q_MM
                                : 0.0f;
    const float dis_p_base = dis_p_raw + dis_q_raw * LOCATER_ENCODER_CROSS_P_FROM_Q;
    const float dis_q_base = dis_q_raw + dis_p_raw * LOCATER_ENCODER_CROSS_Q_FROM_P;
    float d_yaw = 0.0f;

    if (s_encoder_prev_yaw_ready) {
        d_yaw = angle_diff_deg(yaw_deg, s_encoder_prev_yaw_deg);
    } else {
        s_encoder_prev_yaw_ready = true;
    }
    s_encoder_prev_yaw_deg = yaw_deg;

    const float correction = LOCATER_ENCODER_OFFSET_L_MM *
                             angle_to_rad(d_yaw) *
                             LOCATER_INV_SQRT2 *
                             LOCATER_ENCODER_ROT_COMP_GAIN;
    const float live_dis_p = dis_p_base - correction;
    const float live_dis_q = dis_q_base + correction;
    const float angle_cal = angle_normal_deg(yaw_deg + LOCATER_ENCODER_OFFSET_ANGLE_DEG);
    const float angle_rad = angle_to_rad(angle_cal);
    float dx_mm = live_dis_p * cosf(angle_rad) - live_dis_q * sinf(angle_rad);
    float dy_mm = live_dis_p * sinf(angle_rad) + live_dis_q * cosf(angle_rad);

    dx_mm *= LOCATER_ENCODER_POS_X_CORR;
    dy_mm *= LOCATER_ENCODER_POS_Y_CORR;

    s_encoder_x_cm += dx_mm * 0.1f;
    s_encoder_y_cm += dy_mm * 0.1f;

    *dis_p_mm = live_dis_p;
    *dis_q_mm = live_dis_q;
}

void Locater_GetState(Locater_State_t *state)
{
    if (state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *state = s_locater_state;
    taskEXIT_CRITICAL();
}

void Locater_RequestZero(void)
{
    s_zero_requested = true;
}

void StartLocaterTask(void *argument)
{
    (void)argument;

    Driver_Encoder_Init();
    Driver_H30Mini_Init();
    Driver_LidarPose_Init();
    Driver_UART_RegisterDebugCallback(locater_debug_command_callback);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LOCATER_TASK_PERIOD_MS);

    for (;;) {
        Encoder_Snapshot_t encoder = {0};
        H30Mini_Data_t h30 = {0};
        LidarPose_Data_t lidar = {0};
        DT35_Data_t dt35 = {0};
        float encoder_dis_p_mm = 0.0f;
        float encoder_dis_q_mm = 0.0f;

        Driver_Encoder_GetSnapshot(&encoder);
        Driver_H30Mini_GetData(&h30);
        Driver_H30Mini_Service();
        Driver_LidarPose_GetData(&lidar);
        Driver_DT35_GetData(&dt35);

        float h30_yaw_deg = h30_corrected_yaw_deg(&h30);

        update_h30_dead_reckoning(&h30, h30_yaw_deg);
        update_encoder_odometry(&encoder, h30_yaw_deg, &encoder_dis_p_mm, &encoder_dis_q_mm);

        float lidar_abs_x_cm = 0.0f;
        float lidar_abs_y_cm = 0.0f;
        float lidar_abs_yaw_deg = 0.0f;
        const bool lidar_has_pose = lidar_to_chassis_abs_pose(&lidar,
                                                              &lidar_abs_x_cm,
                                                              &lidar_abs_y_cm,
                                                              &lidar_abs_yaw_deg);
        const bool lidar_online = lidar_has_pose && lidar_is_online(&lidar);

        if (consume_zero_request()) {
            reset_locater_origin(&h30,
                                 lidar_has_pose,
                                 lidar.packet_count,
                                 lidar_abs_x_cm,
                                 lidar_abs_y_cm,
                                 lidar_abs_yaw_deg);
            h30_yaw_deg = h30_corrected_yaw_deg(&h30);
            encoder = (Encoder_Snapshot_t){0};
            encoder_dis_p_mm = 0.0f;
            encoder_dis_q_mm = 0.0f;
        } else {
            update_lidar_first_pose(h30_yaw_deg,
                                    lidar_has_pose,
                                    lidar.packet_count,
                                    lidar_abs_x_cm,
                                    lidar_abs_y_cm,
                                    lidar_abs_yaw_deg);
        }

        Locater_State_t next_state = {
            .tick_ms = HAL_GetTick(),
            .x_raw_count = encoder.x_raw_count,
            .y_raw_count = encoder.y_raw_count,
            .x_delta_count = encoder.x_delta_count,
            .y_delta_count = encoder.y_delta_count,
            .x_total_count = encoder.x_total_count,
            .y_total_count = encoder.y_total_count,
            .x_index_seen = encoder.x_index_seen,
            .y_index_seen = encoder.y_index_seen,
            .x_pulse_seen = encoder.x_pulse_seen,
            .y_pulse_seen = encoder.y_pulse_seen,
            .h30_valid = h30.valid,
            .h30_has_attitude = h30.has_attitude,
            .h30_has_accel = h30.has_accel,
            .h30_packet_count = h30.packet_count,
            .h30_rx_byte_count = h30.rx_byte_count,
            .h30_crc_error_count = h30.crc_error_count,
            .h30_frame_error_count = h30.frame_error_count,
            .h30_last_update_ms = h30.last_update_ms,
            .lidar_valid = lidar.valid && lidar.has_pose,
            .lidar_online = lidar_online,
            .lidar_packet_count = lidar.packet_count,
            .lidar_rx_byte_count = lidar.rx_byte_count,
            .lidar_checksum_error_count = lidar.checksum_error_count,
            .lidar_frame_error_count = lidar.frame_error_count,
            .lidar_last_update_ms = lidar.last_update_ms,
            .x_cm = s_pos_x_cm,
            .y_cm = s_pos_y_cm,
            .yaw_deg = s_pos_yaw_deg,
            .h30_yaw_deg = h30_yaw_deg,
            .h30_x_cm = s_h30_x_cm,
            .h30_y_cm = s_h30_y_cm,
            .h30_vx_mps = s_h30_vx_mps,
            .h30_vy_mps = s_h30_vy_mps,
            .h30_accel_x_mps2 = h30.accel_x_mps2,
            .h30_accel_y_mps2 = h30.accel_y_mps2,
            .h30_gyro_z_dps = h30.gyro_z_dps,
            .encoder_x_cm = s_encoder_x_cm,
            .encoder_y_cm = s_encoder_y_cm,
            .encoder_dis_p_mm = encoder_dis_p_mm,
            .encoder_dis_q_mm = encoder_dis_q_mm,
            .calib_x_cm = s_encoder_x_cm,
            .calib_y_cm = s_encoder_y_cm,
            .calib_yaw_deg = h30_yaw_deg,
            .lidar_x_cm = s_lidar_x_cm,
            .lidar_y_cm = s_lidar_y_cm,
            .lidar_yaw_deg = s_lidar_yaw_deg,
            .dt35_1_valid = dt35.sensor_1.valid,
            .dt35_2_valid = dt35.sensor_2.valid,
            .dt35_1_raw = dt35.sensor_1.raw,
            .dt35_2_raw = dt35.sensor_2.raw,
            .dt35_1_ok_count = dt35.sensor_1.ok_count,
            .dt35_2_ok_count = dt35.sensor_2.ok_count,
            .dt35_1_error_count = dt35.sensor_1.error_count,
            .dt35_2_error_count = dt35.sensor_2.error_count,
            .dt35_1_last_update_ms = dt35.sensor_1.last_update_ms,
            .dt35_2_last_update_ms = dt35.sensor_2.last_update_ms,
            .dt35_1_mm = dt35.sensor_1.valid ? dt35.sensor_1.distance_mm : 0.0f,
            .dt35_2_mm = dt35.sensor_2.valid ? dt35.sensor_2.distance_mm : 0.0f,
        };

        taskENTER_CRITICAL();
        s_locater_state = next_state;
        taskEXIT_CRITICAL();

        vTaskDelayUntil(&last_wake, period);
    }
}

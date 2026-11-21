#ifndef AIM_CONTROL_H
#define AIM_CONTROL_H

#include <stdint.h>

/* 相机坐标采用有符号偏移量，和旧代码保持一致
 * 保留光学标定；Y方向遵循硬件校正方向
 * 下面的增益和限幅是保守初始值，不是最终调参结果
 */
#define AIM_CENTER_X           5.0f
#define AIM_CENTER_Y           12.0f
#define AIM_X_POSITIVE_DIR     1U
#define AIM_Y_POSITIVE_DIR     1U
#define AIM_KP_X               0.58f  /* 每像素对应的转速RPM */
#define AIM_KP_Y               0.18f//0.05是稳定的
/* Persistent boot/RESET defaults. X: single-KP PD (legacy boost retained); Y: near/far scheduled PD.
 * KPX/KPY use FAR_KP when scheduled; KD uses NEAR_KD in either mode.
 * Gains are online-tunable via UART; online edits themselves remain RAM-only.
 */
#define AIM_NEAR_KP_X          AIM_KP_X
#define AIM_FAR_KP_X           AIM_KP_X
#define AIM_NEAR_KD_X          0.010f
#define AIM_FAR_KD_X           0.002f
#define AIM_SHAPE_X            0U
#define AIM_NEAR_KP_Y          AIM_KP_Y
#define AIM_FAR_KP_Y           0.24f
#define AIM_NEAR_KD_Y          0.003f
#define AIM_FAR_KD_Y           0.0005f
#define AIM_SHAPE_Y            1U
#define AIM_MAX_RPM_X          15.0f
#define AIM_MAX_RPM_Y          10.0f
#define AIM_MIN_RPM            1.0f   /* 驱动器只接受整数转速 */
#define AIM_SLEW_X             260.0f  /* 每秒转速接受的最大变化量 */
#define AIM_SLEW_Y             25.0f
#define AIM_BOOST_START_ERROR  7.0f  /* 单轴超出7的时候的时候速度额外增益，超出24的时候完全增益 */
#define AIM_BOOST_FULL_ERROR   24.0f
#define AIM_FAR_GAIN_BOOST     0.35f //0.15 /* 最大额外增加比例增益 */
#define AIM_FAR_ACCEL_BOOST    0.55f  //0.20//0.25// 仅加速阶段增强，减速不增强 */
#define AIM_STOP_ERROR         2.0f   /* 误差小于该像素值则停止 */
#define AIM_RESTART_ERROR      4.0f   /* 误差超出该阈值才重新启动运动 */
#define AIM_MOTOR_ACC_X        240U
#define AIM_MOTOR_ACC_Y        60U
#define AIM_OLED_PERIOD_MS     100U

typedef struct
{
    float speed;
    uint8_t settled;
} AimAxis;

typedef struct
{
    float previous_error;
    float filtered_rate;
    uint8_t have_error;
} AimDamping;

static void AimDamping_Reset(AimDamping *state)
{
    state->previous_error = 0.0f;
    state->filtered_rate = 0.0f;
    state->have_error = 0;
}

static void AimAxis_Reset(AimAxis *axis)
{
    axis->speed = 0.0f;
    axis->settled = 1;
}

/* Update once per new camera frame. D uses a low-pass filtered error rate.
 * Driver internal speed regulation is separate from this visual outer loop.
 */
static float AimAxis_UpdateCore(AimAxis *axis, float error, float kp,
                               float limit, float slew, float dt, float damping, uint8_t boost_p)
{
    float magnitude = error < 0.0f ? -error : error;
    float desired;
    float step;
    float boost = 0.0f;
    float current_magnitude;
    float desired_magnitude;

    if (magnitude <= AIM_STOP_ERROR)
    {
        AimAxis_Reset(axis);
        return 0.0f;
    }
    if (axis->settled && magnitude < AIM_RESTART_ERROR)
        return 0.0f;
    axis->settled = 0;

    /* 误差大的时候平缓提升增益，不在阈值处突变跳变 */
    if (magnitude > AIM_BOOST_START_ERROR)
    {
        boost = (magnitude - AIM_BOOST_START_ERROR) /
                (AIM_BOOST_FULL_ERROR - AIM_BOOST_START_ERROR);
        if (boost > 1.0f) boost = 1.0f;
    }
    desired = kp * error * (1.0f + (boost_p ? AIM_FAR_GAIN_BOOST * boost : 0.0f)) + damping;
    /* Damping brakes to zero but cannot reverse before error crosses zero. */
    if ((error > 0.0f && desired < 0.0f) ||
        (error < 0.0f && desired > 0.0f)) desired = 0.0f;
    if (desired > limit) desired = limit;
    if (desired < -limit) desired = -limit;
    /* 持续小误差时，保证最低转速，避免在停止带外被置0而无法收敛 */
    if (desired > 0.0f && desired < AIM_MIN_RPM) desired = AIM_MIN_RPM;
    if (desired < 0.0f && desired > -AIM_MIN_RPM) desired = -AIM_MIN_RPM;

    /* 换向时先减速到0，不直接反向 */
    if ((axis->speed > 0.0f && desired < 0.0f) ||
        (axis->speed < 0.0f && desired > 0.0f))
        desired = 0.0f;

    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
    step = slew * dt;
    current_magnitude = axis->speed < 0.0f ? -axis->speed : axis->speed;
    desired_magnitude = desired < 0.0f ? -desired : desired;
    if (desired_magnitude > current_magnitude)
        step *= 1.0f + AIM_FAR_ACCEL_BOOST * boost;
    if (desired > axis->speed + step)
        axis->speed += step;
    else if (desired < axis->speed - step)
        axis->speed -= step;
    else
        axis->speed = desired;

    return axis->speed;
}

/* Original behavior remains available when shaping is disabled. */
static float AimAxis_Update(AimAxis *axis, float error, float kp,
                            float limit, float slew, float dt)
{
    return AimAxis_UpdateCore(axis, error, kp, limit, slew, dt, 0.0f, 1U);
}

static float AimAxis_UpdateShaped(AimAxis *axis, AimDamping *state,
                                 float error, float near_kp, float far_kp,
                                 float kd, float limit, float slew, float dt)
{
    float magnitude = error < 0.0f ? -error : error;
    float blend = 0.0f;
    float kp, damping, rate;
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
    if (!state->have_error)
    {
        state->have_error = 1;
        state->filtered_rate = 0.0f;
    }
    else
    {
        rate = (error - state->previous_error) / dt;
        state->filtered_rate += dt / (0.06f + dt) * (rate - state->filtered_rate);
    }
    state->previous_error = error;
    if (magnitude > AIM_BOOST_START_ERROR)
    {
        blend = (magnitude - AIM_BOOST_START_ERROR) /
                (AIM_BOOST_FULL_ERROR - AIM_BOOST_START_ERROR);
        if (blend > 1.0f) blend = 1.0f;
        blend = blend * blend * (3.0f - 2.0f * blend);
    }
    kp = near_kp + (far_kp - near_kp) * blend;
    damping = kd * state->filtered_rate;
    if (damping > limit) damping = limit;
    if (damping < -limit) damping = -limit;
    return AimAxis_UpdateCore(axis, error, kp, limit, slew, dt, damping, 1U);
}

/* Single proportional gain plus filtered error derivative; kd=0 is original P. */
static float AimAxis_UpdatePD(AimAxis *axis, AimDamping *state,
                             float error, float kp, float kd,
                             float limit, float slew, float dt)
{
    if (kd == 0.0f)
    {
        AimDamping_Reset(state);
        return AimAxis_Update(axis, error, kp, limit, slew, dt);
    }
    if (dt > 0.1f) AimDamping_Reset(state);
    return AimAxis_UpdateShaped(axis, state, error, kp, kp, kd, limit, slew, dt);
}

/* Per-axis schedule: near/far endpoints, smooth transition, no extra P boost. */
static float AimAxis_ScheduleBlend(float error, float near_error, float far_error)
{
    float magnitude = error < 0.0f ? -error : error;
    float blend;
    if (magnitude <= near_error) return 0.0f;
    if (magnitude >= far_error) return 1.0f;
    if (far_error <= near_error) return 0.0f;
    blend = (magnitude - near_error) / (far_error - near_error);
    return blend * blend * (3.0f - 2.0f * blend);
}

static float AimAxis_UpdateScheduled(AimAxis *axis, AimDamping *state,
                      float error, float near_kp, float far_kp,
                      float near_kd, float far_kd, float near_error, float far_error,
                      float limit, float slew, float dt)
{
    float blend = AimAxis_ScheduleBlend(error, near_error, far_error);
    float kp = near_kp + (far_kp - near_kp) * blend;
    float kd = near_kd + (far_kd - near_kd) * blend;
    float rate, damping;
    if (dt > 0.1f) AimDamping_Reset(state);
    if (dt <= 0.0f) dt = 0.001f;
    if (dt > 0.1f) dt = 0.1f;
    if (!state->have_error)
    {
        state->have_error = 1;
        state->filtered_rate = 0.0f;
    }
    else
    {
        rate = (error - state->previous_error) / dt;
        state->filtered_rate += dt / (0.06f + dt) * (rate - state->filtered_rate);
    }
    state->previous_error = error;
    damping = kd * state->filtered_rate;
    if (damping > limit) damping = limit;
    if (damping < -limit) damping = -limit;
    return AimAxis_UpdateCore(axis, error, kp, limit, slew, dt, damping, 0U);
}

static int16_t AimAxis_Rpm(float speed)
{
    return speed >= 0.0f ? (int16_t)(speed + 0.5f)
                         : (int16_t)(speed - 0.5f);
}

#endif

#include "stm32f10x.h"
#include "motor.h"
#include "ps2.h"
#include "tim1_pwm.h"
#include "arm_config.h"
#include "Delay.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---------------------------------------------------------------------------
 *  摇杆 -> 小车速度映射方向
 *    上推 = 前进（Y 取反），右推 = 右横移。
 *    方向反了把下面两个 1<->0 对调即可。
 * ------------------------------------------------------------------------- */
#define PS2_Y_INVERT      1
#define PS2_X_INVERT      0
#define PS2_LOST_LIMIT    3

/* ============================================================
 * 小车：速度档
 * ============================================================ */
static uint8_t g_speed_half = 0;      /* 0 = 满速，1 = 半速 */

/* ============================================================
 * 机械臂状态
 * ============================================================ */
static float g_deg[SERVO_COUNT];        /* 当前角度（平滑后） */
static float g_target[SERVO_COUNT];     /* 目标角度 */

static float g_pose_base;
static float g_pose_shoulder;
static float g_pose_wrist;

static uint8_t g_wrist_level = WRIST_LEVEL_COMP;   /* 腕部水平补偿 */
static uint8_t g_estop       = 0;                  /* 急停 */

/* 辅助对准状态机 */
typedef enum {
    AS_IDLE = 0, AS_DOWN, AS_GRIP, AS_HOLD, AS_RELEASE, AS_UP
} assist_state_t;

static assist_state_t g_assist = AS_IDLE;
static uint8_t        g_r2_prev = 0;
static uint16_t       g_l2_hold_ms = 0;
static uint8_t        g_sel_prev = 0;

/* ============================================================
 * 工具函数
 * ============================================================ */
static float my_absf(float v) { return (v < 0.0f) ? -v : v; }

static float approach(float cur, float target, float step)
{
    float d = target - cur;
    if (my_absf(d) <= step) return target;
    return cur + ((d > 0.0f) ? step : -step);
}

/* 末端（指尖）离地高度 mm */
static float tip_z_mm(float shoulder_deg, float wrist_deg)
{
    float a1 = shoulder_deg * M_PI / 180.0f;
    float a2 = (shoulder_deg + wrist_deg) * M_PI / 180.0f;
    return SHOULDER_Z_MM
           + ARM_LEN_MM   * sinf(a1)
           + WRIST_LEN_MM * sinf(a2);
}

/* 腕部水平补偿：让夹爪保持竖直朝下 */
static float wrist_level_for(float shoulder_deg)
{
    return Servo_ClampDeg(SERVO_WRIST, -shoulder_deg - 90.0f);
}

static void servo_write_all(void)
{
    Servo_SetAngle(SERVO_BASE,     g_deg[SERVO_BASE]);
    Servo_SetAngle(SERVO_SHOULDER, g_deg[SERVO_SHOULDER]);
    Servo_SetAngle(SERVO_WRIST,    g_deg[SERVO_WRIST]);
    Servo_SetAngle(SERVO_GRIPPER,  g_deg[SERVO_GRIPPER]);
}

static uint8_t servo_settled(void)
{
    uint8_t i;
    for (i = 0; i < SERVO_COUNT; i++)
    {
        if (my_absf(g_deg[i] - g_target[i]) > 0.5f) return 0;
    }
    return 1;
}

/* 设置目标姿态（带腕部补偿 + 末端限高） */
static void set_pose(float base, float shoulder, float wrist)
{
    float s = Servo_ClampDeg(SERVO_SHOULDER, shoulder);
    float w = g_wrist_level ? wrist_level_for(s)
                            : Servo_ClampDeg(SERVO_WRIST, wrist);

    if (tip_z_mm(s, w) < TIP_MIN_Z_MM)      /* 末端限高：不许穿地 */
    {
        s = g_target[SERVO_SHOULDER];
        w = g_target[SERVO_WRIST];
    }

    g_target[SERVO_BASE]     = Servo_ClampDeg(SERVO_BASE, base);
    g_target[SERVO_SHOULDER] = s;
    g_target[SERVO_WRIST]    = w;
}

/* 运动平滑：每周期最多走 MOTION_STEP_DEG */
static void motion_update(void)
{
    uint8_t i;
    for (i = 0; i < SERVO_COUNT; i++)
    {
        float d = g_target[i] - g_deg[i];
        if (d > MOTION_STEP_DEG)        g_deg[i] += MOTION_STEP_DEG;
        else if (d < -MOTION_STEP_DEG)  g_deg[i] -= MOTION_STEP_DEG;
        else                            g_deg[i] = g_target[i];
    }
    servo_write_all();
}

/* ============================================================
 * 辅助对准
 * ============================================================ */
static void assist_on_press(void)
{
    switch (g_assist)
    {
        case AS_IDLE:
            g_assist = AS_DOWN;
            g_pose_shoulder = g_target[SERVO_SHOULDER];
            break;
        case AS_HOLD:
            g_assist = AS_RELEASE;
            break;
        case AS_DOWN:
        case AS_GRIP:
        case AS_RELEASE:
        case AS_UP:
        default:
            g_assist = AS_IDLE;
            break;
    }
}

static void assist_update(void)
{
    const float step = ASSIST_RATE_DEG_PER_S * MOTION_PERIOD_MS / 1000.0f;

    switch (g_assist)
    {
        case AS_DOWN:
            g_target[SERVO_GRIPPER] = GRIP_OPEN_DEG;
            g_pose_shoulder = approach(g_pose_shoulder, PICK_DESCEND_SHOULDER, step);
            set_pose(g_pose_base, g_pose_shoulder, g_pose_wrist);
            if (my_absf(g_pose_shoulder - PICK_DESCEND_SHOULDER) < 0.05f
                || tip_z_mm(g_target[SERVO_SHOULDER],
                            g_target[SERVO_WRIST]) <= TIP_MIN_Z_MM + 1.0f)
            {
                g_assist = AS_GRIP;
            }
            break;

        case AS_GRIP:
            set_pose(g_pose_base, PICK_DESCEND_SHOULDER, g_pose_wrist);
            if (servo_settled())
            {
                g_target[SERVO_GRIPPER] = GRIP_HOLD_DEG;
                g_assist = AS_HOLD;
            }
            else
            {
                g_target[SERVO_GRIPPER] = GRIP_CLOSE_DEG;
            }
            break;

        case AS_HOLD:
            g_target[SERVO_GRIPPER] = GRIP_HOLD_DEG;
            break;

        case AS_RELEASE:
            g_target[SERVO_GRIPPER] = GRIP_OPEN_DEG;
            if (servo_settled())
            {
                g_assist = AS_UP;
            }
            break;

        case AS_UP:
            g_target[SERVO_GRIPPER] = GRIP_OPEN_DEG;
            g_pose_shoulder = approach(g_pose_shoulder, PICK_AIM_SHOULDER, step);
            set_pose(g_pose_base, g_pose_shoulder, g_pose_wrist);
            if (my_absf(g_pose_shoulder - PICK_AIM_SHOULDER) < 0.05f)
            {
                g_assist = AS_IDLE;
            }
            break;

        case AS_IDLE:
        default:
            break;
    }
}

/* ============================================================
 * 手动控制（右摇杆 = 底座/肩，L1/R1 = 腕）
 * ============================================================ */
static void handle_manual(int8_t rx, int8_t ry, uint16_t buttons)
{
    float dt = MOTION_PERIOD_MS / 1000.0f;

    /* 右摇杆 X -> 底座回转（任何时候都有效） */
    if (rx > PS2_DEADZONE || rx < -PS2_DEADZONE)
    {
        float d = (float)rx / 128.0f * PS2_ARM_SCALE_DEG * dt;
        g_pose_base = Servo_ClampDeg(SERVO_BASE, g_pose_base + d);
    }

    /* 右摇杆 Y -> 肩举升（辅助中推杆 = 取消辅助） */
    if (ry > PS2_DEADZONE || ry < -PS2_DEADZONE)
    {
        if (g_assist != AS_IDLE) g_assist = AS_IDLE;
        float d = (float)ry / 128.0f * PS2_ARM_SCALE_DEG * dt;
        g_pose_shoulder = Servo_ClampDeg(SERVO_SHOULDER, g_pose_shoulder + d);
    }

    /* L1 / R1 -> 腕俯仰微调 */
    if (buttons & PSB_L1)
    {
        g_wrist_level = 0;
        g_pose_wrist = Servo_ClampDeg(SERVO_WRIST, g_pose_wrist + PS2_WRIST_STEP_DEG);
    }
    else if (buttons & PSB_R1)
    {
        g_wrist_level = 0;
        g_pose_wrist = Servo_ClampDeg(SERVO_WRIST, g_pose_wrist - PS2_WRIST_STEP_DEG);
    }
    else if (!g_wrist_level)
    {
        g_wrist_level = WRIST_LEVEL_COMP;
    }
}

/* ============================================================
 * 主函数
 * ============================================================ */
int main(void)
{
    int16_t vx, vy;
    uint8_t lx, ly, rx, ry;
    int8_t  arx, ary;
    uint16_t buttons;
    uint8_t lost = 0;
    uint8_t motors_on = 0;
    uint8_t arm_tick = 0;
    uint8_t i;

    Motor_Init();       /* 电机（保持 nSLEEP=0，等手柄连上再唤醒） */
    PS2_Init();
    Servo_Init();       /* 舵机 TIM1 */

    /* 软启动：开机缓慢走到收起姿态、夹爪张开 */
    g_deg[SERVO_BASE]     = 0.0f;
    g_deg[SERVO_SHOULDER] = SHOULDER_STOW;
    g_deg[SERVO_WRIST]    = WRIST_HOME;
    g_deg[SERVO_GRIPPER]  = GRIP_OPEN_DEG;
    for (i = 0; i < SERVO_COUNT; i++) g_target[i] = g_deg[i];
    g_pose_base     = 0.0f;
    g_pose_shoulder = SHOULDER_STOW;
    g_pose_wrist    = WRIST_HOME;
    servo_write_all();
    Delay_ms(800);

    while (1)
    {
        if (PS2_Read() == 0)                    /* 读到手柄 */
        {
            lost = 0;
            if (motors_on == 0)                 /* 首次成功读到手柄才唤醒电机 */
            {
                Motor_Enable();
                motors_on = 1;
            }

            PS2_GetLeftStick(&lx, &ly);
            PS2_GetRightStick(&rx, &ry);
            buttons = PS2_GetButton();

            arx = (int8_t)(rx - 128);           /* -128..127，中心 0 */
            ary = (int8_t)(ry - 128);

            /* SELECT 边沿：车速档切换 */
            {
                uint8_t sel = (buttons & PSB_SELECT) ? 1 : 0;
                if (sel && !g_sel_prev) g_speed_half = !g_speed_half;
                g_sel_prev = sel;
            }

            /* START：急停（车 + 臂 全冻结） */
            g_estop = (buttons & PSB_START) ? 1 : 0;

            /* ---- 小车：左摇杆 = 前进 / 横移（无原地自转） ---- */
            vx = PS2_StickToSpeed(ly, PS2_Y_INVERT);
            vy = PS2_StickToSpeed(lx, PS2_X_INVERT);
            if (g_speed_half) { vx = vx / 2; vy = vy / 2; }
            if (g_estop) Mecanum_Stop();
            else         Mecanum_Move(vx, vy, 0);

            /* ---- 机械臂：每 2 次循环（约 20ms）执行一次 ---- */
            if (++arm_tick >= 2)
            {
                arm_tick = 0;

                /* L2 长按 400ms：一键归位 */
                if (buttons & PSB_L2)
                {
                    g_l2_hold_ms += MOTION_PERIOD_MS;
                    if (g_l2_hold_ms >= PS2_AUTO_HOLD_MS)
                    {
                        g_assist = AS_IDLE;
                        g_pose_base     = 0.0f;
                        g_pose_shoulder = SHOULDER_STOW;
                        g_pose_wrist    = WRIST_HOME;
                        g_target[SERVO_GRIPPER] = GRIP_OPEN_DEG;
                        g_l2_hold_ms = 0;
                    }
                }
                else
                {
                    g_l2_hold_ms = 0;
                }

                /* R2 边沿：触发辅助对准 */
                {
                    uint8_t r2 = (buttons & PSB_R2) ? 1 : 0;
                    if (r2 && !g_r2_prev && !g_estop) assist_on_press();
                    g_r2_prev = r2;
                }

                if (g_estop)
                {
                    for (i = 0; i < SERVO_COUNT; i++) g_target[i] = g_deg[i];
                }
                else
                {
                    handle_manual(arx, ary, buttons);
                    assist_update();
                    set_pose(g_pose_base, g_pose_shoulder, g_pose_wrist);
                }

                motion_update();
            }
        }
        else                                    /* 没读到手柄 */
        {
            if (lost < 255) lost++;
            if (lost >= PS2_LOST_LIMIT) Mecanum_Stop();
            /* 机械臂保持不动 */
        }
        Delay_ms(10);
    }
}

/******************* (C) COPYRIGHT *****END OF FILE*****************************/

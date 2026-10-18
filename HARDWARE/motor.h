/**
  ******************************************************************************
  * @file    motor.h
  * @brief   麦克纳姆轮(麦轮)四轮小车 —— DRV8833 二线式驱动 头文件
  *
  *  硬件平台 : STM32F103C8T6 + Keil MDK5 + STM32 标准外设库(StdPeriph)
  *  驱动模块 : 2 块 DRV8833 双路驱动模块（每块 A/B 两个通道，共 4 路）
  *             左侧驱动模块 : A 通道 -> 左前轮 LF    B 通道 -> 左后轮 LR
  *             右侧驱动模块 : A 通道 -> 右前轮 RF    B 通道 -> 右后轮 RR
  *
  *  ==========================================================================
  *  ★★ DRV8833 接线模型（二线式，没有 PWMA/PWMB）★★
  *
  *      DRV8833 每个通道只有 IN1 / IN2 两根输入 + nSLEEP：
  *          IN1   IN2     输出
  *           0     0      滑行（停止）
  *          PWM    0      正转，转速 = 占空比
  *           0    PWM     反转，转速 = 占空比
  *           1     1      刹车（电机两端短接）
  *
  *      所以每个电机的 IN1 / IN2 都要能输出 PWM。本工程用"符号-幅值"驱动：
  *      正转只给 IN1 灌 PWM、IN2 拉低；反转只给 IN2 灌 PWM、IN1 拉低；
  *      停止两路都拉低（滑行）。四路统一 PWM1 + 高有效，CCR=0 就是 0% 占空比。
  *
  *  ==========================================================================
  *  ★★ 全车引脚占用总表（加新模块前先看这里，防止抢引脚）★★
  *
  *    【8 路 PWM】—— 每个电机 2 路（IN1 + IN2），用 TIM2 4 路 + TIM3 4 路
  *       PA0  TIM2_CH1  -> 左驱动 AIN1   (左前 LF 的 IN1)
  *       PA1  TIM2_CH2  -> 左驱动 AIN2   (左前 LF 的 IN2)
  *       PA2  TIM2_CH3  -> 左驱动 BIN1   (左后 LR 的 IN1)
  *       PA3  TIM2_CH4  -> 左驱动 BIN2   (左后 LR 的 IN2)
  *       PA6  TIM3_CH1  -> 右驱动 AIN1   (右前 RF 的 IN1)
  *       PA7  TIM3_CH2  -> 右驱动 AIN2   (右前 RF 的 IN2)
  *       PB0  TIM3_CH3  -> 右驱动 BIN1   (右后 RR 的 IN1)
  *       PB1  TIM3_CH4  -> 右驱动 BIN2   (右后 RR 的 IN2)
  *
  *    【驱动唤醒】
  *       PB5  (普通GPIO) -> 两块 DRV8833 的 nSLEEP 并联（高=唤醒，低=睡眠）
  *
  *    【PS2 手柄】（见 ps2.h）
  *       PB10 / PB11 / PB12 / PB13 : CS(ATT) / CLK / CMD / DAT
  *
  *    【舵机（TIM1，50Hz，500~2500us）】
  *       PA8  TIM1_CH1 -> 底座回转  SG90
  *       PA9  TIM1_CH2 -> 肩举升    MG995
  *       PA10 TIM1_CH3 -> 腕俯仰    SG90
  *       PA11 TIM1_CH4 -> 夹爪开合  SG90
  *
  *    【已空出】
  *       PA4 PA5 PA12 PA15  PB3 PB4 PB6 PB7 PB8 PB9 PB14 PB15  *  ==========================================================================
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

/* ############################################################################
 * #                      一、引脚宏定义（改接线只改这里）                      #
 * ############################################################################ */

/* PWM 频率 = 72MHz / (PSC+1) / (ARR+1) = 72MHz/72/1000 = 1kHz */
#define MOTOR_PWM_ARR            999        /* 自动重装载值（分辨率 1/1000） */
#define MOTOR_PWM_PSC            71         /* 预分频：72MHz/(71+1) = 1MHz */

/* ---------------------------------------------------------------------------
 * 8 个 PWM 引脚：每个电机两路（IN1 = A 路，IN2 = B 路，符号-幅值驱动）
 * ------------------------------------------------------------------------- */
/* ---- 左前轮 LF：IN1 = PA0(TIM2_CH1)，IN2 = PA1(TIM2_CH2) ---- */
#define MOTOR_LF_A_PORT          GPIOA
#define MOTOR_LF_A_PIN           GPIO_Pin_0
#define MOTOR_LF_B_PORT          GPIOA
#define MOTOR_LF_B_PIN           GPIO_Pin_1

/* ---- 左后轮 LR：IN1 = PA2(TIM2_CH3)，IN2 = PA3(TIM2_CH4) ---- */
#define MOTOR_LR_A_PORT          GPIOA
#define MOTOR_LR_A_PIN           GPIO_Pin_2
#define MOTOR_LR_B_PORT          GPIOA
#define MOTOR_LR_B_PIN           GPIO_Pin_3

/* ---- 右前轮 RF：IN1 = PA6(TIM3_CH1)，IN2 = PA7(TIM3_CH2) ---- */
#define MOTOR_RF_A_PORT          GPIOA
#define MOTOR_RF_A_PIN           GPIO_Pin_6
#define MOTOR_RF_B_PORT          GPIOA
#define MOTOR_RF_B_PIN           GPIO_Pin_7

/* ---- 右后轮 RR：IN1 = PB0(TIM3_CH3)，IN2 = PB1(TIM3_CH4) ---- */
#define MOTOR_RR_A_PORT          GPIOB
#define MOTOR_RR_A_PIN           GPIO_Pin_0
#define MOTOR_RR_B_PORT          GPIOB
#define MOTOR_RR_B_PIN           GPIO_Pin_1

/* ---------------------------------------------------------------------------
 * DRV8833 休眠唤醒脚 nSLEEP（板上可能标 STBY / SLEEP / EN）
 * 两块驱动模块的 nSLEEP 并联，统一由 PB5 控制
 * ------------------------------------------------------------------------- */
#define MOTOR_SLEEP_PORT         GPIOB
#define MOTOR_SLEEP_PIN          GPIO_Pin_5

/* ---------------------------------------------------------------------------
 * 单个轮子方向校正（镜像安装修正）
 *
 *   麦轮左右两侧电机是"镜像"安装的，同一电平下两侧物理转向相反，
 *   必须把其中一侧在软件里取反（INVERT=1），否则推前进会原地打转。
 *
 *   自测：车架空，推左摇杆向上（前进），看四个轮子：
 *     · 某个轮子反了 -> 只改那一个 INVERT（0<->1）；
 *     · 四个全反了（整车后退）-> 四个 INVERT 全部对调。
 * ------------------------------------------------------------------------- */
#define MOTOR_LF_INVERT          0      /* 左前轮 */
#define MOTOR_LR_INVERT          0      /* 左后轮 */
#define MOTOR_RF_INVERT          1      /* 右前轮：镜像安装，软件取反 */
#define MOTOR_RR_INVERT          1      /* 右后轮：镜像安装，软件取反 */

/* ---------------------------------------------------------------------------
 * 速度范围
 * ------------------------------------------------------------------------- */
#define MOTOR_SPEED_MAX          1000
#define MOTOR_PWM_MAX            1000

/* ############################################################################
 * #                          二、类型与接口                                   #
 * ############################################################################ */

/* 四个轮子的编号（顺序不要改，motor.c 的映射表按这个顺序） */
typedef enum
{
    MOTOR_LF = 0,       /* 左前轮 Left-Front  */
    MOTOR_LR = 1,       /* 左后轮 Left-Rear   */
    MOTOR_RF = 2,       /* 右前轮 Right-Front */
    MOTOR_RR = 3        /* 右后轮 Right-Rear  */
} Motor_ID_t;

/* ------------------------------ 底层电机接口 ------------------------------ */

/**
  * @brief  电机驱动初始化
  * @note   内部顺序（★ 防"上电狂转"的关键）：
  *           ① 先把 nSLEEP 拉低，让 DRV8833 睡眠
  *           ② 8 个 PWM 脚先配成"下拉输入"，把电平钉在低
  *           ③ 配 TIM2 / TIM3，四通道比较值全 0，启动定时器
  *           ④ 才把 8 个脚切成复用推挽输出
  *           ⑤ 全停；保持 nSLEEP=0，由 main 首次读到手柄后再 Motor_Enable() 唤醒
  *         使用电机前必须先调用一次
  */
void Motor_Init(void);

/**
  * @brief  设置单个轮子的速度（带方向）
  * @param  id    : 轮子编号 MOTOR_LF / MOTOR_LR / MOTOR_RF / MOTOR_RR
  * @param  speed : -MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX
  *                 正数 = 该轮正转(向前滚)，负数 = 反转，0 = 停止(滑行)
  * @note   符号-幅值：正转只给 IN1 灌 PWM、IN2 拉低；反转相反；停止两路都 0
  */
void Motor_SetSpeed(Motor_ID_t id, int16_t speed);

/**
  * @brief  单个轮子刹车（IN1/IN2 都 100%，DRV8833 内部短接电机两端）
  */
void Motor_Brake(Motor_ID_t id);

/**
  * @brief  四个轮子全部停止（滑行）
  */
void Motor_StopAll(void);

/**
  * @brief  唤醒 / 睡眠 DRV8833（nSLEEP 脚 PB5）
  * @note   Motor_Init() 不唤醒；由 main 首次读到手柄成功后调用 Motor_Enable()。
  */
void Motor_Enable(void);
void Motor_Disable(void);

/* --------------------------- 麦轮运动学接口 -------------------------------- */

/**
  * @brief  麦轮运动合成：把 (前后、横移、自转) 三个分量分配到四个轮子
  * @param  vx : 前后分量, >0 前进,  <0 后退
  * @param  vy : 横移分量, >0 右移,  <0 左移
  * @param  wz : 自转分量, >0 原地右转(顺时针), <0 原地左转(逆时针)
  *
  *         四轮转速分配公式（O 型辊子布局）：
  *             LF = vx - vy + wz
  *             LR = vx + vy + wz
  *             RF = vx + vy - wz
  *             RR = vx - vy - wz
  */
void Mecanum_Move(int16_t vx, int16_t vy, int16_t wz);

/* 下面是常用动作的快捷封装（内部就是调用 Mecanum_Move） */
void Mecanum_Stop(void);                          /* 停止           */
void Mecanum_Forward(uint16_t speed);             /* 前进           */
void Mecanum_Backward(uint16_t speed);            /* 后退           */
void Mecanum_StrafeLeft(uint16_t speed);          /* 左横移         */
void Mecanum_StrafeRight(uint16_t speed);         /* 右横移         */
void Mecanum_TurnLeft(uint16_t speed);            /* 原地左转       */
void Mecanum_TurnRight(uint16_t speed);           /* 原地右转       */
void Mecanum_DiagLeftFront(uint16_t speed);       /* 斜向左前       */
void Mecanum_DiagRightFront(uint16_t speed);      /* 斜向右前       */
void Mecanum_DiagLeftBack(uint16_t speed);        /* 斜向左后       */
void Mecanum_DiagRightBack(uint16_t speed);       /* 斜向右后       */

#endif /* __MOTOR_H */

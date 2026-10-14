/**
  ******************************************************************************
  * @file    motor.h
  * @brief   麦克纳姆轮(麦轮)四轮小车 —— 电机驱动模块 头文件
  *
  *  硬件平台 : STM32F103C8T6 + Keil MDK5 + STM32 标准外设库(StdPeriph)
  *  驱动模块 : 2 个双路电机驱动模块 (L298N / TB6612FNG / DRV8833 / BTS7960 ...)
  *             左侧驱动模块 : A 通道 -> 左前轮 LF    B 通道 -> 左后轮 LR
  *             右侧驱动模块 : A 通道 -> 右前轮 RF    B 通道 -> 右后轮 RR
  *  调速方式 : TIM3 的四路 PWM (CH1~CH4) -> 驱动模块的 ENA/ENB/PWM 脚
  *  方向控制 : 普通 GPIO 推挽输出        -> 驱动模块的 IN1/IN2 (AO1/AO2、BO1/BO2)
  *
  *  ★ 修改接线：只需修改本文件 "一、引脚宏定义" 以及 motor.c 中的 Motor_Hw[]
  *    映射表（两处必须保持一致）。
  *
  *  ==========================================================================
  *  ★★ 全车引脚占用总表（加新模块前先看这里，防止抢引脚）★★
  *       PA3       : DRV8833 的 STBY（拉高唤醒）
  *       PA4 / PA5 : 右前轮 RF 方向 IN1 / IN2
  *       PA6 / PA7 : TIM3_CH1 / CH2 -> 左前轮 LF / 左后轮 LR 的 PWM
  *       PB0 / PB1 : TIM3_CH3 / CH4 -> 右前轮 RF / 右后轮 RR 的 PWM
  *       PB8 / PB9 : 右后轮 RR 方向 IN1 / IN2
  *       PB12/13   : 左前轮 LF 方向 IN1 / IN2
  *       PB14/15   : 左后轮 LR 方向 IN1 / IN2
  *       PB5 / PB6 / PB10 / PB11 : PS2 手柄 CMD / DAT / ATT / CLK（见 ps2.h）
  *       PA13/PA14 : SWD 下载调试，别动
  *
  *      ⚠ 特别注意：PB0/PB1 就是右前/右后轮的 PWM，任何别的模块都不许再用，
  *        否则右轮会失控（PS2 的 ATT/CLK 以前就接在这里，已挪到 PB10/PB11）。
  *  ==========================================================================
  ******************************************************************************
  */
#ifndef __MOTOR_H
#define __MOTOR_H
 
#include "stm32f10x.h"
 
/* ############################################################################
 * #                      一、引脚宏定义（改接线只改这里）                      #
 * ############################################################################ */
 
/* ---------------------------------------------------------------------------
 * 1. PWM 调速引脚：4 路 PWM 全部用 TIM3 产生（同一频率，调速方便）
 *        TIM3_CH1 -> PA6 -> 左前轮 LF
 *        TIM3_CH2 -> PA7 -> 左后轮 LR
 *        TIM3_CH3 -> PB0 -> 右前轮 RF
 *        TIM3_CH4 -> PB1 -> 右后轮 RR
 *
 *    ★ 这 4 个脚是 TIM3 的固定复用引脚，一般不要改。
 *      若确实要改，请同时修改 motor.c 中 Motor_Hw[] 的 TIM_Channel 字段。
 * ------------------------------------------------------------------------- */
#define MOTOR_PWM_TIM            TIM3
#define MOTOR_PWM_TIM_CLK        RCC_APB1Periph_TIM3
 
#define MOTOR_PWM_GPIO_A         GPIOA
#define MOTOR_PWM_GPIO_A_CLK     RCC_APB2Periph_GPIOA
#define MOTOR_PWM_GPIO_A_PINS    (GPIO_Pin_6 | GPIO_Pin_7)     /* PA6 = CH1, PA7 = CH2 */
 
#define MOTOR_PWM_GPIO_B         GPIOB
#define MOTOR_PWM_GPIO_B_CLK     RCC_APB2Periph_GPIOB
#define MOTOR_PWM_GPIO_B_PINS    (GPIO_Pin_0 | GPIO_Pin_1)     /* PB0 = CH3, PB1 = CH4 */
 
/* ---------------------------------------------------------------------------
 * 2. 电机方向引脚：普通 GPIO 推挽输出，接驱动模块的 AO1/AO2、BO1/BO2
 *    （TB6612 上叫 AIN1/AIN2、BIN1/BIN2，原理一样：两脚电平决定正反转）
 * ------------------------------------------------------------------------- */
/* ---- 左侧驱动模块 ---- */
#define MOTOR_LF_IN1_PORT        GPIOB
#define MOTOR_LF_IN1_PIN         GPIO_Pin_12
#define MOTOR_LF_IN2_PORT        GPIOB
#define MOTOR_LF_IN2_PIN         GPIO_Pin_13
 
#define MOTOR_LR_IN1_PORT        GPIOB
#define MOTOR_LR_IN1_PIN         GPIO_Pin_14
#define MOTOR_LR_IN2_PORT        GPIOB
#define MOTOR_LR_IN2_PIN         GPIO_Pin_15
 
/* ---- 右侧驱动模块 ---- */
#define MOTOR_RF_IN1_PORT        GPIOA
#define MOTOR_RF_IN1_PIN         GPIO_Pin_4
#define MOTOR_RF_IN2_PORT        GPIOA
#define MOTOR_RF_IN2_PIN         GPIO_Pin_5
 
#define MOTOR_RR_IN1_PORT        GPIOB
#define MOTOR_RR_IN1_PIN         GPIO_Pin_8
#define MOTOR_RR_IN2_PORT        GPIOB
#define MOTOR_RR_IN2_PIN         GPIO_Pin_9
 
/* ---------------------------------------------------------------------------
 * 3. ★★ 单个轮子方向校正
 *
 *    为什么需要它：
 *      小车左右两侧的电机是"镜像"安装的（同一型号电机在左边和右边要转 180°
 *      才能把轮子朝外装），所以两侧电机加同样的电平，实际转动方向正好相反：
 *      左轮往前走的时候，右轮在往后走，车就会原地打转。
 *
 *    处理办法：
 *      把对应轮子的 INVERT 从 0 改成 1，软件里统一取反，不用拆线。
 *
 *    ★ 自测方法（车架空，30 秒）：
 *      1) 轮子离地，上电；
 *      2) 临时把 main 主循环里的 Mecanum_Move(...) 换成 Mecanum_Forward(400);
 *      3) 看四个轮子：都应该朝"让车往前走"的方向转
 *         （从车侧面看，轮子接触地面的那一侧往后运动）；
 *         · 某个轮子反了 -> 把该轮对应的 INVERT 改成 1；
 *         · 如果改成 1 之后反而变成反的 -> 说明你这台车是反的，改回 0。
 *      4) 四个轮子方向一致后，再依次测左移/右移/自转（麦轮还要看辊子方向）。
 * ------------------------------------------------------------------------- */
#define MOTOR_LF_INVERT          0      /* 左前轮：0 = 正常 */
#define MOTOR_LR_INVERT          0      /* 左后轮：0 = 正常 */
#define MOTOR_RF_INVERT          0      /* 右前轮：0 = 正常 */
#define MOTOR_RR_INVERT          0      /* 右后轮：0 = 正常 */
 
/* ---------------------------------------------------------------------------
 * 4. 速度范围
 *    MOTOR_SPEED_MAX : 用户层速度最大值（Mecanum_Move / Motor_SetSpeed 用）
 *    MOTOR_PWM_MAX   : PWM 比较寄存器最大值，等于 ARR+1（motor.c 中 ARR=999）
 * ------------------------------------------------------------------------- */
#define MOTOR_SPEED_MAX          1000
#define MOTOR_PWM_MAX            1000
 
/* DRV8833 休眠唤醒引脚 STBY */
#define MOTOR_STBY_PORT        GPIOA
#define MOTOR_STBY_PIN         GPIO_Pin_3
/* 说明：Motor_STBY_Init() 是 motor.c 里的 static 函数，不需要（也不应该）
 *       在头文件里声明，否则每个包含 motor.h 的 .c 都会多出一条无用的
 *       static 声明，容易触发编译器警告。这里只保留引脚宏。 */
 
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
  * @brief  电机初始化：配置 TIM3 四路 PWM 与 8 个方向 GPIO
  * @note   使用电机前必须先调用一次
  */
void Motor_Init(void);
 
/**
  * @brief  设置单个轮子的速度（带方向）
  * @param  id    : 轮子编号 MOTOR_LF / MOTOR_LR / MOTOR_RF / MOTOR_RR
  * @param  speed : -MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX
  *                 正数 = 该轮正转(向前滚)，负数 = 反转，0 = 停止
  */
void Motor_SetSpeed(Motor_ID_t id, int16_t speed);
 
/**
  * @brief  单个轮子刹车（两方向脚同电平 + PWM 占空比 0）
  */
void Motor_Brake(Motor_ID_t id);
 
/**
  * @brief  四个轮子全部停止
  */
void Motor_StopAll(void);
 
/* --------------------------- 麦轮运动学接口 -------------------------------- */
 
/**
  * @brief  麦轮运动合成：把 (前后、横移、自转) 三个分量分配到四个轮子
  * @param  vx : 前后分量, >0 前进,  <0 后退
  * @param  vy : 横移分量, >0 右移,  <0 左移
  * @param  wz : 自转分量, >0 原地右转(顺时针), <0 原地左转(逆时针)
  * @note   取值范围建议 -MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX；
  *         三个分量可以任意叠加，函数内部会自动等比限幅，不会超过 100% 占空比。
  *
  *         四轮转速分配公式（X 型辊子布局，行业通用）：
  *             LF = vx + vy + wz
  *             LR = vx - vy + wz
  *             RF = vx - vy - wz
  *             RR = vx + vy - wz
  */
void Mecanum_Move(int16_t vx, int16_t vy, int16_t wz);
 
/* 下面是八个常用动作的快捷封装（内部就是调用 Mecanum_Move） */
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

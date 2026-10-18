/**
 * @file    tim1_pwm.h
 * @brief   舵机驱动（TIM1 四通道 50Hz PWM）—— StdPeriph 标准外设库版
 *
 *   把原 HAL 版翻译成标准外设库。
 *   TIM1 CH1~CH4 = PA8 / PA9 / PA10 / PA11，分别对应：
 *     底座(SG90) / 肩(MG995) / 腕(SG90) / 夹爪(SG90)。
 */
#ifndef __TIM1_PWM_H
#define __TIM1_PWM_H

#include "stm32f10x.h"
#include "arm_config.h"

/**
  * @brief  初始化 TIM1 四通道舵机 PWM（50Hz，脉宽 500~2500us）
  * @note   系统时钟 72MHz；PSC=71 -> 1MHz，ARR=19999 -> 20ms。
  *         高级定时器必须开 MOE，否则引脚无波形。
  */
void Servo_Init(void);

/**
  * @brief  把关节角度夹到机械限位内（返回夹取后的角度）
  */
float Servo_ClampDeg(servo_id_t id, float deg);

/**
  * @brief  设置某个舵机的指令角（内部按 CAL_*_MID/SIGN 转脉宽后输出）
  * @param  id  : SERVO_BASE / SERVO_SHOULDER / SERVO_WRIST / SERVO_GRIPPER
  * @param  deg : 指令角（度），会自动夹到限位内
  */
void Servo_SetAngle(servo_id_t id, float deg);

#endif /* __TIM1_PWM_H */

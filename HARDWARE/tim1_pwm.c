/**
  ******************************************************************************
  * @file    tim1_pwm.c
  * @brief   舵机驱动 —— TIM1 四通道 50Hz PWM（StdPeriph 标准外设库版）
  *
  *  把原 HAL 版翻译成标准外设库：
  *    - TIM1 时基：72MHz / (71+1) = 1MHz，ARR = 19999 -> 周期 20ms（50Hz）
  *    - 四通道 PWM1 + 高有效，脉宽 500~2500us
  *    - PA8/PA9/PA10/PA11 = TIM1_CH1~CH4（默认复用，无需重映射）
  *    - ★ 高级定时器必须 TIM_CtrlPWMOutputs(TIM1, ENABLE) 打开 MOE，
  *      否则引脚上没有波形（对应 HAL 的 __HAL_TIM_MOE_ENABLE）。
  ******************************************************************************
  */
#include "tim1_pwm.h"

/* 关节角度夹到机械限位 */
float Servo_ClampDeg(servo_id_t id, float deg)
{
    switch (id)
    {
        case SERVO_BASE:
            if (deg < BASE_MIN)     return BASE_MIN;
            if (deg > BASE_MAX)     return BASE_MAX;
            return deg;
        case SERVO_SHOULDER:
            if (deg < SHOULDER_MIN) return SHOULDER_MIN;
            if (deg > SHOULDER_MAX) return SHOULDER_MAX;
            return deg;
        case SERVO_WRIST:
            if (deg < WRIST_MIN)    return WRIST_MIN;
            if (deg > WRIST_MAX)    return WRIST_MAX;
            return deg;
        case SERVO_GRIPPER:
            if (deg < GRIPPER_MIN)  return GRIPPER_MIN;
            if (deg > GRIPPER_MAX)  return GRIPPER_MAX;
            return deg;
        default:
            return deg;
    }
}

/* 指令角 -> 脉宽（us）：servo_deg = mid + sign * deg，再线性映射到 500~2500us */
static uint16_t Servo_AngleToPulse(servo_id_t id, float deg)
{
    const float mid[SERVO_COUNT]  = { CAL_BASE_MID,     CAL_SHOULDER_MID,
                                      CAL_WRIST_MID,    CAL_GRIPPER_MID };
    const float sign[SERVO_COUNT] = { CAL_BASE_SIGN,    CAL_SHOULDER_SIGN,
                                      CAL_WRIST_SIGN,   CAL_GRIPPER_SIGN };

    float d  = Servo_ClampDeg(id, deg);
    float sd = mid[id] + sign[id] * d;
    float t;

    if (sd < 0.0f)   sd = 0.0f;
    if (sd > 180.0f) sd = 180.0f;

    t = sd / 180.0f;
    return (uint16_t)(SERVO_PWM_MIN_US
                      + t * (SERVO_PWM_MAX_US - SERVO_PWM_MIN_US));
}

/**
  * @brief  初始化 TIM1 四通道舵机 PWM
  */
void Servo_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    /* 1. 开 TIM1（APB2）、GPIOA、AFIO 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* 2. PA8~PA11 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 |
                                    GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 3. 时基：1MHz 计数，周期 20000 -> 20ms（50Hz） */
    TIM_TimeBaseStructure.TIM_Period            = SERVO_PERIOD_US - 1;   /* 19999 */
    TIM_TimeBaseStructure.TIM_Prescaler         = 72 - 1;                /* 1us 步进 */
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* 4. 四通道统一 PWM1 + 高有效，中位 1500us
     *    ★ 必须显式关掉互补输出（TIM_OutputNState_Disable），
     *      否则 TIM1_CH1N/2N/3N = PA7/PB0/PB1 会被误开启，和电机脚冲突。 */
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = 1500;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /* 5. ★ 高级定时器必须开 MOE，否则引脚无波形 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* 6. 启动 TIM1 */
    TIM_Cmd(TIM1, ENABLE);
}

/**
  * @brief  设置某个舵机的指令角
  */
void Servo_SetAngle(servo_id_t id, float deg)
{
    uint16_t us = Servo_AngleToPulse(id, deg);

    switch ((uint8_t)id)
    {
        case SERVO_BASE:     TIM_SetCompare1(TIM1, us); break;
        case SERVO_SHOULDER: TIM_SetCompare2(TIM1, us); break;
        case SERVO_WRIST:    TIM_SetCompare3(TIM1, us); break;
        case SERVO_GRIPPER:  TIM_SetCompare4(TIM1, us); break;
        default: break;
    }
}

/******************* (C) COPYRIGHT *****END OF FILE*****************************/

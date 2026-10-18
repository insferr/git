/**
  ******************************************************************************
  * @file    motor.c
  * @brief   麦克纳姆轮(麦轮)四轮小车 —— DRV8833 二线式驱动（符号-幅值 PWM）
  *
  *  硬件平台 : STM32F103C8T6 + Keil MDK5 + STM32 标准外设库(StdPeriph)
  *
  *  ---------------------------------------------------------------------------
  *  小车俯视图（车头朝上）
  *
  *        左前 LF  ●───────────●  右前 RF
  *                 │   车头 ↑  │
  *                 │           │
  *        左后 LR  ●───────────●  右后 RR
  *
  *      左侧驱动模块：A 通道 -> LF    B 通道 -> LR
  *      右侧驱动模块：A 通道 -> RF    B 通道 -> RR
  *  ---------------------------------------------------------------------------
  *
  *  ★★ DRV8833 二线式驱动原理 ★★
  *
  *      DRV8833 每个通道只有 IN1 / IN2 两根输入（没有独立 PWM/EN 脚）：
  *          IN1   IN2     输出
  *           0     0      滑行（停止）
  *          PWM    0      正转，转速 = 占空比
  *           0    PWM     反转，转速 = 占空比
  *           1     1      刹车
  *
  *      所以每个电机两根 IN 都要能输出 PWM（共 8 路）。本工程用"符号-幅值"：
  *          正转：IN1(A) = PWM，IN2(B) = 0
  *          反转：IN1(A) = 0，IN2(B) = PWM
  *          停止：IN1 = IN2 = 0（滑行）
  *
  *      四路统一 PWM1 + 高有效：CCR=0 就是 0% 占空比（恒低），
  *      所以"停止"一定是 IN1/IN2 都低（滑行），绝不会上电乱转。
  *
  *      8 路 PWM 来自两个定时器（都是 1kHz）：
  *          TIM2: CH1 = PA0   CH2 = PA1   CH3 = PA2   CH4 = PA3
  *          TIM3: CH1 = PA6   CH2 = PA7   CH3 = PB0   CH4 = PB1
  *
  *  ★★ 上电顺序（防止"通电瞬间四轮狂转"）★★
  *      ① nSLEEP 先拉低  -> DRV8833 睡眠，输出高阻
  *      ② 8 个 PWM 脚先配成"下拉输入" -> 电平被钉在低
  *      ③ 定时器配好、比较值清零，才启动
  *      ④ 才把 8 个脚切成复用推挽输出
  *      ⑤ 全停后保持 nSLEEP=0，由 main 首次读到手柄后再唤醒
  ******************************************************************************
  */
#include "motor.h"

/* ############################################################################
 * #                     一、内部参数（一般不需要改）                           #
 * ############################################################################ */
#define MOTOR_PWM_ARR            999        /* 自动重装载值 */
#define MOTOR_PWM_PSC            71         /* 预分频：72MHz/(71+1)=1MHz 计数时钟 */

/* ############################################################################
 * #         二、硬件映射表：每轮 = A 路(IN1) + B 路(IN2)（符号-幅值）           #
 * ############################################################################
 *  成员含义：
 *      A_Port/A_Pin/A_TIM/A_Ch : 该轮 IN1 所在端口、引脚、定时器、通道
 *      B_Port/B_Pin/B_TIM/B_Ch : 该轮 IN2 所在端口、引脚、定时器、通道
 *      Invert                  : 0 = 正常；1 = 该轮电机镜像安装，软件取反
 * -------------------------------------------------------------------------- */
typedef struct
{
    GPIO_TypeDef *A_Port;
    uint16_t      A_Pin;
    uint8_t       A_TIM;
    uint8_t       A_Ch;
    GPIO_TypeDef *B_Port;
    uint16_t      B_Pin;
    uint8_t       B_TIM;
    uint8_t       B_Ch;
    uint8_t       Invert;
} Motor_HwTypeDef;

/* 定时器编号（自定义编号，避免结构体里直接塞 TIM_TypeDef* 不好维护） */
#define MOTOR_TMR_TIM2           0
#define MOTOR_TMR_TIM3           1

static const Motor_HwTypeDef Motor_Hw[4] =
{
    /* --- 0: 左前轮 LF —— IN1 = PA0(TIM2_CH1)   IN2 = PA1(TIM2_CH2) --- */
    { MOTOR_LF_A_PORT, MOTOR_LF_A_PIN, MOTOR_TMR_TIM2, TIM_Channel_1,
      MOTOR_LF_B_PORT, MOTOR_LF_B_PIN, MOTOR_TMR_TIM2, TIM_Channel_2,
      MOTOR_LF_INVERT },

    /* --- 1: 左后轮 LR —— IN1 = PA2(TIM2_CH3)   IN2 = PA3(TIM2_CH4) --- */
    { MOTOR_LR_A_PORT, MOTOR_LR_A_PIN, MOTOR_TMR_TIM2, TIM_Channel_3,
      MOTOR_LR_B_PORT, MOTOR_LR_B_PIN, MOTOR_TMR_TIM2, TIM_Channel_4,
      MOTOR_LR_INVERT },

    /* --- 2: 右前轮 RF —— IN1 = PA6(TIM3_CH1)   IN2 = PA7(TIM3_CH2) --- */
    { MOTOR_RF_A_PORT, MOTOR_RF_A_PIN, MOTOR_TMR_TIM3, TIM_Channel_1,
      MOTOR_RF_B_PORT, MOTOR_RF_B_PIN, MOTOR_TMR_TIM3, TIM_Channel_2,
      MOTOR_RF_INVERT },

    /* --- 3: 右后轮 RR —— IN1 = PB0(TIM3_CH3)   IN2 = PB1(TIM3_CH4) --- */
    { MOTOR_RR_A_PORT, MOTOR_RR_A_PIN, MOTOR_TMR_TIM3, TIM_Channel_3,
      MOTOR_RR_B_PORT, MOTOR_RR_B_PIN, MOTOR_TMR_TIM3, TIM_Channel_4,
      MOTOR_RR_INVERT },
};

/* ############################################################################
 * #                     三、小工具函数                                         #
 * ############################################################################ */

/** @brief 定时器编号 -> 定时器寄存器基址 */
static TIM_TypeDef *Motor_TimGet(uint8_t t)
{
    if (t == MOTOR_TMR_TIM2)
    {
        return TIM2;
    }
    return TIM3;
}

/** @brief 按"定时器编号 + 通道"写比较寄存器（也就是写该脚的占空比） */
static void Motor_PwmSet(uint8_t t, uint8_t ch, uint16_t value)
{
    TIM_TypeDef *TIMx = Motor_TimGet(t);

    switch (ch)
    {
        case TIM_Channel_1: TIM_SetCompare1(TIMx, value); break;
        case TIM_Channel_2: TIM_SetCompare2(TIMx, value); break;
        case TIM_Channel_3: TIM_SetCompare3(TIMx, value); break;
        case TIM_Channel_4: TIM_SetCompare4(TIMx, value); break;
        default: break;
    }
}

/**
  * @brief  配置一个定时器为 4 路 PWM 输出（1kHz）
  * @note   ★ 关键：四路统一用 PWM1 + 高有效，CCR=0 即 0% 占空比（恒低），
  *         保证"停止 = IN1/IN2 都低 = 滑行"，不会上电乱转。
  */
static void Motor_PwmTimInit(TIM_TypeDef *TIMx, uint32_t tim_clk)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    /* 1. 开该定时器时钟（TIM2/TIM3 都在 APB1） */
    RCC_APB1PeriphClockCmd(tim_clk, ENABLE);

    /* 2. 时基：1MHz 计数，周期 1000 -> 1kHz PWM */
    TIM_TimeBaseStructure.TIM_Period            = MOTOR_PWM_ARR;
    TIM_TimeBaseStructure.TIM_Prescaler         = MOTOR_PWM_PSC;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    /* 3. 四路统一 PWM1 + 高有效，初始占空比 0 */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OC1Init(TIMx, &TIM_OCInitStructure);
    TIM_OC2Init(TIMx, &TIM_OCInitStructure);
    TIM_OC3Init(TIMx, &TIM_OCInitStructure);
    TIM_OC4Init(TIMx, &TIM_OCInitStructure);

    /* 4. 使能 CCR 预装载：写比较值后下一个周期生效，避免波形毛刺 */
    TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIMx, ENABLE);

    /* 5. 先把四路比较值清零，再启动定时器（避免启动瞬间出高电平） */
    TIM_SetCompare1(TIMx, 0);
    TIM_SetCompare2(TIMx, 0);
    TIM_SetCompare3(TIMx, 0);
    TIM_SetCompare4(TIMx, 0);
    TIM_Cmd(TIMx, ENABLE);
}

/* ############################################################################
 * #   四、nSLEEP 唤醒脚（PB5）：上电先睡眠，一切就绪后才唤醒                    #
 * ############################################################################ */
static void Motor_SleepPinInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = MOTOR_SLEEP_PIN;         /* PB5 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_SLEEP_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(MOTOR_SLEEP_PORT, MOTOR_SLEEP_PIN);       /* ★ 先睡眠！ */
}

/** @brief 唤醒 DRV8833（nSLEEP = 1，允许输出） */
void Motor_Enable(void)
{
    GPIO_SetBits(MOTOR_SLEEP_PORT, MOTOR_SLEEP_PIN);
}

/** @brief 睡眠 DRV8833（nSLEEP = 0，输出高阻，电机立即停） */
void Motor_Disable(void)
{
    GPIO_ResetBits(MOTOR_SLEEP_PORT, MOTOR_SLEEP_PIN);
}

/* ############################################################################
 * #                      五、初始化                                            #
 * ############################################################################ */
/**
  * @brief  电机驱动初始化
  * @note   顺序见文件头"上电顺序"说明，核心是：
  *         先睡眠 -> 8 脚先下拉 -> 定时器 0 占空比 -> 才切复用 -> 全停
  *         （保持 nSLEEP=0，由 main 首次读到手柄后再 Motor_Enable() 唤醒）
  */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ---------- ① 第一件事：让 DRV8833 进睡眠，杜绝上电乱转 ---------- */
    Motor_SleepPinInit();

    /* ---------- ② 开 GPIO 与 AFIO 时钟 ------------------------------ */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);

    /* ---------- ③ 8 个 PWM 脚先配成"下拉输入"，把电平钉在低 ---------
     *   目的：从复位到程序跑起来这段时间，脚不会浮空，
     *         DRV8833 看到的永远是明确的低电平（0/0 = 滑行）。        */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = MOTOR_LF_A_PIN; GPIO_Init(MOTOR_LF_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_LF_B_PIN; GPIO_Init(MOTOR_LF_B_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_LR_A_PIN; GPIO_Init(MOTOR_LR_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_LR_B_PIN; GPIO_Init(MOTOR_LR_B_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_RF_A_PIN; GPIO_Init(MOTOR_RF_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_RF_B_PIN; GPIO_Init(MOTOR_RF_B_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_RR_A_PIN; GPIO_Init(MOTOR_RR_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_RR_B_PIN; GPIO_Init(MOTOR_RR_B_PORT, &GPIO_InitStructure);

    /* ---------- ④ 两个定时器：8 路 PWM，全部 1kHz，比较值 0 ---------- */
    Motor_PwmTimInit(TIM2, RCC_APB1Periph_TIM2);
    Motor_PwmTimInit(TIM3, RCC_APB1Periph_TIM3);

    /* ---------- ⑤ 现在才把 8 个脚切成"复用推挽输出"，交给定时器 ----- */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

    GPIO_InitStructure.GPIO_Pin = MOTOR_LF_A_PIN; GPIO_Init(MOTOR_LF_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_LF_B_PIN; GPIO_Init(MOTOR_LF_B_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_LR_A_PIN; GPIO_Init(MOTOR_LR_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_LR_B_PIN; GPIO_Init(MOTOR_LR_B_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_RF_A_PIN; GPIO_Init(MOTOR_RF_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_RF_B_PIN; GPIO_Init(MOTOR_RF_B_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_RR_A_PIN; GPIO_Init(MOTOR_RR_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MOTOR_RR_B_PIN; GPIO_Init(MOTOR_RR_B_PORT, &GPIO_InitStructure);

    /* ---------- ⑥ 全部停住；保持 nSLEEP=0，等手柄连上再唤醒 ---------- */
    Motor_StopAll();
    /* Motor_Enable() 移到 main：首次读到手柄成功后才唤醒 */
}

/* ############################################################################
 * #                   六、单轮调速（符号-幅值，双向可调）                       #
 * ############################################################################ */
/**
  * @brief  设置单个轮子的速度和方向
  * @param  id    : 轮子编号 MOTOR_LF / MOTOR_LR / MOTOR_RF / MOTOR_RR
  * @param  speed : -1000 ~ +1000；正数正转、负数反转、0 停止(滑行)
  *
  *  符号-幅值：正转只给 A 路(IN1)、反转只给 B 路(IN2)，另一路恒为 0
  */
void Motor_SetSpeed(Motor_ID_t id, int16_t speed)
{
    const Motor_HwTypeDef *hw = &Motor_Hw[id];
    int16_t  spd = speed;
    uint16_t duty_a;
    uint16_t duty_b;

    /* 1. 限幅，防止越界 */
    if (spd >  MOTOR_SPEED_MAX) spd =  MOTOR_SPEED_MAX;
    if (spd < -MOTOR_SPEED_MAX) spd = -MOTOR_SPEED_MAX;

    /* 2. 该轮如果装反了，这里统一取反 */
    if (hw->Invert)
    {
        spd = -spd;
    }

    /* 3. 计算符号-幅值占空比 */
    if (spd > 0)                        /* 正转：A(IN1) 给 PWM */
    {
        duty_a = (uint16_t)spd;
        duty_b = 0;
    }
    else if (spd < 0)                   /* 反转：B(IN2) 给 PWM */
    {
        duty_a = 0;
        duty_b = (uint16_t)(-spd);
    }
    else                                /* 停止：两路都 0 = 滑行 */
    {
        duty_a = 0;
        duty_b = 0;
    }

    /* 4. 写入对应通道的比较寄存器 */
    Motor_PwmSet(hw->A_TIM, hw->A_Ch, duty_a);
    Motor_PwmSet(hw->B_TIM, hw->B_Ch, duty_b);
}

/**
  * @brief  单个轮子刹车（IN1/IN2 都 100%，DRV8833 内部短接电机两端）
  */
void Motor_Brake(Motor_ID_t id)
{
    const Motor_HwTypeDef *hw = &Motor_Hw[id];

    Motor_PwmSet(hw->A_TIM, hw->A_Ch, MOTOR_PWM_MAX);
    Motor_PwmSet(hw->B_TIM, hw->B_Ch, MOTOR_PWM_MAX);
}

/**
  * @brief  四个轮子全部停止（滑行）
  */
void Motor_StopAll(void)
{
    Motor_SetSpeed(MOTOR_LF, 0);
    Motor_SetSpeed(MOTOR_LR, 0);
    Motor_SetSpeed(MOTOR_RF, 0);
    Motor_SetSpeed(MOTOR_RR, 0);
}

/* ############################################################################
 * #                       七、麦轮运动学解算                                   #
 * ############################################################################
 *
 *  记  vx = 前后分量 (>0 前进)
 *      vy = 横移分量 (>0 右移)
 *      wz = 自转分量 (>0 原地右转/顺时针)
 *
 *  分配公式（O 型辊子布局：左右两侧同向，即"左前-左后同向、右前-右后同向"）:
 *
 *      LF(左前) = vx - vy + wz
 *      LR(左后) = vx + vy + wz
 *      RF(右前) = vx + vy - wz
 *      RR(右后) = vx - vy - wz
 *
 *  注意：三个分量相加后单轮值可能超过量程，必须做等比限幅。
 * ########################################################################## */
/**
  * @brief  麦轮运动合成
  * @param  vx : 前后, >0 前进
  * @param  vy : 横移, >0 右移
  * @param  wz : 自转, >0 右转(顺时针)
  */
void Mecanum_Move(int16_t vx, int16_t vy, int16_t wz)
{
    int32_t w[4];
    int32_t max;
    int32_t vx32 = vx;
    int32_t vy32 = vy;
    int32_t wz32 = wz;
    uint8_t i;

    /* ---------- 1. 四轮转速分配（顺序：LF, LR, RF, RR） ---------- */
    w[MOTOR_LF] = vx32 - vy32 + wz32;      /* 左前 */
    w[MOTOR_LR] = vx32 + vy32 + wz32;      /* 左后 */
    w[MOTOR_RF] = vx32 + vy32 - wz32;      /* 右前 */
    w[MOTOR_RR] = vx32 - vy32 - wz32;      /* 右后 */

    /* ---------- 2. 找出最大绝对值 ---------- */
    max = 0;
    for (i = 0; i < 4; i++)
    {
        int32_t a = (w[i] >= 0) ? w[i] : -w[i];
        if (a > max)
        {
            max = a;
        }
    }

    /* ---------- 3. 等比限幅：超过量程就整体缩小，保持运动方向不变 ------ */
    if (max > MOTOR_SPEED_MAX)
    {
        for (i = 0; i < 4; i++)
        {
            w[i] = (w[i] * MOTOR_SPEED_MAX) / max;
        }
    }

    /* ---------- 4. 下发到四个电机 ---------- */
    Motor_SetSpeed(MOTOR_LF, (int16_t)w[MOTOR_LF]);
    Motor_SetSpeed(MOTOR_LR, (int16_t)w[MOTOR_LR]);
    Motor_SetSpeed(MOTOR_RF, (int16_t)w[MOTOR_RF]);
    Motor_SetSpeed(MOTOR_RR, (int16_t)w[MOTOR_RR]);
}

/* ############################################################################
 * #                       八、常用动作快捷封装                                 #
 * ############################################################################ */
void Mecanum_Stop(void)         { Mecanum_Move(0, 0, 0); }
void Mecanum_Forward(uint16_t speed)     { Mecanum_Move((int16_t)speed,  0,  0); }
void Mecanum_Backward(uint16_t speed)    { Mecanum_Move(-(int16_t)speed, 0,  0); }
void Mecanum_StrafeLeft(uint16_t speed)  { Mecanum_Move(0, -(int16_t)speed,  0); }   /* vy<0 = 左移 */
void Mecanum_StrafeRight(uint16_t speed) { Mecanum_Move(0,  (int16_t)speed,  0); }   /* vy>0 = 右移 */
void Mecanum_TurnLeft(uint16_t speed)    { Mecanum_Move(0, 0, -(int16_t)speed); }    /* wz<0 = 左转 */
void Mecanum_TurnRight(uint16_t speed)   { Mecanum_Move(0, 0,  (int16_t)speed); }    /* wz>0 = 右转 */

/*  斜向移动：前进/后退分量和横移分量都取 0.707 倍，合成速度大小不变 */
#define MECANUM_DIAG_K   707
void Mecanum_DiagLeftFront(uint16_t speed)
{
    int16_t v = (int16_t)(((int32_t)speed * MECANUM_DIAG_K) / 1000);
    Mecanum_Move(v, -v, 0);                     /* 前进 + 左移 = 左前 */
}
void Mecanum_DiagRightFront(uint16_t speed)
{
    int16_t v = (int16_t)(((int32_t)speed * MECANUM_DIAG_K) / 1000);
    Mecanum_Move(v, v, 0);                      /* 前进 + 右移 = 右前 */
}
void Mecanum_DiagLeftBack(uint16_t speed)
{
    int16_t v = (int16_t)(((int32_t)speed * MECANUM_DIAG_K) / 1000);
    Mecanum_Move(-v, -v, 0);                    /* 后退 + 左移 = 左后 */
}
void Mecanum_DiagRightBack(uint16_t speed)
{
    int16_t v = (int16_t)(((int32_t)speed * MECANUM_DIAG_K) / 1000);
    Mecanum_Move(-v, v, 0);                     /* 后退 + 右移 = 右后 */
}

/******************* (C) COPYRIGHT *****END OF FILE*****************************/

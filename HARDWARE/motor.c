/**
  ******************************************************************************
  * @file    motor.c
  * @brief   麦克纳姆轮四轮小车 —— 电机驱动模块（PWM 调速 + 方向控制 + 麦轮解算）
  *
  *  ┌─────────────── 小车俯视图（车头朝上） ───────────────┐
  *  │                                                      │
  *  │       左前 LF  ●───────────●  右前 RF                │
  *  │                │   车头 ↑  │                         │
  *  │                │          │                         │
  *  │       左后 LR  ●───────────●  右后 RR                │
  *  │                                                      │
  *  └──────────────────────────────────────────────────────┘
  *     左侧驱动模块：A 通道(LF) + B 通道(LR)
  *     右侧驱动模块：A 通道(RF) + B 通道(RR)
  *
  *  每个电机 3 根控制线：IN1、IN2 决定方向，PWM 决定转速。
  *      IN1=1, IN2=0, PWM=D  -> 正转，转速正比于 D
  *      IN1=0, IN2=1, PWM=D  -> 反转
  *      IN1=0, IN2=0, PWM=0  -> 停止
  ******************************************************************************
  */
#include "motor.h"
/* ############################################################################
 * #                     一、内部参数（一般不需要改）                           #
 * ############################################################################ */
/*  PWM 频率 = 72MHz / (PSC+1) / (ARR+1)
 *           = 72MHz / 72 / 1000 = 1kHz
 *  1kHz 对普通有刷直流电机 + L298N/TB6612 最合适（太高会有开关损耗和噪音）
 *  想让最高速更快/更慢，改 ARR 即可；想让 PWM 频率变高，改 PSC。          */
#define MOTOR_PWM_ARR            999        /* 自动重装载值 */
#define MOTOR_PWM_PSC            71         /* 预分频值：72MHz/(71+1)=1MHz 计数时钟 */
/* ############################################################################
 * #            二、电机硬件映射表（改接线时，与 motor.h 一起改）               #
 * ############################################################################
 *  成员含义：
 *      IN1_Port/IN1_Pin : 方向脚 1 所在端口和引脚
 *      IN2_Port/IN2_Pin : 方向脚 2 所在端口和引脚
 *      TIM_Channel      : 该轮 PWM 用的定时器通道（TIM3 的 CH1~CH4）
 *      Invert           : 0 = 正常；1 = 该轮电机接反了，软件取反
 * -------------------------------------------------------------------------- */
typedef struct
{
    GPIO_TypeDef *IN1_Port;
    uint16_t      IN1_Pin;
    GPIO_TypeDef *IN2_Port;
    uint16_t      IN2_Pin;
    uint8_t       TIM_Channel;
    uint8_t       Invert;
} Motor_HwTypeDef;
static const Motor_HwTypeDef Motor_Hw[4] =
{
    /* --- 0: 左前轮 LF —— 左侧驱动 A 通道，PWM = TIM3_CH1 = PA6 --- */
    { MOTOR_LF_IN1_PORT, MOTOR_LF_IN1_PIN, MOTOR_LF_IN2_PORT, MOTOR_LF_IN2_PIN,
      TIM_Channel_1, MOTOR_LF_INVERT },
    /* --- 1: 左后轮 LR —— 左侧驱动 B 通道，PWM = TIM3_CH2 = PA7 --- */
    { MOTOR_LR_IN1_PORT, MOTOR_LR_IN1_PIN, MOTOR_LR_IN2_PORT, MOTOR_LR_IN2_PIN,
      TIM_Channel_2, MOTOR_LR_INVERT },
    /* --- 2: 右前轮 RF —— 右侧驱动 A 通道，PWM = TIM3_CH3 = PB0 --- */
    { MOTOR_RF_IN1_PORT, MOTOR_RF_IN1_PIN, MOTOR_RF_IN2_PORT, MOTOR_RF_IN2_PIN,
      TIM_Channel_3, MOTOR_RF_INVERT },
    /* --- 3: 右后轮 RR —— 右侧驱动 B 通道，PWM = TIM3_CH4 = PB1 --- */
    { MOTOR_RR_IN1_PORT, MOTOR_RR_IN1_PIN, MOTOR_RR_IN2_PORT, MOTOR_RR_IN2_PIN,
      TIM_Channel_4, MOTOR_RR_INVERT },
};
/* ############################################################################
 * #                   STBY 唤醒函数（新增，其余不动）                          #
 * ############################################################################ */
static void Motor_STBY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = MOTOR_STBY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MOTOR_STBY_PORT, &GPIO_InitStructure);
    GPIO_SetBits(MOTOR_STBY_PORT, MOTOR_STBY_PIN); // 拉高STBY，唤醒DRV8833
}
/* ############################################################################
 * #                   三、底层：初始化 + 单轮调速                              #
 * ############################################################################ */
/**
  * @brief  电机驱动初始化
  *         - TIM3 CH1~CH4 输出 1kHz PWM
  *         - 8 个方向 GPIO 配置为推挽输出，初始全部输出低电平
  */
void Motor_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    uint8_t i;
 
    /* ========== 新增：STBY 唤醒 DRV8833 ========== */
    Motor_STBY_Init();
 
    /* ---------- 1. 开时钟 ------------------------------------------------ */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(MOTOR_PWM_TIM_CLK, ENABLE);
    /* ---------- 2. 4 路 PWM 引脚：复用推挽输出 AF_PP -------------------- */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = MOTOR_PWM_GPIO_A_PINS;      /* PA6 / PA7 */
    GPIO_Init(MOTOR_PWM_GPIO_A, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = MOTOR_PWM_GPIO_B_PINS;      /* PB0 / PB1 */
    GPIO_Init(MOTOR_PWM_GPIO_B, &GPIO_InitStructure);
    /* ---------- 3. 8 个方向引脚：推挽输出，初始拉低 ---------------------- */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    for (i = 0; i < 4; i++)
    {
        GPIO_InitStructure.GPIO_Pin = Motor_Hw[i].IN1_Pin;
        GPIO_Init(Motor_Hw[i].IN1_Port, &GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = Motor_Hw[i].IN2_Pin;
        GPIO_Init(Motor_Hw[i].IN2_Port, &GPIO_InitStructure);
        GPIO_ResetBits(Motor_Hw[i].IN1_Port, Motor_Hw[i].IN1_Pin);
        GPIO_ResetBits(Motor_Hw[i].IN2_Port, Motor_Hw[i].IN2_Pin);
    }
    /* ---------- 4. TIM3 时基：1MHz 计数时钟，周期 1000 ------------------ */
    TIM_TimeBaseStructure.TIM_Period        = MOTOR_PWM_ARR;    /* ARR */
    TIM_TimeBaseStructure.TIM_Prescaler     = MOTOR_PWM_PSC;    /* PSC */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInit(MOTOR_PWM_TIM, &TIM_TimeBaseStructure);
    /* ---------- 5. 四个通道都配置成 PWM1 模式 --------------------------- */
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse       = 0;                    /* 初始占空比 0 */
    TIM_OC1Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
    TIM_OC2Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
    TIM_OC3Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
    TIM_OC4Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
    /* 使能 CCR 预装载：调用 TIM_SetCompareX 后下一个周期生效，避免波形毛刺 */
    TIM_OC1PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(MOTOR_PWM_TIM, ENABLE);
    TIM_Cmd(MOTOR_PWM_TIM, ENABLE);
    /* ---------- 6. 上电先全部停住 --------------------------------------- */
    Motor_StopAll();
}
/**
  * @brief  设置单个轮子的速度和方向
  * @param  id    : 轮子编号
  * @param  speed : -1000 ~ +1000，正数正转、负数反转、0 停止
  */
void Motor_SetSpeed(Motor_ID_t id, int16_t speed)
{
    const Motor_HwTypeDef *hw = &Motor_Hw[id];
    uint16_t duty;
    /* 1. 限幅，防止越界 */
    if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;
    /* 2. 该轮如果装反了，这里统一取反 */
    if (hw->Invert)
    {
        speed = -speed;
    }
    /* 3. 方向 GPIO + 占空比 */
    if (speed > 0)                      /* 正转 */
    {
        GPIO_SetBits(hw->IN1_Port,   hw->IN1_Pin);
        GPIO_ResetBits(hw->IN2_Port, hw->IN2_Pin);
        duty = (uint16_t)speed;
    }
    else if (speed < 0)                 /* 反转 */
    {
        GPIO_ResetBits(hw->IN1_Port, hw->IN1_Pin);
        GPIO_SetBits(hw->IN2_Port,   hw->IN2_Pin);
        duty = (uint16_t)(-speed);
    }
    else                                /* 停止：两方向脚都拉低，占空比 0 */
    {
        GPIO_ResetBits(hw->IN1_Port, hw->IN1_Pin);
        GPIO_ResetBits(hw->IN2_Port, hw->IN2_Pin);
        duty = 0;
    }
    if (duty > MOTOR_PWM_MAX)           /* 1000 已经等于 100% 占空比 */
    {
        duty = MOTOR_PWM_MAX;
    }
    /* 4. 写入对应通道的比较寄存器 */
    switch (hw->TIM_Channel)
    {
        case TIM_Channel_1: TIM_SetCompare1(MOTOR_PWM_TIM, duty); break;
        case TIM_Channel_2: TIM_SetCompare2(MOTOR_PWM_TIM, duty); break;
        case TIM_Channel_3: TIM_SetCompare3(MOTOR_PWM_TIM, duty); break;
        case TIM_Channel_4: TIM_SetCompare4(MOTOR_PWM_TIM, duty); break;
        default: break;
    }
}
/**
  * @brief  单个轮子刹车（IN1 = IN2 = 1 时驱动芯片会把电机两端短接，实现刹车）
  * @note   本项目统一用"停止"处理，需要刹车时把两脚都置 1 即可
  */
void Motor_Brake(Motor_ID_t id)
{
    const Motor_HwTypeDef *hw = &Motor_Hw[id];
    GPIO_SetBits(hw->IN1_Port, hw->IN1_Pin);
    GPIO_SetBits(hw->IN2_Port, hw->IN2_Pin);
    switch (hw->TIM_Channel)
    {
        case TIM_Channel_1: TIM_SetCompare1(MOTOR_PWM_TIM, MOTOR_PWM_MAX); break;
        case TIM_Channel_2: TIM_SetCompare2(MOTOR_PWM_TIM, MOTOR_PWM_MAX); break;
        case TIM_Channel_3: TIM_SetCompare3(MOTOR_PWM_TIM, MOTOR_PWM_MAX); break;
        case TIM_Channel_4: TIM_SetCompare4(MOTOR_PWM_TIM, MOTOR_PWM_MAX); break;
        default: break;
    }
}
/**
  * @brief  四个轮子全部停止
  */
void Motor_StopAll(void)
{
    Motor_SetSpeed(MOTOR_LF, 0);
    Motor_SetSpeed(MOTOR_LR, 0);
    Motor_SetSpeed(MOTOR_RF, 0);
    Motor_SetSpeed(MOTOR_RR, 0);
}
/* ############################################################################
 * #                       四、麦轮运动学解算                                   #
 * ############################################################################
 *
 *  麦克纳姆轮的辊子与轮轴成 45°，靠四个轮子的转速差就能合成出
 *  "前后 + 左右 + 自转" 的任意组合运动，这就是麦轮独有的全向移动能力。
 *
 *  ------------------------- 四轮转速分配逻辑 -------------------------
 *
 *      记  vx = 前后分量 (>0 前进)
 *          vy = 横移分量 (>0 右移)
 *          wz = 自转分量 (>0 原地右转/顺时针)
 *
 *      分配公式（X 型辊子布局，四轮均为"左前-右后同向、右前-左后同向"）:
 *
 *          LF(左前) = vx + vy + wz
 *          LR(左后) = vx - vy + wz
 *          RF(右前) = vx - vy - wz
 *          RR(右后) = vx + vy - wz
 *
 *  ------------------------- 为什么是这个公式 -------------------------
 *
 *      · 纯前进 (vx=+V, vy=0, wz=0): 四轮全部 +V  -> 四个轮子同速正转。
 *      · 纯右移 (vx=0, vy=+V, wz=0): LF=+V, RR=+V, LR=-V, RF=-V
 *            -> 左前/右后正转、右前/左后反转，
 *               四个 45° 辊子的切向力在"左右方向"叠加、在前后方向互相抵消，
 *               所以车体纯横向平移（这是麦轮最典型的动作）。
 *      · 纯右转 (vx=0, vy=0, wz=+V): LF=+V, LR=+V, RF=-V, RR=-V
 *            -> 左侧两轮正转、右侧两轮反转，车体绕中心顺时针自转。
 *      · 斜向移动 (vx=+V, vy=+V, wz=0): 就是"前进 + 右移"的矢量合成，
 *          四个轮子得到 +2V/+2V/0/0，走 45° 斜线。
 *      · 任意方向：vx、vy 分别取 sin/cos 分量即可走出任意角度的平移，
 *          若同时叠加 wz，就是"边走边转"的弧线运动。
 *
 *      注意：三个分量相加后单轮值最大可能到 300，必须做等比限幅，
 *            否则会超过 100% 占空比，导致方向比例失真（车会跑偏）。
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
    w[MOTOR_LF] = vx32 + vy32 + wz32;      /* 左前 */
    w[MOTOR_LR] = vx32 - vy32 + wz32;      /* 左后 */
    w[MOTOR_RF] = vx32 - vy32 - wz32;      /* 右前 */
    w[MOTOR_RR] = vx32 + vy32 - wz32;      /* 右后 */
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
 * #                       五、常用动作快捷封装                                 #
 * ############################################################################ */
void Mecanum_Stop(void)         { Mecanum_Move(0, 0, 0); }
void Mecanum_Forward(uint16_t speed)     { Mecanum_Move((int16_t)speed,  0,  0); }
void Mecanum_Backward(uint16_t speed)    { Mecanum_Move(-(int16_t)speed, 0,  0); }
void Mecanum_StrafeLeft(uint16_t speed)  { Mecanum_Move(0, -(int16_t)speed,  0); }   /* vy<0 = 左移 */
void Mecanum_StrafeRight(uint16_t speed) { Mecanum_Move(0,  (int16_t)speed,  0); }   /* vy>0 = 右移 */
void Mecanum_TurnLeft(uint16_t speed)    { Mecanum_Move(0, 0, -(int16_t)speed); }    /* wz<0 = 左转 */
void Mecanum_TurnRight(uint16_t speed)   { Mecanum_Move(0, 0,  (int16_t)speed); }    /* wz>0 = 右转 */
/*  斜向移动：前进/后退分量和横移分量都取 0.707 倍，合成速度大小不变
 *  0.707 ≈ 707/1000，用整数运算避免浮点（F103 没有 FPU）             */
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

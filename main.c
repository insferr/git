#include "stm32f10x.h"
#include "motor.h"
#include "ps2.h"
#include "Delay.h"
/* ---------------------------------------------------------------------------
 *  摇杆 -> 小车速度的映射方向
 *    PS2 左摇杆"往上推"时，LY 读到的是 0（往下推是 255），
 *    所以想让"往上推 = 前进"，Y 方向要取反（invert = 1）。
 *    左右方向"往右推"时 LX 变大（255），不用取反。
 *    ★ 实测如果前后 / 左右是反的，把下面这两个 1<->0 对调即可。
 * ------------------------------------------------------------------------- */
#define PS2_Y_INVERT      1
#define PS2_X_INVERT      0
/* 连续读失败达到这个次数就认为手柄断开 -> 自动停车（防止小车失控跑飞） */
#define PS2_LOST_LIMIT    3
int main(void)
{
    int16_t vx, vy, wz;
    uint8_t joy_x, joy_y;
    uint8_t lost = 0;
    /* 1. 初始化模块 */
    Motor_Init();       /* 电机、PWM，里面自动拉高 STBY 唤醒 DRV8833 */
    PS2_Init();         /* PS2 引脚 + 让手柄进入模拟模式 */
    while (1)           /* 主循环 */
    {
        if (PS2_Read() == 0)                    /* 读到有效数据 */
        {
            lost = 0;
            PS2_GetLeftStick(&joy_x, &joy_y);   /* 一次性读出左摇杆 X、Y */
            /* 摇杆映射：内部已含死区 + 自动限幅到 ±1000 */
            vx = PS2_StickToSpeed(joy_y, PS2_Y_INVERT);     /* >0 前进 */
            vy = PS2_StickToSpeed(joy_x, PS2_X_INVERT);     /* >0 右移 */
            wz = 0;                                         /* 需要自转就再读右摇杆 X */
            if (PS2_AnyButtonPressed())         /* 急停：任意按键按下就停 */
            {
                Mecanum_Stop();
            }
            else
            {
                Mecanum_Move(vx, vy, wz);
            }
        }
        else                                    /* 这次没读到手柄 */
        {
            if (lost < 255)
            {
                lost++;
            }
            if (lost >= PS2_LOST_LIMIT)         /* 手柄拔了 / 没电 -> 停车 */
            {
                Mecanum_Stop();
            }
        }
        Delay_ms(10);       /* 约 10~15ms 读一次，和手柄刷新率匹配 */
    }
		if(PS2_Init() == 0)
	{
		// 初始化成功，比如点亮一个LED
	}
	else
	{
		// 初始化失败，说明接线/供电/手柄有问题
	}

}

/******************* (C) COPYRIGHT *****END OF FILE*****************************/




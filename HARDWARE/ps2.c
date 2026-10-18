#include "ps2.h"
#include "motor.h"         
#define PS2_CLK_H()      GPIO_SetBits(PS2_CLK_PORT, PS2_CLK_PIN)
#define PS2_CLK_L()      GPIO_ResetBits(PS2_CLK_PORT, PS2_CLK_PIN)
#define PS2_CS_H()       GPIO_SetBits(PS2_CS_PORT, PS2_CS_PIN)
#define PS2_CS_L()       GPIO_ResetBits(PS2_CS_PORT, PS2_CS_PIN)
#define PS2_CMD_H()      GPIO_SetBits(PS2_CMD_PORT, PS2_CMD_PIN)
#define PS2_CMD_L()      GPIO_ResetBits(PS2_CMD_PORT, PS2_CMD_PIN)
#define PS2_DAT_READ()   GPIO_ReadInputDataBit(PS2_DAT_PORT, PS2_DAT_PIN)
 
PS2_DataTypeDef PS2_Data;
static uint8_t s_VibSmall = 0;
static uint8_t s_VibBig   = 0;
 
static void PS2_Delay(uint32_t us)
{
    volatile uint32_t i;
    while (us--)
    {
        for (i = 0; i < 8; i++)
        {
            __NOP();                  
        }
    }
}
 
static uint8_t PS2_TransferByte(uint8_t tx)
{
    uint8_t rx = 0;
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if (tx & 0x01)
        {
            PS2_CMD_H();
        }
        else
        {
            PS2_CMD_L();
        }
        PS2_Delay(PS2_HALF_CLK_US);
     
        PS2_CLK_L();
        PS2_Delay(PS2_HALF_CLK_US);
        PS2_CLK_H();
        PS2_Delay(PS2_HALF_CLK_US);
        if (PS2_DAT_READ())
        {
            rx |= (uint8_t)(1u << i);     
        }
        PS2_Delay(PS2_HALF_CLK_US);
        tx >>= 1;
    }
    return rx;
}
 
static void PS2_Exchange(const uint8_t *cmd, uint8_t *resp, uint8_t len)
{
    uint8_t i;
    PS2_CS_H();
    PS2_CLK_H();
    PS2_CMD_H();
    PS2_Delay(PS2_HALF_CLK_US);
    PS2_CS_L();                          
    PS2_Delay(PS2_HALF_CLK_US * 2);
    for (i = 0; i < len; i++)
    {
        resp[i] = PS2_TransferByte(cmd[i]);
        PS2_Delay(PS2_HALF_CLK_US);
    }
    PS2_CS_H();                        
    PS2_Delay(PS2_HALF_CLK_US * 2);
}
 
uint8_t PS2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t cmd[9];
    uint8_t resp[9];
    uint8_t i;
    RCC_APB2PeriphClockCmd(PS2_GPIO_CLK, ENABLE);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = PS2_CLK_PIN;
    GPIO_Init(PS2_CLK_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = PS2_CS_PIN;
    GPIO_Init(PS2_CS_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = PS2_CMD_PIN;
    GPIO_Init(PS2_CMD_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     
    GPIO_InitStructure.GPIO_Pin  = PS2_DAT_PIN;
    GPIO_Init(PS2_DAT_PORT, &GPIO_InitStructure);
    
    PS2_CS_H();
    PS2_CLK_H();
    PS2_CMD_H();
    PS2_Data.Mode   = 0;
    PS2_Data.Btn1   = 0xFF;
    PS2_Data.Btn2   = 0xFF;
    PS2_Data.RX     = 128;
    PS2_Data.RY     = 128;
    PS2_Data.LX     = 128;
    PS2_Data.LY     = 128;
    PS2_Data.Online = 0;
    PS2_Delay(100000);
    
    cmd[0]=0x01; cmd[1]=0x42; cmd[2]=0x00; cmd[3]=0x00; cmd[4]=0x00;
    cmd[5]=0x00; cmd[6]=0x00; cmd[7]=0x00; cmd[8]=0x00;
    PS2_Exchange(cmd, resp, 9);
    
    cmd[0]=0x01; cmd[1]=0x43; cmd[2]=0x00; cmd[3]=0x01; cmd[4]=0x00;
    cmd[5]=0x00; cmd[6]=0x00; cmd[7]=0x00; cmd[8]=0x00;
    PS2_Exchange(cmd, resp, 9);
    
    cmd[0]=0x01; cmd[1]=0x44; cmd[2]=0x00; cmd[3]=0x01; cmd[4]=0x00;
    cmd[5]=0x00; cmd[6]=0x00; cmd[7]=0x00; cmd[8]=0x00;
    PS2_Exchange(cmd, resp, 9);
    
    cmd[0]=0x01; cmd[1]=0x4D; cmd[2]=0x00; cmd[3]=0x00; cmd[4]=0x01;
    cmd[5]=0x00; cmd[6]=0x00; cmd[7]=0x00; cmd[8]=0x00;
    PS2_Exchange(cmd, resp, 9);
    
    cmd[0]=0x01; cmd[1]=0x4F; cmd[2]=0x00; cmd[3]=0xFF; cmd[4]=0xFF;
    cmd[5]=0x03; cmd[6]=0x00; cmd[7]=0x00; cmd[8]=0x00;
    for (i = 0; i < 3; i++)
    {
        PS2_Exchange(cmd, resp, 9);
    }
    
    /* ★ 修改点：删掉了原来末尾那条
     *     cmd[0]=0x01; cmd[1]=0x43; cmd[2]=0x00; cmd[3]=0x00; cmd[4]=0x5A;
     *     cmd[5]=0x5A; cmd[6]=0x5A; cmd[7]=0x5A; cmd[8]=0x5A;
     *     PS2_Exchange(cmd, resp, 9);
     *   因为 0x4F（配置完成）之后手柄已经进入数据模式，
     *   再发 0x43（进入配置模式）会把配置"打回"，导致后面
     *   PS2_Read() 读到的一直是配置响应而不是按键/摇杆数据。
     */
    
    if (PS2_Read() == 0)
    {
        return 0;                       
    }
    return 1;                            
}
 
uint8_t PS2_Read(void)
{
    uint8_t cmd[9];
    uint8_t resp[9];
    cmd[0] = 0x01;
    cmd[1] = 0x42;
    cmd[2] = 0x00;
    cmd[3] = s_VibSmall;                  
    cmd[4] = s_VibBig;                    
    cmd[5] = 0x00;
    cmd[6] = 0x00;
    cmd[7] = 0x00;
    cmd[8] = 0x00;
    PS2_Exchange(cmd, resp, 9);
    PS2_Data.Mode = resp[1];
    PS2_Data.Btn1 = resp[3];
    PS2_Data.Btn2 = resp[4];
#if PS2_SWAP_STICKS
    PS2_Data.RX   = resp[7];
    PS2_Data.RY   = resp[8];
    PS2_Data.LX   = resp[5];
    PS2_Data.LY   = resp[6];
#else
    PS2_Data.RX   = resp[5];
    PS2_Data.RY   = resp[6];
    PS2_Data.LX   = resp[7];
    PS2_Data.LY   = resp[8];
#endif
  
    if ((resp[1] == 0x41) || (resp[1] == 0x73) || (resp[1] == 0x79))
    {
        PS2_Data.Online = 1;
        return 0;
    }
    PS2_Data.Online = 0;
    return 1;
}
 
uint16_t PS2_GetButton(void)
{
    uint16_t raw;
    raw = (uint16_t)(((uint16_t)PS2_Data.Btn2 << 8) | PS2_Data.Btn1);
    return (uint16_t)(~raw);
}
 
uint8_t PS2_ButtonPressed(uint16_t key)
{
    return ((PS2_GetButton() & key) != 0) ? 1 : 0;
}
 
uint8_t PS2_AnyButtonPressed(void)
{
    return ((PS2_GetButton() & PSB_ALL_BUTTONS) != 0) ? 1 : 0;
}
 
void PS2_GetLeftStick(uint8_t *x, uint8_t *y)
{
    if (x != 0) { *x = PS2_Data.LX; }
    if (y != 0) { *y = PS2_Data.LY; }
}
 
void PS2_GetRightStick(uint8_t *x, uint8_t *y)
{
    if (x != 0) { *x = PS2_Data.RX; }
    if (y != 0) { *y = PS2_Data.RY; }
}
 
int16_t PS2_StickToSpeed(uint8_t raw, uint8_t invert)
{
    int32_t v;
    v = (int32_t)raw - PS2_STICK_CENTER;
    if ((v < PS2_STICK_DEAD) && (v > -PS2_STICK_DEAD))
    {
        return 0;
    }
    if (v > 0)
    {
        v = (v - PS2_STICK_DEAD) * 1000 / (127 - PS2_STICK_DEAD);
    }
    else
    {
        v = (v + PS2_STICK_DEAD) * 1000 / (128 - PS2_STICK_DEAD);
    }
    if (invert)
    {
        v = -v;
    }
    if (v >  MOTOR_SPEED_MAX) v =  MOTOR_SPEED_MAX;
    if (v < -MOTOR_SPEED_MAX) v = -MOTOR_SPEED_MAX;
    return (int16_t)v;
}
 
void PS2_Vibrate(uint8_t small, uint8_t big)
{
    s_VibSmall = small;
    s_VibBig   = big;
}

#ifndef __PS2_H
#define __PS2_H

#include "stm32f10x.h"


#define PS2_GPIO_CLK      RCC_APB2Periph_GPIOB

#define PS2_CS_PORT       GPIOB
#define PS2_CS_PIN        GPIO_Pin_10     

#define PS2_CLK_PORT      GPIOB
#define PS2_CLK_PIN       GPIO_Pin_11     

#define PS2_CMD_PORT      GPIOB
#define PS2_CMD_PIN       GPIO_Pin_12

#define PS2_DAT_PORT      GPIOB
#define PS2_DAT_PIN       GPIO_Pin_13    


#define PS2_HALF_CLK_US   10     
#define PS2_BYTE_DELAY_US 10     
#define PS2_READ_GAP_US   1000     
#define PS2_READ_RETRY    5        

/* 1 = 左右摇杆字节对调（本车接收机实测：右摇杆有反应、左摇杆没反应，确认为对调）；
 * 0 = 标准顺序。改这里即可，无需改 ps2.c 逻辑。                              */
#define PS2_SWAP_STICKS   1

#define PS2_STICK_CENTER  128
#define PS2_STICK_DEAD    12    

#define PSB_SELECT        0x0001           
#define PSB_L3            0x0002          
#define PSB_R3            0x0004            
#define PSB_START         0x0008          
#define PSB_PAD_UP        0x0010          
#define PSB_PAD_RIGHT     0x0020       
#define PSB_PAD_DOWN      0x0040       
#define PSB_PAD_LEFT      0x0080           
#define PSB_L2            0x0100
#define PSB_R2            0x0200
#define PSB_L1            0x0400
#define PSB_R1            0x0800
#define PSB_TRIANGLE      0x1000        
#define PSB_CIRCLE        0x2000     
#define PSB_CROSS         0x4000        
#define PSB_SQUARE        0x8000     

#define PSB_UP            PSB_PAD_UP
#define PSB_RIGHT         PSB_PAD_RIGHT
#define PSB_DOWN          PSB_PAD_DOWN
#define PSB_LEFT          PSB_PAD_LEFT


#define PSB_ALL_BUTTONS   0xFFFF

typedef struct
{
    uint8_t  Mode;    
    uint8_t  Btn1;     
    uint8_t  Btn2;    
    uint8_t  RX;     
    uint8_t  RY;     
    uint8_t  LX;    
    uint8_t  LY; 
    uint8_t  Online;   
} PS2_DataTypeDef;

extern PS2_DataTypeDef PS2_Data;



uint8_t PS2_Init(void);


uint8_t PS2_Read(void);


uint16_t PS2_GetButton(void);

uint8_t PS2_ButtonPressed(uint16_t key);


uint8_t PS2_AnyButtonPressed(void);

void PS2_GetLeftStick(uint8_t *x, uint8_t *y);


void PS2_GetRightStick(uint8_t *x, uint8_t *y);


int16_t PS2_StickToSpeed(uint8_t raw, uint8_t invert);


void PS2_Vibrate(uint8_t small, uint8_t big);

void    PS2_ReadData(void);     
uint8_t PS2_GetLeftX(void);    
uint8_t PS2_GetLeftY(void);  
uint8_t PS2_GetKey(void);    

#endif 

/*
 * Copyright (c) 2014 by Sun Zebo <sun.zebo@gmail.com>
 * Servo controller library for maple.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#include "Servo_PWM.h"

HardwareTimer timer1(1);
HardwareTimer timer2(2);
HardwareTimer timer3(3);
HardwareTimer timer4(4);
HardWire WirePort(2, I2C_FAST_MODE);

Servo_PWM::Servo_PWM()
{

}

const char *VERSION = "POWER RHYTHM 32 V2.03";
const char *A_MODE[2] = {"default mode", "trapezoidal_curve mode"};

unsigned int counter_20ms = 0;                          //counter for real time/20ms
unsigned char Timer_state = 1;                          //Timer state On/Off

unsigned char line1 = 1;                                // 缓存存入口与出口之间的距离，即当前缓存中有多少个没有执行的PWM数据
unsigned char point_now = 0;                            // 标记PWM数据组缓存出口位置，即取数位置
unsigned char point_aim = 1;                            // 与point_now一起标记PWM数据组缓存出口位置，即取数位置 
unsigned char point_in = 2;                             // 标记PWM数据组缓存入口位置，即上一行数据存放位置
unsigned int PWM_value_old[7][33];                      // PWM数据缓存数组

unsigned char flag_Rec = 0;                                 // 接收标记
unsigned char flag_RecFul = 0;                          // PWM数据组缓存饱和标志
unsigned char flag_stop = 1;				// 表示一行PWM数据执行结束
unsigned char flag_vpwm = 0;				// 表示到达了该更新PWM数据的时间
unsigned char flag_in = 1;                              // PWM数据组缓存中有空闲空间的标志位
unsigned char flag_out = 1;                             // PWM数据组缓存中有可执行数据的标志位
unsigned int n = 1000;                                  // 用来计算需要建立多少个插补增量数据
unsigned int m = 1;                                     // 用来累计已经执行了多少插补增量数据

unsigned char UartCmd;                                  // 串口命令值
unsigned char Offline_group_num;                        // 脱机PWM数据组号
unsigned char flag_Down;                                // 下载标记
unsigned char Read_num;                                 // 读出的PWM数据组长度。
unsigned char All_Offline_Group_num;                    // 所有PWM数据组组数
unsigned char flag_iic_read_over;                       // 标记已读完数据
unsigned char flag_Go = 0;                              // 循环运行脱机PWM数据组标志
unsigned char Down_num = 0;                             // 一组下载的PWM数据次数。
unsigned char redata[400];                              // 串口接收数据缓存数组
unsigned char ADC_CH = 15;                              // 模数转换通道号
unsigned char Down_Mode = 0;                            // 下载模式标志

double dp;                                              // PWM增量值
double dp0[33];                                         // PWM插补增量
double aclrt[33];                                       // PWM增量的加速度
double t_m[33];                                         // PWM增量的加速时间
double t_0[33];                                         // PWM增量的匀速时间
double V_MAX = 0.18;                                    // 舵机最大速度，单位S：/60°
unsigned char dp_sign[33];                              // 插补增量符号：+-
unsigned char aclrt_mode = Trapezoidal_curve;           // 加速模式

unsigned char pwm_num;                                  // PWM序号
unsigned char numtemp = 0;                              // PWM数据位于对应PWM数据组的行数


unsigned int UartRec[33] = {100, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,      // 串口接收的PWM数据
                                1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
				1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
				1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
unsigned int EepromData[33] = {100, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,   // Flash读出的PWM数据
                                   1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
				   1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
				   1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};	 
unsigned int PWM_value[33] = {100, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,    // 用于更新PWM宽度的PWM数据
                                  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
				  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
				  1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

void Servo_PWM::Servo_PWM_initialize(void)
{
    unsigned char datatemp[67];                         // 临时数据
    
    // 配置数字输出引脚
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(S4, OUTPUT);
    pinMode(S5, OUTPUT);
    pinMode(S6, OUTPUT);
    pinMode(S7, OUTPUT);
    pinMode(S8, OUTPUT);
    pinMode(S9, OUTPUT);
    pinMode(S10, OUTPUT);
    pinMode(S11, OUTPUT);
    pinMode(S12, OUTPUT);
    pinMode(S13, OUTPUT);
    pinMode(S14, OUTPUT);
    pinMode(S15, OUTPUT);
    pinMode(S16, OUTPUT);
    pinMode(S17, OUTPUT);
    pinMode(S18, OUTPUT);
    pinMode(S19, OUTPUT);
    pinMode(S20, OUTPUT);
    pinMode(S21, OUTPUT);
    pinMode(S22, OUTPUT);
    pinMode(S23, OUTPUT);
    pinMode(S24, OUTPUT);
    pinMode(S25, OUTPUT);
    pinMode(S26, OUTPUT);
    pinMode(S27, OUTPUT);
    pinMode(S28, OUTPUT);
    pinMode(S29, OUTPUT);
    pinMode(S30, OUTPUT);
    pinMode(S31, OUTPUT);
    pinMode(S32, OUTPUT);
    pinMode(BOARD_LED_PIN, OUTPUT);
    
    // 关闭LED
    digitalWrite(BOARD_LED_PIN,LOW);
  
    // 配置模数转换输入引脚
    pinMode(AD0, INPUT_ANALOG);
    pinMode(AD1, INPUT_ANALOG);
    
    Serial1.begin(115200);                                                           // 串口1的初始化
    
    WirePort.begin();                                                                 // IIC总线初始化
    while(AT24CXX_Check())                                                            // 检查Flash的连接
    {  
        SerialUSB.println("FLASH no found!check the connection.");
        Serial1.println("FLASH no found!check the connection.");
    }
    Offline_group_num = 0;//AT24CXX_ReadOneByte(Offline_Flag_ADDR);                        // 读脱机PWM数据组号，如不为0则脱机运行Offline_group_num组。
    if (Offline_group_num == 0xff)                                                     // 校验读出的Flash数据 
    {
        Offline_group_num = 0;
    }
    All_Offline_Group_num = AT24CXX_ReadOneByte(All_Group_NUM_ADDR);                   // 读取脱机PWM数据组总数
    if (All_Offline_Group_num == 0xff)                                                 // 校验读出的Flash数据
    {
        All_Offline_Group_num = 48; 
    }
    flag_Go = AT24CXX_ReadOneByte(Circle_Flag_ADDR);                                   // 读取循环运行脱机PWM数据组标志，为1时循环更新所有PWM数据组
    if (flag_Go == 0xff)                                                               // 校验读出的Flash数据
    {
        flag_Go = 0; 
    }
    if (flag_Go != 0)                                                                
    {
        Offline_group_num = 1;                                                         // 从PWM数据组1开始更新
    }
    if ((Offline_group_num > 0) && (Offline_group_num <= Group_MAX_NUM))                // 判断是否为有效的PWM数据组值
    {
        Read_num = AT24CXX_ReadOneByte( Group_Length_ADDR + Offline_group_num );       // 开始读取PWM数据组中的数据
        flag_iic_read_over = 0;                                                        // 表示读取结束
        delay( 5 );
    } 
    else 
    {
        Read_num = 0;
        Offline_group_num = 0;
    }
    
    AT24CXX_Read(eeprom_AddBaisc - 67, datatemp, 67);                                  // 读取初始的PWM数据
    if (datatemp[66] == 0x01)                                                        // 读取成功
    {
      u8Tou16( datatemp, PWM_value_old[1] );                                           // 初始化PWM数据缓存数组的1、2行 
      u8Tou16( datatemp, PWM_value_old[2] );
    }
    
    // 定时器配置，用于产生32路PWM信号
    timer1.pause();                                                                    
    timer1.setPeriod(PWM_0); 
    timer1.setChannel1Mode(TIMER_OUTPUT_COMPARE); 
    timer1.setCompare(TIMER_CH1, 1); 
    timer1.attachCompare1Interrupt(Servo_PWM_callback1);
    timer1.refresh(); 
    timer1.resume(); 
    timer2.pause();
    timer2.setPeriod(PWM_0); 
    timer2.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer2.setCompare(TIMER_CH1, 1); 
    timer2.attachCompare1Interrupt(Servo_PWM_callback2);
    timer2.refresh();
    timer2.resume();
    timer3.pause();
    timer3.setPeriod(PWM_0);
    timer3.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer3.setCompare(TIMER_CH1, 1); 
    timer3.attachCompare1Interrupt(Servo_PWM_callback3);
    timer3.refresh();
    timer3.resume();
    timer4.pause();
    timer4.setPeriod(PWM_0);
    timer4.setChannel1Mode(TIMER_OUTPUT_COMPARE);
    timer4.setCompare(TIMER_CH1, 1);  
    timer4.attachCompare1Interrupt(Servo_PWM_callback4);
    timer4.refresh();
    timer4.resume();
}

void Servo_PWM::Set_timer_state(unsigned char state)
{
    if (state == 0)
    {
       timer1.pause();
       timer2.pause();
       timer3.pause();
       timer4.pause();
       Timer_state = 0;
    }
    else
    {
       timer1.resume();
       timer2.resume();
       timer3.resume();
       timer4.resume();
       Timer_state = 1;
    }
}

// 定时器1的中断服务函数，用于产生1~8的PWM信号
void Servo_PWM::Servo_PWM_callback1()
{
    static unsigned char order = 0;                                                        // 中断步长数 
    switch(order) 
    {                                                                    
        case 1:
            if (PWM_value[1] < PWM_0)                                                      // 如果为0或低于500非法值，将给定时器500。order+1
            {
	        PWM1_output_low;
	        timer1.setPeriod(PWM_0);
  	        order++;
	    } 
            else 
            { 
	        PWM1_output_high;
	        timer1.setPeriod(PWM_value[1] - PWM_Gain);                                 // 更新PWM宽度值
	    }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 2:
            PWM1_output_low;
	    if (PWM_value[1] < PWM_0)                                                      // 如果为0或低于500非法值，将给定时器500。order+1.
            {
                timer1.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);                         // 更新PWM宽度值
            } 
            else 
            {
                timer1.setPeriod(PWM_OUR_TIME - (PWM_value[1] - PWM_Gain));                // 更新PWM宽度值
            }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 3:
            if (PWM_value[2] < PWM_0) 
            {
    	        PWM2_output_low;
    	        timer1.setPeriod(PWM_0);
    	        order++;
            } 
            else 
            { 
	        PWM2_output_high;
	        timer1.setPeriod(PWM_value[2] - PWM_Gain);
	    }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 4:
            PWM2_output_low;
	    if (PWM_value[2] < PWM_0) 
            {
                timer1.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            } 
            else 
            {
                timer1.setPeriod(PWM_OUR_TIME - (PWM_value[2] - PWM_Gain)); 
            }
            timer1.refresh();
	    flag_vpwm = 1;
            break;
        case 5:
	    if (PWM_value[3] < PWM_0) 
            {
                PWM3_output_low;
                timer1.setPeriod(PWM_0);
                order++;
	    } 
            else 
            { 
                PWM3_output_high;
                timer1.setPeriod(PWM_value[3] - PWM_Gain);
	    }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 6:
            PWM3_output_low;
            if (PWM_value[3] < PWM_0) 
            {
                timer1.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            } 
            else 
            {
                timer1.setPeriod(PWM_OUR_TIME - (PWM_value[3] - PWM_Gain));
            }
            timer1.refresh();
    	flag_vpwm = 1;
            break;
        case 7:	  
	    if (PWM_value[4] < PWM_0) 
            {
    	        PWM4_output_low;
                timer1.setPeriod(PWM_0);
	        order++;
	    } 
            else 
            { 
                PWM4_output_high;
                timer1.setPeriod(PWM_value[4] - PWM_Gain); 
	    }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 8:
            PWM4_output_low;
            if (PWM_value[4] < PWM_0) 
            {
                timer1.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            } 
            else 
            {
                timer1.setPeriod(PWM_OUR_TIME - (PWM_value[4] - PWM_Gain));
            }
            timer1.refresh();
	    flag_vpwm = 1;
            break;
        case 9:
            if (PWM_value[5] < PWM_0) 
            {
	        PWM5_output_low;
	        timer1.setPeriod(PWM_0);
	        order++;
	    } 
            else 
            { 
                PWM5_output_high;
                timer1.setPeriod(PWM_value[5] - PWM_Gain);
	    }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 10:
            PWM5_output_low;
            if (PWM_value[5] < PWM_0) 
            {
                timer1.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            } 
            else 
            {
                timer1.setPeriod(PWM_OUR_TIME - (PWM_value[5] - PWM_Gain));
            }
            timer1.refresh();
	    flag_vpwm = 1;
            break;
        case 11:
            if (PWM_value[6] < PWM_0) 
            {
	        PWM6_output_low;
	        timer1.setPeriod(PWM_0);
	        order++;
	    } 
            else 
            { 
                PWM6_output_high;
                timer1.setPeriod(PWM_value[6] - PWM_Gain);
	    }
            timer1.refresh();
            flag_vpwm = 1;
          break;
        case 12:
            PWM6_output_low;
            if (PWM_value[6] < PWM_0) 
            {
                timer1.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            } 
            else 
            {
                timer1.setPeriod(PWM_OUR_TIME - (PWM_value[6] - PWM_Gain));
            }
            timer1.refresh();
	    flag_vpwm = 1;
            break;
        case 13:
            if (PWM_value[7] < PWM_0) 
            {
	        PWM7_output_low;
	        timer1.setPeriod(PWM_0);
	        order++;
	    } 
            else 
            { 
                PWM7_output_high;
                timer1.setPeriod(PWM_value[7] - PWM_Gain);
	    }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 14:
            PWM7_output_low;
            if (PWM_value[7] < PWM_0) 
            {
                timer1.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            } 
            else 
            {
              timer1.setPeriod(PWM_OUR_TIME - (PWM_value[7] - PWM_Gain));
            }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 15:
	    if (PWM_value[8] < PWM_0) 
            {
	        PWM8_output_low;
	        timer1.setPeriod(PWM_0);
	        order++;
	    } 
            else 
            { 
                PWM8_output_high;
                timer1.setPeriod(PWM_value[8] - PWM_Gain);
	    }
            timer1.refresh();
            flag_vpwm = 1;
            break;
        case 16:
            PWM8_output_low;
            if (PWM_value[8] < PWM_0) 
            {
                timer1.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            } 
            else 
            {
                timer1.setPeriod(PWM_OUR_TIME -(PWM_value[8] - PWM_Gain));
            }
            timer1.refresh();
	    flag_vpwm = 1;
	    order = 0;
            break;
        default: 
            order = 0;
            if (counter_20ms < PWM_time_out)
            {
                counter_20ms++;
            }
    }  
    order++;	  
}

void Servo_PWM::Servo_PWM_callback2()
{
    static unsigned char order = 0; 
    switch(order)
    {
        case 1:
            if (PWM_value[9]<PWM_0) 
	    {
	        PWM9_output_low;
	        timer2.setPeriod(PWM_0);
  	        order++;
	    } 
	    else
	    { 
	        PWM9_output_high;
	        timer2.setPeriod(PWM_value[9] - PWM_Gain); 
	    }
            timer2.refresh();
        break;
        case 2:
            PWM9_output_low;
	    if (PWM_value[9] < PWM_0) 
            {
                timer2.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
            else
            { 
                timer2.setPeriod(PWM_OUR_TIME - (PWM_value[9] - PWM_Gain));
            }
            timer2.refresh();
        break;
        case 3:
            if (PWM_value[10] < PWM_0) 
	    {
	        PWM10_output_low;
	        timer2.setPeriod(PWM_0);
	        order++;
            } 
	    else
	    { 
	        PWM10_output_high;
	        timer2.setPeriod(PWM_value[10] - PWM_Gain); 
	    }
            timer2.refresh();
        break;
        case 4:
            PWM10_output_low;
	    if (PWM_value[10] < PWM_0)
            {
                timer2.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer2.setPeriod(PWM_OUR_TIME - (PWM_value[10] - PWM_Gain));
            }
            timer2.refresh();
        break;
        case 5:
	    if (PWM_value[11] < PWM_0) 
	    {
                PWM11_output_low;
                timer2.setPeriod(PWM_0);
                order++;
	    } 
	    else
	    { 
                PWM11_output_high;
                timer2.setPeriod(PWM_value[11] - PWM_Gain);
	    }
            timer2.refresh();
        break;
        case 6:
            PWM11_output_low;
            if (PWM_value[11] < PWM_0) 
            {
                timer2.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
            else  
            {
                timer2.setPeriod(PWM_OUR_TIME - (PWM_value[11] - PWM_Gain));
            }
            timer2.refresh();
        break;
        case 7:	  
	    if (PWM_value[12] < PWM_0) 
            {
	        PWM12_output_low;
                timer2.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM12_output_high;
                timer2.setPeriod(PWM_value[12] - PWM_Gain);
	    }
            timer2.refresh();
        break;
        case 8:
            PWM12_output_low;
            if (PWM_value[12] < PWM_0) 
            { 
                timer2.setPeriod(PWM_OUR_TIME-PWM_Gain - PWM_0);
            }
	    else  
            {
                timer2.setPeriod(PWM_OUR_TIME - (PWM_value[12] - PWM_Gain));
            }
            timer2.refresh();
        break;
        case 9:
            if (PWM_value[13] < PWM_0) 
	    {
	        PWM13_output_low;
	        timer2.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM13_output_high;
                timer2.setPeriod(PWM_value[13] - PWM_Gain);
	    }
            timer2.refresh();
        break;
        case 10:
            PWM13_output_low;
            if (PWM_value[13] < PWM_0) 
            {
                timer2.setPeriod(PWM_OUR_TIME-PWM_Gain - PWM_0);
            }
	    else  
            {
                timer2.setPeriod(PWM_OUR_TIME - (PWM_value[13] - PWM_Gain));
            }
            timer2.refresh();
        break;
        case 11:
            if (PWM_value[14] < PWM_0) 
	    {
	        PWM14_output_low;
	        timer2.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM14_output_high;
                timer2.setPeriod(PWM_value[14] - PWM_Gain);
	    }
            timer2.refresh();
        break;
        case 12:
            PWM14_output_low;
            if (PWM_value[14] < PWM_0) 
            {
                timer2.setPeriod(PWM_OUR_TIME-PWM_Gain - PWM_0);
            }
	    else  
            {
                timer2.setPeriod(PWM_OUR_TIME - (PWM_value[14] - PWM_Gain));
            }
            timer2.refresh();
        break;
        case 13:
            if (PWM_value[15] < PWM_0) 
	    {
	        PWM15_output_low;
	        timer2.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM15_output_high;
                timer2.setPeriod(PWM_value[15] - PWM_Gain);
	    }
            timer2.refresh();
        break;
        case 14:
            PWM15_output_low;
            if (PWM_value[15] < PWM_0) 
            {
                timer2.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer2.setPeriod(PWM_OUR_TIME - (PWM_value[15] - PWM_Gain));
            }
            timer2.refresh();
        break;
        case 15:
	    if (PWM_value[16] < PWM_0) 
	    {
	        PWM16_output_low;
	        timer2.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM16_output_high;
                timer2.setPeriod(PWM_value[16] - PWM_Gain);
	    }
            timer2.refresh();
        break;
        case 16:
            PWM16_output_low;
            if (PWM_value[16] < PWM_0) 
            {
                timer2.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer2.setPeriod(PWM_OUR_TIME - (PWM_value[16] - PWM_Gain));
            }
            timer2.refresh();
	    order = 0;
        break;
        default:
            order = 0;
    }  
    order++;	  
}

void Servo_PWM::Servo_PWM_callback3()
{
    static unsigned char order = 0;       
    switch(order)
    {
        case 1:
            if (PWM_value[17] < PWM_0) 
	    {
	        PWM17_output_low;
	        timer3.setPeriod(PWM_0);
  	        order++;
	    } 
	    else
	    { 
	        PWM17_output_high;
	        timer3.setPeriod(PWM_value[17] - PWM_Gain);
	    }
            timer3.refresh();
        break;
        case 2:
            PWM17_output_low;
	    if (PWM_value[17] < PWM_0) 
            {
                timer3.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
            else 
            {
                timer3.setPeriod(PWM_OUR_TIME - (PWM_value[17] - PWM_Gain));
            }
            timer3.refresh();
        break;
        case 3:
            if (PWM_value[18] < PWM_0) 
	    {
	        PWM18_output_low;
	        timer3.setPeriod(PWM_0);
	        order++;
            } 
	    else
	    { 
	        PWM18_output_high;
	        timer3.setPeriod(PWM_value[18] - PWM_Gain);
	    }
            timer3.refresh();
        break;
        case 4:
            PWM18_output_low;
	    if (PWM_value[18] < PWM_0) 
            {
                timer3.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer3.setPeriod(PWM_OUR_TIME - (PWM_value[18] - PWM_Gain));
            }
            timer3.refresh();
        break;
        case 5:
	    if (PWM_value[19] < PWM_0) 
	    {
                PWM19_output_low;
                timer3.setPeriod(PWM_0);
                order++;
	    } 
	    else
	    { 
                PWM19_output_high;
                timer3.setPeriod(PWM_value[19] - PWM_Gain);
	    }
            timer3.refresh();
        break;
        case 6:
            PWM19_output_low;
            if (PWM_value[19] < PWM_0) 
            {
                timer3.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer3.setPeriod(PWM_OUR_TIME-(PWM_value[19] - PWM_Gain));
            }
            timer3.refresh();
        break;
        case 7:	  
	    if (PWM_value[20] < PWM_0) 
            {
	        PWM20_output_low;
                timer3.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM20_output_high;
                timer3.setPeriod(PWM_value[20] - PWM_Gain);
	    }
            timer3.refresh();
        break;
        case 8:
            PWM20_output_low;
            if (PWM_value[20] < PWM_0) 
            {
                timer3.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer3.setPeriod(PWM_OUR_TIME - (PWM_value[20] - PWM_Gain));
            }
            timer3.refresh();
        break;
        case 9:
            if (PWM_value[21] < PWM_0) 
	    {
	        PWM21_output_low;
	        timer3.setPeriod(PWM_0);
	        order++;
	    }
	    else
	    { 
                PWM21_output_high;
                timer3.setPeriod(PWM_value[21] - PWM_Gain);
	    }
            timer3.refresh();
        break;
        case 10:
            PWM21_output_low;
            if (PWM_value[21] < PWM_0) 
            {  
                timer3.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer3.setPeriod(PWM_OUR_TIME - (PWM_value[21] - PWM_Gain));
            }
            timer3.refresh();
        break;
        case 11:
            if (PWM_value[22] < PWM_0) 
	    {
	        PWM22_output_low;
	        timer3.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM22_output_high;
                timer3.setPeriod(PWM_value[22] - PWM_Gain);
	    }
            timer3.refresh();
        break;
        case 12:
            PWM22_output_low;
            if (PWM_value[22] < PWM_0) 
            {
                timer3.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer3.setPeriod(PWM_OUR_TIME - (PWM_value[22] - PWM_Gain));
            }
            timer3.refresh();
        break;
        case 13:
            if (PWM_value[23] < PWM_0) 
	    {
	        PWM23_output_low;
	        timer3.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM23_output_high;
                timer3.setPeriod(PWM_value[23] - PWM_Gain);
	    }
            timer3.refresh();
        break;
        case 14:
            PWM23_output_low;
            if (PWM_value[23] < PWM_0) 
            {
                timer3.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer3.setPeriod(PWM_OUR_TIME - (PWM_value[23] - PWM_Gain));
            }
            timer3.refresh();
        break;
        case 15:
	    if (PWM_value[24] < PWM_0) 
	    {
	        PWM24_output_low;
	        timer3.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM24_output_high;
                timer3.setPeriod(PWM_value[24] - PWM_Gain);
	    }
            timer3.refresh();
        break;
        case 16:
            PWM24_output_low;
            if (PWM_value[24] < PWM_0) 
            {
                timer3.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer3.setPeriod(PWM_OUR_TIME - (PWM_value[24] - PWM_Gain));
            }
            timer3.refresh();
	    order = 0;
        break;
        default: 
            order = 0;
    }  
    order++;
}

void Servo_PWM::Servo_PWM_callback4()
{
    static unsigned char order = 0;        
    switch(order)
    {
        case 1:
            if (PWM_value[25] < PWM_0) 
	    {
	      PWM25_output_low;
	      timer4.setPeriod(PWM_0);
  	      order++;
	    } 
	    else
	    { 
	        PWM25_output_high;
	        timer4.setPeriod(PWM_value[25] - PWM_Gain);
	    }
            timer4.refresh();
        break;
        case 2:
            PWM25_output_low;
	    if (PWM_value[25] < PWM_0) 
            {
                timer4.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
            else 
            {
                timer4.setPeriod(PWM_OUR_TIME - (PWM_value[25] - PWM_Gain));
            }
            timer4.refresh();
        break;
        case 3:
            if (PWM_value[26] < PWM_0) 
	    {
	        PWM26_output_low;
	        timer4.setPeriod(PWM_0);
	        order++;
            } 
	    else
	    { 
	        PWM26_output_high;
	        timer4.setPeriod(PWM_value[26] - PWM_Gain);
	    }
            timer4.refresh();
        break;
        case 4:
            PWM26_output_low;
	    if (PWM_value[26] < PWM_0) 
            {
                timer4.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer4.setPeriod(PWM_OUR_TIME - (PWM_value[26] - PWM_Gain));
            }
            timer4.refresh();
        break;
        case 5:
	    if (PWM_value[27] < PWM_0) 
	    {
                PWM27_output_low;
                timer4.setPeriod(PWM_0);
                order++;
	    } 
	    else
	    { 
                PWM27_output_high;
                timer4.setPeriod(PWM_value[27] - PWM_Gain);
	    }
            timer4.refresh();
        break;
        case 6:
            PWM27_output_low;
            if (PWM_value[27] < PWM_0) 
            {
                timer4.setPeriod(PWM_OUR_TIME-PWM_Gain - PWM_0);
            }
	    else  
            {
                timer4.setPeriod(PWM_OUR_TIME - (PWM_value[27] - PWM_Gain));
            }
            timer4.refresh();
        break;
        case 7:	  
	    if (PWM_value[28] < PWM_0) 
            {
	        PWM28_output_low;
                timer4.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM28_output_high;
                timer4.setPeriod(PWM_value[28] - PWM_Gain);
	    }
            timer4.refresh();
        break;
        case 8:
            PWM28_output_low;
            if (PWM_value[28] < PWM_0) 
            {
                timer4.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer4.setPeriod(PWM_OUR_TIME - (PWM_value[28] - PWM_Gain));
            }
            timer4.refresh();
        break;
        case 9:
            if (PWM_value[29] < PWM_0) 
	    {
	        PWM29_output_low;
	        timer4.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM29_output_high;
                timer4.setPeriod(PWM_value[29] - PWM_Gain);
	    }
            timer4.refresh();
        break;
        case 10:
            PWM29_output_low;
            if (PWM_value[29] < PWM_0) 
            {
                timer4.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer4.setPeriod(PWM_OUR_TIME - (PWM_value[29] - PWM_Gain));
            }
            timer4.refresh();
        break;
        case 11:
            if (PWM_value[30] < PWM_0) 
	    {
	        PWM30_output_low;
	        timer4.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM30_output_high;
                timer4.setPeriod(PWM_value[30] - PWM_Gain);
	    }
            timer4.refresh();
        break;
        case 12:
            PWM30_output_low;
            if (PWM_value[30] < PWM_0) 
            {
                timer4.setPeriod(PWM_OUR_TIME - PWM_Gain-PWM_0);
            }
	    else  
            {
                timer4.setPeriod(PWM_OUR_TIME - (PWM_value[30] - PWM_Gain));
            }
            timer4.refresh();
        break;
        case 13:
            if (PWM_value[31] < PWM_0) 
	    {
	        PWM31_output_low;
	        timer4.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM31_output_high;
                timer4.setPeriod(PWM_value[31] - PWM_Gain);
	    }
            timer4.refresh();
        break;
        case 14:
            PWM31_output_low;
            if (PWM_value[31] < PWM_0) 
            {
                timer4.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer4.setPeriod(PWM_OUR_TIME - (PWM_value[31] - PWM_Gain));
            }
            timer4.refresh();
        break;
        case 15:
	    if (PWM_value[32] < PWM_0) 
	    {
	        PWM32_output_low;
	        timer4.setPeriod(PWM_0);
	        order++;
	    } 
	    else
	    { 
                PWM32_output_high;
                timer4.setPeriod(PWM_value[32] - PWM_Gain);
	    }
            timer4.refresh();
        break;
        case 16:
            PWM32_output_low;
            if (PWM_value[32] < PWM_0) 
            {
                timer4.setPeriod(PWM_OUR_TIME - PWM_Gain - PWM_0);
            }
	    else  
            {
                timer4.setPeriod(PWM_OUR_TIME - (PWM_value[32] - PWM_Gain));
            }
            timer4.refresh();
	    order = 0;
            digitalWrite(BOARD_LED_PIN, LOW);
        break;
        default: 
            order = 0;
    }  
    order++;
}

// 从AT24CXX的指定寄存器地址ReadAddr读取一字节
unsigned char Servo_PWM::AT24CXX_ReadOneByte(unsigned int ReadAddr)
{
    unsigned char temp = 0XFF;
    if (EE_TYPE > AT24C16)
    {
        WirePort.beginTransmission(I2CBASEADDR);	                                      // 发送写命令
        WirePort.send((unsigned char)(ReadAddr >> 8));                                        // 发送寄存器地址高8位MSB write upper 8 bits to wire (by bit shifting to right)	
    }
    else 
    {
        WirePort.beginTransmission((unsigned char)(I2CBASEADDR + ((ReadAddr / 256) << 1)));   // 发送器件地址I2CBASEADDR,写数据 	 
    }
    WirePort.send((char)(ReadAddr & 0xFF));                                                   // 发送寄存器地址低8位LSB write lower 8 bits to wire (by bit & by 0xFF)
    WirePort.endTransmission();                                                               // 结束IIC通信end transmission on wire
    WirePort.requestFrom((unsigned char)I2CBASEADDR, (unsigned char)1);                       // 从EEPROM取出一字节request 1 byte from EEPROM (memory location sent above)	    
    if (WirePort.available()) 
    {
        temp = WirePort.receive();
    }
    return temp;
}

// 向AT24CXX的指定寄存器地址WriteAddr写入一字节数据
void Servo_PWM::AT24CXX_WriteOneByte(unsigned int WriteAddr, unsigned char DataToWrite)
{
    if (EE_TYPE > AT24C16)
    {
        WirePort.beginTransmission(I2CBASEADDR); 
        WirePort.send((unsigned char)(WriteAddr >> 8));   
    }
    else
    {
        WirePort.beginTransmission((unsigned char)(I2CBASEADDR + ((WriteAddr / 256) << 1))); 
    }
    WirePort.send((char)(WriteAddr & 0xFF));	 										  		   
    WirePort.send(DataToWrite);                                                               //发送字节							   
    WirePort.endTransmission();
    delay(5); 
}

// 从AT24CXX的指定寄存器地址ReadAddr读取NumToRead字节存入数组pBuffer
void Servo_PWM:: AT24CXX_Read(unsigned int ReadAddr, unsigned char *pBuffer, unsigned int NumToRead)
{
    /* if(EE_TYPE>AT24C16)
    {
        WirePort.beginTransmission(I2CBASEADDR);
        WirePort.send((unsigned char)(ReadAddr >> 8));
    }
    else 
    WirePort.beginTransmission((unsigned char)(I2CBASEADDR + ((ReadAddr / 256) << 1)));  	 
    WirePort.send((char)(ReadAddr & 0xFF));  
    WirePort.endTransmission();
    WirePort.requestFrom((unsigned char)I2CBASEADDR,NumToRead); */
    while (NumToRead--)
    {
        *pBuffer = AT24CXX_ReadOneByte(ReadAddr);
        ReadAddr++;
        pBuffer++;
    }
}

// 向AT24CXX的指定寄存器地址WriteAddr写入NumToWrite字节pBuffer中的数据
void Servo_PWM::AT24CXX_Write(unsigned int WriteAddr, unsigned char *pBuffer, unsigned int NumToWrite)
{
    /* if (EE_TYPE > AT24C16)
    {
        WirePort.beginTransmission(I2CBASEADDR);
        WirePort.send((unsigned char)(WriteAddr >> 8));
    }
    else
    {
        WirePort.beginTransmission((unsigned char)(I2CBASEADDR + ((WriteAddr / 256) << 1)));   
    }
    WirePort.send((char)(WriteAddr & 0xFF));*/
    while(NumToWrite--)
    {
        AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
        WriteAddr++;
        pBuffer++;
    }								   
 
}

// 检查AT24CXX的连接，连接正常时返回0
unsigned char Servo_PWM::AT24CXX_Check(void)
{
    unsigned char temp;
    temp = AT24CXX_ReadOneByte(Check_Addr);                                              //避免每次开机都写AT24CXX			   
    if(temp == 0X55)
    {
        return 0;
    }		   
    else                                                                                 //排除第一次初始化的情况
    {
        AT24CXX_WriteOneByte(Check_Addr,0X55);
        temp = AT24CXX_ReadOneByte(Check_Addr);	  
        if(temp == 0X55)
        {
            return 0;
        }
    }
    return 1;											  
}

// 根据运行时间t_n计算从point_1运行至point_2的加速度aclrt和加速时间t_m与匀速时间t_0
void Servo_PWM::accelerate_method(unsigned char point_1, unsigned char point_2, unsigned int t_n)
{
    unsigned char s;
    switch(aclrt_mode) 
    {
        case 1:                                                                              // 计算Trapezoidal_curve模式下的插补增加量
            for (s = 1; s < 33; s++) 
            {
                if (PWM_value_old[point_2][s] > PWM_value_old[point_1][s]) 
                {
                    dp = PWM_value_old[point_2][s] - PWM_value_old[point_1][s];
                    dp_sign[s] = 1;
                } 
                else 
                {
                    dp = PWM_value_old[point_1][s] - PWM_value_old[point_2][s];
                    dp_sign[s] = 0;
                 }
                 t_0[s] = 2 * dp * V_MAX - t_n;
                 if (t_0[s] < 0) 
                 {
                     t_0[s] = 0;
                 }                  
                 t_m[s] = (t_n - t_0[s]) / 2;
                 aclrt[s] = dp / (t_m[s] * (t_m[s] + t_0[s]));
                 dp0[s] = 0;
             }
             break;
         default :                                                                            // 计算匀速下的插补增量
             for (s = 1;s < 33; s++) 
             {    
                 if (PWM_value_old[point_2][s] > PWM_value_old[point_1][s]) 
                 {
                     dp = PWM_value_old[point_2][s] - PWM_value_old[point_1][s];
                     dp0[s] = dp / t_n;
                     dp_sign[s] = 1;
                 }
                 if (PWM_value_old[point_2][s] < PWM_value_old[point_1][s]) 
                 {
                     dp = PWM_value_old[point_1][s] - PWM_value_old[point_2][s];
                     dp0[s] = dp / t_n;
                     dp_sign[s] = 0;
                 }
                 if (PWM_value_old[point_2][s] == PWM_value_old[point_1][s]) 
                 {
                     dp0[s] = 0;
                 }
             }
    }
}

//根据加速度控制模式计算每次的插补增量dp_nbr
double Servo_PWM::dp_calculate(unsigned int t1_25, unsigned char dp_nbr)
{
    switch(aclrt_mode) 
    {
    case 1:
        if (t1_25 < t_m[dp_nbr]) {
            dp0[dp_nbr] = dp0[dp_nbr] + t1_25 * aclrt[dp_nbr];
        } else if (t1_25 > (t_m[dp_nbr] + t_0[dp_nbr])) {
            dp0[dp_nbr] = dp0[dp_nbr] + (2 * t_m[dp_nbr] + t_0[dp_nbr] - t1_25) * aclrt[dp_nbr];
        } else {
            dp0[dp_nbr] = dp0[dp_nbr] + t_m[dp_nbr] * aclrt[dp_nbr]; 
        }
        return dp0[dp_nbr];
        break;
    default :
       return t1_25 * dp0[dp_nbr];
    }
}

//更新PWM数据
 void Servo_PWM::PWM_change(void)
{
    if (line1 > 0)                                         // 表示PWM数据缓存数组中有数据
    {  
        line1--;                                           // 使用了一行PWM数据
        if (line1 < 5)                                     // PWM数据缓数组存允许放入新的数据	
        {
            flag_in = 1;
        }
        
        point_now++;		                           // PWM数据位置更新
        point_aim++;                                       // PWM数据位置更新
        
        if (point_aim == 7)                                // 循环使用PWM数据缓存数组
        {
            point_aim = 0;
        }
        if (point_now == 7)
        {
            point_now = 0;
        }
        
        PWM_value[0] = PWM_value_old[point_aim][0];        // 运行时间
        if (PWM_value_old[point_aim][0] == 0)              // 运行时间为0时差不次数为1，不进行插补
        {
            n=1;
        }
        else
        {
            n = PWM_value_old[point_aim][0] * 4 / 5;	   // 计算新的插补次数
        }
        accelerate_method(point_now, point_aim, n);        // 计算加速过程相关参数
        m = 0;				  	           // m清0
        flag_stop = 0;		  	                   // 产生了新的目标PWM数据，停止标志清零
    }
    else	  					   // 没有缓存数据，即line==0
    {
        flag_out = 0;                                      // 表示PWM数据缓存数组中没有数据
    }
}

//PWM变化速度控制
void Servo_PWM::V_PWM(void)		 
{	   
    unsigned char j=0;
  unsigned char how=0;
  unsigned int mun=0;
  unsigned int temp=0;
  static unsigned char flag_how;
  static unsigned char flag_Tover;
  if(flag_stop==1)   					//一行作业全部完成
  {
    if(flag_out==1)	 				//缓冲数组中有数据
    PWM_change();					//更新行
    else
    for(j=1;j<33;j++)
      PWM_value[j]=PWM_value[j];				//缓存中没有数据，则停止在当前位置
  }
  else	
  {
    m++;							//用来累加插补过的次数
    if(m==n)
    {
      flag_Tover=1;				//一行数据的执行时间已经完成
    }
    for(j=1;j<33;j++)
    {
      
      if (PWM_value[j]>PWM_value_old[point_aim][j]) 
      mun=PWM_value[j]-PWM_value_old[point_aim][j];
      else if  (PWM_value[j]==PWM_value_old[point_aim][j])
      mun=0;
      else mun =PWM_value_old[point_aim][j]-PWM_value[j];
      if((mun<4)||(mun==0))
      {						   	//检测靠近终点位置
        how++;				   	//是，则累加一个
        PWM_value[j]=PWM_value_old[point_aim][j];//并且直接过度到终点位置
      }	
      else						//不靠近终点，继续插补	
      {
        if (dp_sign[j] == 1) {
            temp = PWM_value_old[point_now][j] + dp_calculate(m, j);
        } else {
            temp = PWM_value_old[point_now][j] - dp_calculate(m, j);
        }
      if (temp>2500) {temp=2500;how++;}
      else if(temp<500) {temp=500;how++;}	 //2013/7/14增加，防止定时器溢出。
      else PWM_value[j]=temp;
      //PWM_value[j]=PWM_value_old[point_aim][j];
      }
    } 
    if(how==32)
    {
      flag_how=1;	  					//32个舵机都到达终点
      //Uart1_PutChar('H');
    }
    how=0; 
    if((flag_Tover==1)&&(flag_how==1))
    {								//从插补次数，和脉宽宽度两方面都到达终点，本作业行完成
      flag_Tover=0;
      flag_how=0;
      // m=0;
      flag_stop=1;			 	//行完成标志置1
      flag_iic_read_over=0;	//脱机时更新下一行
      SerialUSB.print('N',BYTE);//向上位机返回一个应答信号，表示一行执行完成。
      Serial1.print('N',BYTE);
      digitalWrite(BOARD_LED_PIN, HIGH);
    }
  }
  return;
}

//串口与USB口的数据接收
void Servo_PWM::Serial_process(USBSerial *serial)
{
    static unsigned int pointer = 0;
    if(serial->available())
    {
        redata[pointer] = (char)serial->read();
        pointer++;
        if(redata[pointer - 1] == '!')	//！，表示字符串结束，一个指令结束
        {
            flag_RecFul = 1;
            pointer=0;
        }
        digitalWrite(BOARD_LED_PIN, HIGH);
    } 
    else if (Serial1.available()) 
    {
        redata[pointer] = Serial1.read();
        pointer++;
        if (redata[pointer - 1] == '!') 
        {	//！，表示字符串结束，一个指令结束
            flag_RecFul = 1;
            pointer = 0;
            counter_20ms = 0;
        }
        digitalWrite(BOARD_LED_PIN, HIGH);
    } 
    
    if (pointer > 399)
    {
        pointer = 0;
    }
}

unsigned char Servo_PWM::ASC_To_Valu(unsigned char asc)
{
  unsigned char valu;
  switch(asc)
  {
    case 0x30:valu=0;break;
    case 0x31:valu=1;break;
    case 0x32:valu=2;break;
    case 0x33:valu=3;break;
    case 0x34:valu=4;break;
    case 0x35:valu=5;break;
    case 0x36:valu=6;break;
    case 0x37:valu=7;break;
    case 0x38:valu=8;break;
    case 0x39:valu=9;break;
  }
  return valu;
}

void Servo_PWM::RecStr_to_pos(void)
{
  unsigned char x;
  if(line1<7) 					//缓存还有空闲，没有满
  {
    line1++;					//缓存中又增加了一行数据
    point_in++;
    if(point_in==7)
    point_in=0;
    for(x=0;x<33;x++)
    {
      if ( (Offline_group_num > 0) && ( Offline_group_num <= Group_MAX_NUM))
      {
          if ((4 < x < 29) && (flag_Rec == 1))
          {
              PWM_value_old[point_in][x]= UartRec[x];
          }
          else
          PWM_value_old[point_in][x]= EepromData[x];//printf("%d",EepromData[x]);
      }
      else 
      PWM_value_old[point_in][x]= UartRec[x];	
    }
  }
  
  else
  {
    flag_in=0;				//表示没有空间了
  }
  if(line1>0)
  flag_out=1;					//表示缓存中有数据
}

void Servo_PWM::RrcStr_to_pwm(void)
{
  unsigned char i=0;
  for (i=0;i<pwm_num+1;i++)
  PWM_value[i]= UartRec[i];
}

void Servo_PWM::RecStr_to_Down(void)
{ 
  unsigned char tempchar[66];
  u16Tou8(tempchar,UartRec);
  if( Down_Mode != 0 ){
    if(( Down_num < Group_MAX_Length ) && ( All_Offline_Group_num <= Group_MAX_NUM ))   //只能下载20个运作数据
    {
      
      AT24CXX_Write( eeprom_AddBaisc + ( Down_Mode - 1 ) * 1341 + 67 * Down_num, tempchar, 66 );    //下载第一组数据
      delay( 5 );
      AT24CXX_WriteOneByte(eeprom_AddBaisc+ ( Down_Mode - 1 ) *1341 + 67 * Down_num + 66, 0x01 );	 //在最后一位加一个标记，在读出时以此为标记读完
      SerialUSB.print('A',BYTE);		//每写入一组数据，回PC机一个A，告诉PC机可以再发下一条了
      Serial1.print('A',BYTE);	
    } 
    Down_num++;		//运作数加1
  } else  {
      AT24CXX_Write( eeprom_AddBaisc - 67, tempchar, 66 );    //下载第一组数据
      delay( 5 );
      AT24CXX_WriteOneByte(eeprom_AddBaisc - 1, 0x01 );	 //在最后一位加一个标记，在读出时以此为标记读完
      SerialUSB.print('A',BYTE);		//每写入一组数据，回PC机一个A，告诉PC机可以再发下一条了
      Serial1.print('A',BYTE);
    }
}

void Servo_PWM::Deal_commands(unsigned char *str)
{
  unsigned char motor_num=0;		   //舵机号
  unsigned int motor_jidu=0;	   //舵机脉宽值
  unsigned int motor_time=0;	   //执行时间
  unsigned char num_now=0;		   //编号解析中间变量
  unsigned char PWM_now=0;		   //脉宽解析中间变量
  unsigned char time_now=0;		   //执行时间解析中间变量
  unsigned char Offline_group_num_now = 0;
  unsigned char Down_Mode_now = 0;
  unsigned char flag_num=0;		   //标记出现过#
  unsigned char flag_jidu=0;		   //标记出现过P
  unsigned char flag_time=0;		   //标记出现过T
  unsigned int i=0;				   //用来移动字符串
  while( str[i]!='!' )
  {
    if(flag_num==1)	 				//出现过#
    {
      if(str[i]!='P')				//如果当前字符不是P
      {
        num_now=ASC_To_Valu(str[i]);//把当前数字字符转化成数字的值
        motor_num=motor_num*10+num_now;
      }
      else  						//当前字符是P
      flag_num=0;
    }
    else
    {
      if(str[i]=='C'&&str[i+1]=='L'&&str[i+2]=='E'&&str[i+3]=='A'&&str[i+4]=='R'&&str[i+5]=='!')
      {
        UartCmd=Cmd_Clear;
        break;
      }
      if(str[i]=='V'&&str[i+1]=='!')
      {
        UartCmd= Cmd_PC_Link;
        break;
      }	
      if(str[i]=='E'&&str[i+1]=='N')
      {
        UartCmd=Cmd_Enable ;
        while( str[i + 2 ] != '!' )
        {
          Offline_group_num_now = Offline_group_num_now * 10 + ASC_To_Valu(str[i + 2]);
          i ++;
        }
        Offline_group_num = Offline_group_num_now;
        if (Offline_group_num!=0) 
        {
          numtemp = 0;		//動作組序號清零
          UartCmd=Cmd_Enable;
          flag_stop=1;					//強制停止作業
          flag_iic_read_over=0;	//脱机时更新下一行、
        } 
        else  UartCmd=Cmd_Disabled;
        flag_Rec = 0;
        break;
      }
      if(str[i]=='R'&&str[i+1]=='U'&&str[i+2]=='N'&&str[i+3]=='!') 
      {
        UartCmd=Cmd_Run;
        break;
      }
      if(str[i]=='R'&&str[i+1]=='E'&&str[i+2]=='A'&&str[i+3]=='D'&&str[i+4]=='!') 
      {
        UartCmd=Cmd_READ;
        break;
      }
      if(str[i]=='G'&&str[i+1]=='O'&&str[i+2]=='!') 
      {
        UartCmd=Cmd_GO;
        numtemp = 0;		//動作組序號清零
        flag_stop=1;					//強制停止作業
        flag_iic_read_over=0;	//脱机时更新下一行、
        break;
      }
      if(str[i]=='D'&&str[i+1]=='O'&&str[i+2]=='W'&&str[i+3]=='N') 
      {
        UartCmd = Cmd_DOWN;
        UartCmd = Cmd_DOWN;
        while( str[i + 4 ] != '!' )
        {
          Down_Mode_now = Down_Mode_now * 10 + ASC_To_Valu(str[i + 4]);
          i ++;
        }
        Down_Mode = Down_Mode_now;
	//Down_Mode = str[i+4] - '0';
        SerialUSB.print( 'A' ,BYTE);
        Serial1.print( 'A' ,BYTE);
        flag_Down = 1;
        break;
      }
      if(str[i]=='S'&&str[i+1]=='T'&&str[i+2]=='O'&&str[i+3]=='P'&&str[i+4]=='!')
      {
        if( Down_Mode != 0 ) {
          All_Offline_Group_num = 48;
          AT24CXX_WriteOneByte( Group_Length_ADDR + Down_Mode, Down_num );
          AT24CXX_WriteOneByte( All_Group_NUM_ADDR, All_Offline_Group_num );
          AT24CXX_WriteOneByte( Clear_Flag_ADDR, 1 );
        }
        flag_Down = 0;
        Down_num = 0;
        break;
      }
      if(str[i]=='A'&&str[i+1]=='D'&&str[i+3]=='!')
      {
        ADC_CH = str[i+2] - '0' +  15;
        if( ADC_CH == 15 || ADC_CH == 16 )
        UartCmd = Cmd_ADC;
        break;
      }
      if(str[i]=='S'&&str[i+1]=='M'&&str[i+3]=='!')
      {
        aclrt_mode = str[i+2] - '0';
        UartCmd = Cmd_SM;
        break;
      }
    }
    if(flag_jidu==1)				//出现过P
    {
      if((str[i]!='T')&(str[i]!='#'))
      {							//当前字符是出现p之后的非#非T的字符
        PWM_now=ASC_To_Valu(str[i]);//把当前数字字符转化成数字的值
        motor_jidu=motor_jidu*10+PWM_now;
      }
      else  						//当前字符是#或者T，角度数据结束
      {
        flag_jidu=0;
        if(motor_jidu>2500)
        motor_jidu=2500;
        if(motor_jidu<500)
        motor_jidu=500;
        UartRec[motor_num]=motor_jidu;
			//	Uart1_PutChar(UartRec[motor_num]);
        pwm_num=motor_num;
        motor_jidu=0;
        motor_num=0;
      }
    }
    if(flag_time==1)				//出现了T
    {
      time_now=ASC_To_Valu(str[i]);//把当前数字字符转化成数字的值
      motor_time=motor_time*10+time_now;
      UartRec[0]=motor_time;	   	//执行时间放在【0】位置
			//Uart1_PutChar(UartRec[0]);
      if(str[i+1]=='!')
      {	 
        if (flag_Down == 1)
        {	 //下载数据
          RecStr_to_Down();  
        }
        else
        {	//不是下载数据
          if(motor_time<=100)		 //如果给定的时间小于100us，则舵机不进行速度控制，实时的改变舵机角度
          {
            RrcStr_to_pwm();
          }
          else					 //如果时间大于100us进行速度控制
          {
            flag_Rec = 1;                                 // 接收标记
            RecStr_to_pos(); 	//下一个字符是！，表示本行指令结束 ，将速度控制的时间值存放在数组的0位
          }
        }
      }
    }
    if(str[i]=='#')
    flag_num=1;
    if(str[i]=='P')
    flag_jidu=1;
    if(str[i]=='T')
    flag_time=1;
    i++;
  }
}

void Servo_PWM::u8Tou16(unsigned char datatemp8[],unsigned int datatemp16[])
{		   
  unsigned char i=0;
  for (i=0;i<33;i++)
  {
   datatemp16[i] = (datatemp8[2*i+1]<<8)&0xff00;
	 datatemp16[i] += datatemp8[2*i];
  }
}

void Servo_PWM::u16Tou8(unsigned char datatemp8[],unsigned int datatemp16[])
{		   
  unsigned char i=0;
  for (i=0;i<33;i++)
  {
   datatemp8[2*i] = (unsigned char)(datatemp16[i]&0xff);
	 datatemp8[2*i+1] =(unsigned char)(datatemp16[i]>>8);
  }
}

void Servo_PWM::Read_Data()
{
  unsigned char datatemp[67];
  if((Offline_group_num > 0) && (Offline_group_num <= Group_MAX_NUM)&&(Read_num != 0))
  {
    if (numtemp + 1 > Read_num)
    {
      if( flag_Go == 1 )
      {
        Offline_group_num ++;
        if (Offline_group_num > All_Offline_Group_num) Offline_group_num=1;
        SerialUSB.print(Offline_group_num,DEC);
        Serial1.print(Offline_group_num,DEC);
        Read_num = AT24CXX_ReadOneByte( Group_Length_ADDR + Offline_group_num );
      }
      numtemp = 0;//flag_iic_read_over=1;//读取结束
    }
    else
    {
      AT24CXX_Read( eeprom_AddBaisc + ( Offline_group_num - 1 )*1341 + 67 * numtemp, datatemp, 67 );		   //不知道什么时候读取完了，为此在67位上增加标记。当67等到标记时，证明全读出了。可以更换PWM了。
      if ( datatemp[66] == 0x01 )  //读完数据后才开始转换
      {	
        
        flag_iic_read_over = 1;
        //UART1_Send_Array(datatemp,67);
        u8Tou16( datatemp, EepromData );//EepromData);   	UartRec
        RecStr_to_pos(); 
        numtemp ++;
      }
    }
  }
  else flag_iic_read_over=1;
}

void Servo_PWM::Uart_Cmd(void)
{
  unsigned char datatemp[67];
  unsigned char datacleartemp[ Group_MAX_NUM ]={0};
  switch(UartCmd)
  {
    case Cmd_PC_Link:
      SerialUSB.print(VERSION);
      Serial1.print(VERSION);
    break;
    case Cmd_Clear:
      AT24CXX_WriteOneByte( Clear_Flag_ADDR, 0xff );
      All_Offline_Group_num = 0;
      Offline_group_num = 0;
      flag_Go = 0;
      AT24CXX_WriteOneByte( Offline_Flag_ADDR, Offline_group_num );
      AT24CXX_WriteOneByte( All_Group_NUM_ADDR, All_Offline_Group_num );
      AT24CXX_WriteOneByte( Circle_Flag_ADDR, flag_Go );
      AT24CXX_Write( Group_Length_ADDR + 1, datacleartemp, Group_MAX_NUM );	 //清11-58的数据
      Read_num=0x00;
      SerialUSB.print('U',BYTE);
      Serial1.print('U',BYTE);
    break;
    case Cmd_Enable:
      AT24CXX_WriteOneByte( Offline_Flag_ADDR, Offline_group_num );  //it works when 0 byte is not FF
      Read_num = AT24CXX_ReadOneByte( Group_Length_ADDR + Offline_group_num );
      flag_Go = 0;
      AT24CXX_WriteOneByte( Circle_Flag_ADDR, flag_Go );
      SerialUSB.print('W',BYTE);
      Serial1.print('W',BYTE);
    break;
    case Cmd_Disabled:
      Offline_group_num = 0;
      flag_Go = 0;
      AT24CXX_WriteOneByte( Offline_Flag_ADDR, Offline_group_num );
      AT24CXX_WriteOneByte( Circle_Flag_ADDR, flag_Go );
      delay(5);
      AT24CXX_Read( eeprom_AddBaisc - 67, datatemp, 67 );		   //不知道什么时候读取完了，为此在67位上增加标记。当67等到标记时，证明全读出了。可以更换PWM了。
      if ( datatemp[66] == 0x01 )  {//读完数据后才开始转换
	//UART1_Send_Array(datatemp,67);
	u8Tou16( datatemp, UartRec );//EepromData);   	UartRec
	RecStr_to_pos(); 
      }
      SerialUSB.print('W',BYTE);
      Serial1.print('W',BYTE);
    break;
    case Cmd_READ:
      if( AT24CXX_ReadOneByte( Clear_Flag_ADDR ) == 1 ) 
      {
        SerialUSB.print( All_Offline_Group_num,DEC);
        Serial1.print( All_Offline_Group_num,DEC);
      }
      else {
      SerialUSB.print(0,DEC);
      Serial1.print(0,DEC);
      }
    break;
    case Cmd_DOWN:
    break;
    case Cmd_Run:
      SerialUSB.print('N',BYTE);
      Serial1.print('N',BYTE);
    break;
    case Cmd_GO:
      flag_Go = 1;
      Offline_group_num = 1;
      Read_num=AT24CXX_ReadOneByte( Group_Length_ADDR + Offline_group_num );
      AT24CXX_WriteOneByte( Circle_Flag_ADDR, flag_Go );
      SerialUSB.print(1,DEC);
      Serial1.print(1,DEC);
     break;
     case Cmd_ADC:
       SerialUSB.print( analogRead( ADC_CH ),DEC);
       Serial1.print( analogRead( ADC_CH ),DEC);
     break;
     case Cmd_SM:
         SerialUSB.print("Switch into ");
         Serial1.print("Switch into ");
         SerialUSB.println(A_MODE[aclrt_mode]);
         Serial1.println(A_MODE[aclrt_mode]);
     break;
     default:
     break;
   }
   UartCmd=Cmd_NULL;	 //	增加此值，是命令只需执行一次。
}

void Servo_PWM::Servo_process(void)
{
  /*unsigned int datatemp1[33]={ 100, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                                1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
				1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
				1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500 };
 unsigned char datatemp2[66];
  unsigned char datatemp[66];*/
  if ( flag_iic_read_over == 0 )
    Read_Data(  );
  if( flag_vpwm == 1)
  {	
    V_PWM();					//更新pwm[]数组
    flag_vpwm=0;
  }
  Serial_process(&SerialUSB);
  if(flag_RecFul == 1)
  {
    Deal_commands(redata);
    Uart_Cmd();
    flag_RecFul = 0;
  } 
  if (counter_20ms >= PWM_time_out)
  {
     
  }
  /*u16Tou8(datatemp2,datatemp1);
  AT24CXX_Write( 11 , datatemp2, 66 );;
  delay(5);
  AT24CXX_Read( 100, datatemp, 66 );
  for(int i=0;i<66;i++)
  {
  SerialUSB.print(datatemp[i],DEC);
  SerialUSB.print(" ");
  }
  SerialUSB.println(" ");*/
}


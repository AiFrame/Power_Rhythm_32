/*
 * Copyright (c) 2014 by Sun Zebo <sun.zebo@gmail.com>
 * Servo controller library for maple.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
 
#ifndef SERVO_PWM_H  //预编译指令，防止重复定义类
#define SERVO_PWM_H

#include <maple.h>
#include <WProgram.h>
#include <usb_serial.h>
#include "HardWire.h"

#define PWM_OUR_TIME 20000/8   //周期  20000US=20MS 几路PWM :8   
#define PWM_Gain 5           //误差增益
#define PWM_0 500

#define S1 21
#define S2 22
#define S3 23
#define S4 17
#define S5 18
#define S6 2
#define S7 3
#define S8 1
#define S9 0
#define S10 10
#define S11 12
#define S12 11
#define S13 19
#define S14 20
#define S15 27
#define S16 28
#define S17 31
#define S18 32
#define S19 33
#define S20 34
#define S21 35
#define S22 36
#define S23 37
#define S24 38
#define S25 6
#define S26 26
#define S27 25
#define S28 4
#define S29 5
#define S30 9
#define S31 14
#define S32 24

#define PWM1_output_high  digitalWrite(S1,HIGH)         //宏定义 方便操作
#define PWM1_output_low         digitalWrite(S1,LOW)

#define PWM2_output_high  digitalWrite(S2,HIGH)
#define PWM2_output_low         digitalWrite(S2,LOW)

#define PWM3_output_high  digitalWrite(S3,HIGH)
#define PWM3_output_low          digitalWrite(S3,LOW)

#define PWM4_output_high  digitalWrite(S4,HIGH)
#define PWM4_output_low          digitalWrite(S4,LOW)

#define PWM5_output_high  digitalWrite(S5,HIGH)
#define PWM5_output_low          digitalWrite(S5,LOW)

#define PWM6_output_high  digitalWrite(S6,HIGH)
#define PWM6_output_low          digitalWrite(S6,LOW)

#define PWM7_output_high  digitalWrite(S7,HIGH)
#define PWM7_output_low          digitalWrite(S7,LOW)

#define PWM8_output_high  digitalWrite(S8,HIGH)
#define PWM8_output_low          digitalWrite(S8,LOW)

#define PWM9_output_high  digitalWrite(S9,HIGH)
#define PWM9_output_low          digitalWrite(S9,LOW)

#define PWM10_output_high  digitalWrite(S10,HIGH)
#define PWM10_output_low         digitalWrite(S10,LOW)

#define PWM11_output_high  digitalWrite(S11,HIGH)
#define PWM11_output_low          digitalWrite(S11,LOW)

#define PWM12_output_high  digitalWrite(S12,HIGH)
#define PWM12_output_low          digitalWrite(S12,LOW)

#define PWM13_output_high  digitalWrite(S13,HIGH)
#define PWM13_output_low          digitalWrite(S13,LOW)

#define PWM14_output_high  digitalWrite(S14,HIGH)
#define PWM14_output_low          digitalWrite(S14,LOW)

#define PWM15_output_high  digitalWrite(S15,HIGH)
#define PWM15_output_low          digitalWrite(S15,LOW)

#define PWM16_output_high  digitalWrite(S16,HIGH)
#define PWM16_output_low          digitalWrite(S16,LOW)

#define PWM17_output_high  digitalWrite(S17,HIGH)
#define PWM17_output_low          digitalWrite(S17,LOW)

#define PWM18_output_high  digitalWrite(S18,HIGH)
#define PWM18_output_low          digitalWrite(S18,LOW)

#define PWM19_output_high  digitalWrite(S19,HIGH)
#define PWM19_output_low          digitalWrite(S19,LOW)

#define PWM20_output_high  digitalWrite(S20,HIGH)
#define PWM20_output_low          digitalWrite(S20,LOW)

#define PWM21_output_high  digitalWrite(S21,HIGH)
#define PWM21_output_low          digitalWrite(S21,LOW)

#define PWM22_output_high  digitalWrite(S22,HIGH)
#define PWM22_output_low          digitalWrite(S22,LOW)

#define PWM23_output_high  digitalWrite(S23,HIGH)
#define PWM23_output_low          digitalWrite(S23,LOW)

#define PWM24_output_high  digitalWrite(S24,HIGH)
#define PWM24_output_low          digitalWrite(S24,LOW)

#define PWM25_output_high  digitalWrite(S25,HIGH)
#define PWM25_output_low          digitalWrite(S25,LOW)

#define PWM26_output_high  digitalWrite(S26,HIGH)
#define PWM26_output_low          digitalWrite(S26,LOW)

#define PWM27_output_high  digitalWrite(S27,HIGH)
#define PWM27_output_low          digitalWrite(S27,LOW)

#define PWM28_output_high  digitalWrite(S28,HIGH)
#define PWM28_output_low          digitalWrite(S28,LOW)

#define PWM29_output_high  digitalWrite(S29,HIGH)
#define PWM29_output_low          digitalWrite(S29,LOW)

#define PWM30_output_high  digitalWrite(S30,HIGH)
#define PWM30_output_low          digitalWrite(S30,LOW)

#define PWM31_output_high  digitalWrite(S31,HIGH)
#define PWM31_output_low          digitalWrite(S31,LOW)

#define PWM32_output_high  digitalWrite(S32,HIGH)
#define PWM32_output_low          digitalWrite(S32,LOW)

#define  Cmd_PC_Link    1
#define  Cmd_Clear      2
#define  Cmd_READ       3
#define  Cmd_DOWN       4
#define  Cmd_Enable     5
#define  Cmd_Disabled   6
#define  Cmd_Run        7
#define  Cmd_GO         8
#define  Cmd_ADC        9
#define  Cmd_SM         10
#define  Cmd_NULL       0

#define  Check_Addr       99
#define  eeprom_AddBaisc  167   //动作起始地址
#define  Group_MAX_Length 20    
#define	 Group_MAX_NUM		48
#define  Clear_Flag_ADDR	0                  //清空标记为1则为有数据，否则为空（这样做不需要对整个eeprom擦除）
#define  Offline_Flag_ADDR 1                  //是否脱机（0时为禁用，1-48为脱机运行的组号。
#define	 Circle_Flag_ADDR 2                  //是否循环
#define	 Circle_Time_ADDR 3                  //脱机运行次数
#define	 All_Group_NUM_ADDR   4                  //共有几组(共48组）
#define	 Group_Length_ADDR 10    		   //每次的动作数量

#define AT24C01		127
#define AT24C02		255
#define AT24C04		511
#define AT24C08		1023
#define AT24C16		2047
#define AT24C32		4095
#define AT24C64	        8191
#define AT24C128	16383
#define AT24C256	32767  
#define AT24C512	65535
//Mini STM32开发板使用的是24c02，所以定义EE_TYPE为AT24C02
#define EE_TYPE AT24C512
#define I2CBASEADDR 0x50

#define AD0 15
#define AD1 16

#define Trapezoidal_curve  1
#define S_curve  2

#define PWM_time_out 250

extern unsigned int PWM_value[33];
extern unsigned int PWM_value_old[7][33];
extern unsigned char flag_vpwm;
extern unsigned char flag_RecFul;

class Servo_PWM
{
private:    //私有成员，用来保存色彩的RGB分量
  unsigned char redata[1600];    // 定义接收数据变量数组
  unsigned char ASC_To_Valu(unsigned char);
  void accelerate_method(unsigned char, unsigned char, unsigned int);
  double dp_calculate(unsigned int, unsigned char);
  void Deal_commands(unsigned char*);
  void Serial_process(USBSerial*);
  void Uart_Cmd(void);
  void V_PWM(void);
  void RecStr_to_pos(void);
  void RrcStr_to_pwm(void);
  void RecStr_to_Down(void);
  static void Servo_PWM_callback1();
  static void Servo_PWM_callback2();
  static void Servo_PWM_callback3();
  static void Servo_PWM_callback4();
public:
  Servo_PWM(); //类的构造函数，与类名相同
  
  void Servo_PWM_initialize(void);
  void Set_timer_state(unsigned char state);
  void PWM_change(void);
  void Read_Data();
  void Servo_process(void);
  unsigned char AT24CXX_ReadOneByte(unsigned int);
  void AT24CXX_WriteOneByte(unsigned int, unsigned char);
  void AT24CXX_Read(unsigned int,unsigned char *,unsigned int);
  void AT24CXX_Write(unsigned int,unsigned char *,unsigned int);
  unsigned char AT24CXX_Check(void);
  void u8Tou16(unsigned char *,unsigned int *);
  void u16Tou8(unsigned char *,unsigned int *);
};

#endif // Servo_PWM_H




#include "Servo_PWM.h"

Servo_PWM PR32;
unsigned char testtext1[20] = {"ABCDEFGHIJKLMNOPQRS"};
unsigned char testtext2[20];
void setup()
{
  PR32.Servo_PWM_initialize();
}

void loop()
{
    PR32.Servo_process();  
}



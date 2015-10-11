
#include "Servo_PWM.h"

Servo_PWM PR32;

void setup()
{
  PR32.Servo_PWM_initialize();
}

void loop()
{
    PR32.Servo_process();  
}



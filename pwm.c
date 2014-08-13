#include "pwm.h"
#include <msp430f5529.h>

void initPWM()
{
    P1DIR |=BIT3; // 设置 P1.3为输出
    P1SEL |=BIT3; // 设置 P1.3为TA0.2输出
    P1DIR |=BIT4;
    P1SEL |=BIT4;
    
    TA0CCR0 = 1500; // 设置PWM 周期
    TA0CCTL2 = OUTMOD_7; // 设置PWM 输出模式为：7 - PWM复位/置位模式，
    TA0CCTL3 = OUTMOD_7;
    TA0CTL= TASSEL_2 +MC_1; // 设置TIMERA的时钟源为SMCLK, 计数模式为up,到CCR0再自动从0开始计数
    TA0CCR2=0;
    TA0CCR3=0;
}


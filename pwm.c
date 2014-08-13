#include "pwm.h"
#include <msp430f5529.h>

void initPWM()
{
    P1DIR |=BIT3; // ���� P1.3Ϊ���
    P1SEL |=BIT3; // ���� P1.3ΪTA0.2���
    P1DIR |=BIT4;
    P1SEL |=BIT4;
    
    TA0CCR0 = 1500; // ����PWM ����
    TA0CCTL2 = OUTMOD_7; // ����PWM ���ģʽΪ��7 - PWM��λ/��λģʽ��
    TA0CCTL3 = OUTMOD_7;
    TA0CTL= TASSEL_2 +MC_1; // ����TIMERA��ʱ��ԴΪSMCLK, ����ģʽΪup,��CCR0���Զ���0��ʼ����
    TA0CCR2=0;
    TA0CCR3=0;
}


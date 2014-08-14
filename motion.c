#include "motion.h"
#include <msp430f5529.h>

#define EN1_SET     (P6OUT |= BIT0)
#define EN1_CLR     (P6OUT &= ~BIT0)

#define EN2_SET     (P6OUT |= BIT1)
#define EN2_CLR     (P6OUT &= ~BIT1)

#define IN1_SET     (P6OUT |= BIT5)
#define IN1_CLR     (P6OUT &= ~BIT5)

#define IN2_SET     (P6OUT |= BIT4)
#define IN2_CLR     (P6OUT &= ~BIT4)

#define IN3_SET     (P6OUT |= BIT3)
#define IN3_CLR     (P6OUT &= ~BIT3)

#define IN4_SET     (P6OUT |= BIT2)
#define IN4_CLR     (P6OUT &= ~BIT2)

void initMotion()
{
    P6SEL = 0x00;
    P6DIR = 0xff;
    P6OUT = 0x00;
}

void forntMotion(u16 leftSpeed, u16 rightSpeed)
{
    TA0CCR2 = leftSpeed;
    TA0CCR3 = rightSpeed;
    
    IN1_CLR;
    IN2_SET;
    
    IN3_CLR;
    IN4_SET;
}

void backMotion(u16 leftSpeed, u16 rightSpeed)
{
    TA0CCR2 = leftSpeed;
    TA0CCR3 = rightSpeed;
    
    IN1_SET;
    IN2_CLR;
    
    IN3_SET;
    IN4_CLR;
}

void rightMotion(u16 leftSpeed, u16 rightSpeed)
{
    TA0CCR2 = leftSpeed;
    TA0CCR3 = rightSpeed;
    
    IN1_SET;
    IN2_CLR;
    
    IN3_CLR;
    IN4_SET;
}

void leftMotion(u16 leftSpeed, u16 rightSpeed)
{
    TA0CCR2 = leftSpeed;
    TA0CCR3 = rightSpeed;
    
    IN1_CLR;
    IN2_SET;
    
    IN3_SET;
    IN4_CLR;
    
}

void stop()
{
    IN1_CLR;
    IN2_CLR;
    
    IN3_CLR;
    IN4_CLR;
}
#include <msp430f5529.h>
#include "LDC1000_cmd.h"
#include "spi.h"
#include "motion.h"
#include "pwm.h"

#define CPU_F ((double)4000000) 
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) 
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0)) 

#define OUTSIDE 0
#define INSIDE 1

#define SAMPLE_COUNT 200
#define MAX_TURN_SPEED 700
#define MAX_FORNT_SPEED 600

#define SIDE_ERROR 150

//#define MOTION_SPEED 0
//
#define DELAY_TIME 70

#define MAX_TIME 5000

void SetVCoreUp(unsigned int level);


///////////////////////////////////
unsigned char proximtyData[2];
unsigned char frequencyData[3];

unsigned char data[6];

unsigned long fre;
unsigned int pro;
///////////////////////////////////

///////////////////////////////////
unsigned int comValue = 0;
unsigned int sideValue = 0;
unsigned int sideMinValue = 0;
///////////////////////////////////

#define RPMIN 0x3a
#define RPMAX 0x13

unsigned int c;
unsigned int a;
unsigned int b;

void initUART()
{
    P3SEL |= BIT3 + BIT4;   //P3.3为UCA0TXD
    UCA0CTL1 |= UCSWRST;    //置位UCSWRST,使USCI复位，在其为1时初始化所有USCI寄存器
    UCA0CTL1 |= UCSSEL_1;
    UCA0BR0 = 0x03;         //设置波特率为9600
    UCA0BR1 = 0x00;
    UCA0MCTL= UCBRS_3 + UCBRF_0;
    UCA0CTL1 &= ~UCSWRST;   //软件清除UCSWRST
    UCA0IE |= UCRXIE;       // 使能接收中断
}

#pragma vector = USCI_A0_VECTOR
__interrupt void USART_A0()
{
    switch(__even_in_range(UCA0IV, 4)) {
        case 0 :
        break;
        case 2 :
         
        break;
        case 4 : 
        break;
        default :break;
    }
}

void sendDataWithUART(unsigned int sendData)
{
    
    static unsigned char sendCount = 0;
    data[0] = (unsigned char)(sendData & 0x00ff); 
    data[1] = (unsigned char)(sendData >> 8) & 0x00ff; 
    while(1) {
        if (sendCount >= 2) {
            sendCount = 0;
            break;
        } else {
            UCA0TXBUF = *(data + sendCount);
            while (!(UCA0IFG & UCTXIFG));
            sendCount += 1;
        }
    }
}

void initClock()
{
    //  //UCS SETTING
    UCSCTL3 |= SELREF__REFOCLK;

    __bis_SR_register(SCG0);                  // Disable the FLL control loop
    UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
    UCSCTL1 = DCORSEL_6;                      // Select DCO range 24MHz operation
    UCSCTL2 = FLLD_0 + 731;                   // Set DCO Multiplier for 24MHz
                                                // (N + 1) * FLLRef = Fdco
                                               // (731 + 1) * 32768 = 24MHz
                                               // Set FLL Div = fDCOCLK/2
    __bic_SR_register(SCG0);                  // Enable the FLL control loop
    UCSCTL4 |= SELA__DCOCLK + SELS__REFOCLK + SELM__DCOCLK; //ACLK,SMCLK,MCLK Source select
    UCSCTL5 |= DIVPA_2;                                   //ACLK output divide
    UCSCTL6 |= XT1DRIVE_3 + XCAP_0;                       //XT1 cap
}

void initPort() 
{
       //PORT INIT

    P1DIR |= BIT0;                        // LDC CLK for Freq counter (set to output selected clock)
    P1SEL |= BIT0;

    // LEDs
    P7DIR |= BIT0;
    P4DIR |= BIT7;
    
    // key
    P2SEL &= ~BIT1;
    P2DIR &= ~BIT1;
    P2OUT |= BIT1;
    P2REN |= BIT1;
    

    // buzzer
    P8SEL &= ~BIT2;
    P8DIR |= BIT2;
    P8OUT &= ~BIT2;
    
    
    //INTB INIT
//  P1DIR &= ~BIT2;                            // Set P1.2 input
//  P1IES |= BIT2;                           // P1.2 Hi/Lo edge
//  P1IFG &= ~BIT2;                           // P1.2 IFG cleared
//  P1IE |= BIT2;                             // P1.2 interrupt enabled

    // initialize SPI
    P4DIR |= BIT0;  // Output
    P4SEL &= ~BIT0;
    
    P2SEL |= BIT2;
    P2DIR |= BIT2;
}

void initSPI()
{
    //SPI SETUP
    P4SEL |=BIT1 + BIT2 + BIT3;
    UCB1CTL1 |= UCSWRST;
    UCB1CTL0 |= UCMST+UCMSB+UCSYNC+UCCKPL;   // 3-pin, 8-bit SPI master,Clock polarity high, MSB
    UCB1CTL1 |= UCSSEL__SMCLK;                 // CLOCK ACLK
    UCB1BR0 = 0x06;
    UCB1BR1 = 0;
    UCB1CTL1 &= ~UCSWRST;
}



void initLDC1000()
{
       //read all REG value using default setting
    unsigned char orgVal[20];

    //write to register
    spi_writeByte(LDC1000_CMD_RPMAX,       RPMAX);
    spi_writeByte(LDC1000_CMD_RPMIN,       RPMIN);
    spi_writeByte(LDC1000_CMD_SENSORFREQ,  0xB3);
    spi_writeByte(LDC1000_CMD_LDCCONFIG,   0x12);
    spi_writeByte(LDC1000_CMD_CLKCONFIG,   0x00);
    spi_writeByte(LDC1000_CMD_INTCONFIG,   0x00);

    spi_writeByte(LDC1000_CMD_THRESHILSB,  0x50);
    spi_writeByte(LDC1000_CMD_THRESHIMSB,  0x14);
    spi_writeByte(LDC1000_CMD_THRESLOLSB,  0xC0);
    spi_writeByte(LDC1000_CMD_THRESLOMSB,  0x12);

    spi_writeByte(LDC1000_CMD_PWRCONFIG,   0x01);

    //read all registers

    spi_readBytes(LDC1000_CMD_REVID, &orgVal[0],12);
}

unsigned int getProximtyData()
{
    spi_readBytes(LDC1000_CMD_PROXLSB,&proximtyData[0],2);
    pro = 0;
    pro += proximtyData[1];
    pro <<= 8;
    pro += proximtyData[0];
    
    return pro;
}



unsigned int getMedianNum(unsigned int *bArray, int iFilterLen)  
{  
    int i,j;// 循环变量  
    unsigned int bTemp;  
      
    // 用冒泡法对数组进行排序  
    for (j = 0; j < iFilterLen - 1; j ++)  
    {  
        for (i = 0; i < iFilterLen - j - 1; i ++)  
        {  
            if (bArray[i] > bArray[i + 1])  
            {  
                // 互换  
                bTemp = bArray[i];  
                bArray[i] = bArray[i + 1];  
                bArray[i + 1] = bTemp;  
            }  
        }  
    }  
      
    // 计算中值  
    if ((iFilterLen & 1) > 0)  
    {  
        // 数组有奇数个元素，返回中间一个元素  
        bTemp = bArray[(iFilterLen + 1) / 2];  
    }  
    else  
    {  
        // 数组有偶数个元素，返回中间两个元素平均值  
        bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2;  
    }  
  
    return bTemp;  
}  

unsigned int getMaxOfArray(unsigned int *array, int length)
{
    int max = *(array);
    int i;
    for (i = 1; i < length; i++) {
      if (max < *(array + i)) {
         max = *(array + i);
      }
    }
    return max;
}

unsigned int getMinOfArray(unsigned int *array, int length)
{
    int min = *(array);
    int i;
    for (i = 1; i < length; i++) {
      if (min > *(array + i)) {
         min = *(array + i);
      }
    }
    return min;
}

unsigned int getCountWithValue(unsigned int *array, int length, unsigned int value)
{
    int i;
    unsigned int count;
    for (i = 0; i < length; ++i) {
        if (*(array + i) > value) {
            count ++;
        }
    }
    return count;
}


char isSide(unsigned int *sampWithArray, unsigned int length)
{

    return 1;
}

unsigned int maxValue() 
{
    unsigned int getValue;
    unsigned int max = 0;
    int i;
    for (i = 0; i < SAMPLE_COUNT; ++i) {
        getValue = getProximtyData();
        if(getValue > max && getValue < 15000) {
            max = getValue;
        }
    }
    return max;
}

typedef enum {
    fornt,
    right,
    left,
    line,
    stop,
    side,
    test,
    coilStop,
    blinkLED,
    coilTest,
    getComValue,
    inSerachCoil
}status;

char isACoil(status nowState)
{
    unsigned char i;
    unsigned char a[5];
    unsigned int value;
    while (1) {
        if (nowState == line) {
            for(i = 0; i < 5; ++i) {
                forntMotion(800, 800);
                delay_ms(10);
                stopMotion();
                delay_ms(20);
                value = maxValue();
                if(value <= (sideValue + SIDE_ERROR) 
                   && value >= (comValue + 50)) {
                    a[i] = 1;
                } else {
                    a[i] = 0;
                }
            }
            
            if ( a[0] + a[1] + a[2] + a[3] + a[4] >= 2) {
                return 1;
            }  else {
                return 0;
            }
        } else if (nowState == right) {
            //leftMotion(800, 800);
        
            for(i = 0; i < 5; ++i) {
                rightMotion(800, 800);
                delay_ms(10);
                stopMotion();
                delay_ms(20);
                value = maxValue();
                if(value <= (sideValue + SIDE_ERROR) 
                   && value >= (comValue + 50)) {
                    a[i] = 1;
                } else {
                    a[i] = 0;
                }
            }

            if ( a[0] + a[1] + a[2] + a[3] + a[4] >= 2) {
                return 1;
            }  else {
                return 0;
            }

        } else if (nowState == left) {
            //rightMotion(800, 800);

            for(i = 0; i < 5; ++i) {
                leftMotion(800, 800);
                delay_ms(10);
                stopMotion();
                delay_ms(20);
                value = maxValue();
                if(value <= (sideValue + SIDE_ERROR) 
                   && value >= (comValue + 50)) {
                    a[i] = 1;
                } else {
                    a[i] = 0;
                }
            }            
            if ( a[0] + a[1] + a[2] + a[3] + a[4] >= 2) {
                return 1;
            }  else {
                return 0;
            }
        }
    }

    
}




void motionControl()
{
    static status nowState  = inSerachCoil;
    static status nextState = inSerachCoil;
    static status lastState = inSerachCoil;
    static unsigned int selValue = 0;
    
    static char coilIsOpen = 0;
    
    static char isInPane = 0;
    static unsigned int time = 0;
    
    static char flag = 0;
    
    unsigned int motionValue;
    
   
    unsigned char dataIn; 
    
    switch(nowState) {
        case inSerachCoil :
            delay_ms(20);
            dataIn = P2IN & BIT1;
            if (dataIn == 0) {
                flag = 1;
                coilIsOpen = 1;
                nowState = blinkLED;
                break;
            } else {
                nowState = blinkLED;
                coilIsOpen = 0;
                flag = 0;
                break;
            } 
        break;
        case blinkLED :
            dataIn = P2IN & BIT1;
            if (dataIn == 0 && flag == 0) {
                nowState = getComValue;
                forntMotion(MAX_FORNT_SPEED, MAX_FORNT_SPEED);
                delay_ms(40);
                break;
            } else if (dataIn != 0 && flag == 1) {
                flag = 0;
            } else if (P1IN & BIT1 == 0) {
                nowState = coilTest;
                break;
            } else if ((P4OUT & (BIT7)) == BIT7) {
                P4OUT &= ~BIT7;
            } else {
                P4OUT |= BIT7;
            }
            

            
            delay_ms(80);
        break;
///////////////////////////////////////////////////////////////////////////        
        case getComValue:
            comValue = maxValue();
            sendDataWithUART(comValue);
            isInPane = OUTSIDE;
            nowState = fornt;
        break;
            
        case fornt :
            motionValue = getProximtyData();
            if(motionValue >= comValue + 50) {
                sideValue = maxValue();
                motionValue = maxValue();
                sideValue = sideValue > motionValue ? sideValue : motionValue;
                if (sideValue >= comValue + 100) {
                    sendDataWithUART(sideValue);
                    nowState = line;
                    lastState = left;
                    forntMotion(MAX_FORNT_SPEED, MAX_FORNT_SPEED);
                    delay_ms(40);
                    if (isInPane == OUTSIDE) {
                        isInPane = INSIDE;
                    } else {
                        isInPane = OUTSIDE;
                    }
                    break;
                }
            }
        break;

//////////////////////////////////////////////////////////////////////
        case line : 
            motionValue = getProximtyData();
            if(motionValue >= comValue + 50) {
                motionValue = maxValue();
                if (motionValue >= sideValue + SIDE_ERROR) {
                    nowState = stop;
                    stopMotion();
                    lastState = line;
                    sendDataWithUART(motionValue);
                    break;
                } else if (motionValue >= (comValue + 50) && coilIsOpen == 1 && isACoil(line) == 1) {
                    nowState = coilStop;
                    break;
                }else if (motionValue >= (comValue + 50) ) {
                    nowState = side;
                    
                    if (lastState == left) {
                        nextState = right;
                    } else {
                        nextState = left;
                    }
                    
                    lastState = line;
                    if (isInPane == OUTSIDE) {
                        isInPane = INSIDE;
                    } else {
                        isInPane = OUTSIDE;
                    }
                    time = 0;
                    break;
                } 
            }

        break;
////////////////////////////////////////////////////////////////////////
        case side :
            delay_ms(DELAY_TIME);
            motionValue = maxValue();
            if (motionValue <= (sideValue + SIDE_ERROR) 
                && motionValue >= (comValue + 50)) {
                break;
            } else if (nextState == left) {
                nowState = left;
                leftMotion(MAX_TURN_SPEED, MAX_TURN_SPEED);
            } else if (nextState == right) {
                nowState = right;
                rightMotion(MAX_TURN_SPEED, MAX_TURN_SPEED);
            } else {
                nowState = line;
                forntMotion(MAX_FORNT_SPEED, MAX_FORNT_SPEED);
            }
        break; 
 /////////////////////////////////////////////////////////////////////////////       
        case left :
            time++;

            motionValue = getProximtyData();
            if(motionValue >= comValue + 50) {
                motionValue = maxValue();

                if (motionValue >= sideValue + SIDE_ERROR ) {
                    //sendDataWithUART(data, 5);
                    nowState = stop;
                    lastState = left;
                    stopMotion();
                    sendDataWithUART(motionValue);
                    break;
                } else if (motionValue >= (comValue + 50) == 1 && coilIsOpen == 1 && isACoil(left)) {
                    nowState = coilStop;
                    break;
                } else if (motionValue >= (comValue + 50)) {
                    nowState = side;
                    if (lastState == line) {
                        nextState = line;
                    } else {
                        nextState = right;
                    }
                    lastState = left;

                    if (isInPane == 0) {
                        isInPane = 1;
                    } else {
                        isInPane = 0;
                    }
                    time = 0;
                    
                    break;
                } 
                
            } else if (time > MAX_TIME) {
                forntMotion(MAX_TURN_SPEED,MAX_TURN_SPEED);
                nowState = line;
                lastState = left;
                time = 0;
                break;
            }
        break;
//////////////////////////////////////////////////////////////////        
        case right :
            time++;
            motionValue = getProximtyData();
            if(motionValue >= comValue + 50) { 
                motionValue = maxValue();

                if (motionValue >= sideValue + SIDE_ERROR) {
                    nowState = stop;
                    lastState = right;
                    stopMotion();
                    sendDataWithUART(motionValue);
                    break;
                } else if (motionValue >= (comValue + 50) && coilIsOpen == 1 && isACoil(right) == 1 ) {
                    nowState = coilStop;
                    break;
                } else if (motionValue >= (comValue + 50)) {
                    nowState = side;
                    if (lastState == line) {
                        nextState = line;
                    } else {
                        nextState = left;
                    }
                    lastState = right;

                    if (isInPane == 0) {
                        isInPane = 1;
                    } else {
                        isInPane = 0;
                    }
                    time = 0;
                    break;
                } 
            } else if (time > MAX_TIME) {
                forntMotion(MAX_TURN_SPEED,MAX_TURN_SPEED);
                nowState = line;
                lastState = right;
                time = 0;
                break;
           }
        break;
 
 /////////////////////////////////////////////////////////////////////////////  
        case coilStop :
            stopMotion();
            P8OUT |= BIT2;
            while(1);
        break;
        
/////////////////////////////////////////////////////////////////////////////
        case stop :
            motionValue = 0;
            if (lastState == line) {
                backMotion(800, 800);
                delay_ms(30);
                while(1) {
                    backMotion(800, 800);
                    delay_ms(10);
                    stopMotion();
                    delay_ms(20);
                    motionValue = maxValue();
                    if (selValue < motionValue) {
                        selValue = motionValue; 

                    } else {
                        forntMotion(800, 800);
                        delay_ms(10);
                        break;
                    }
                }
            } else if (lastState == left) {
                rightMotion(800, 800);
                delay_ms(30);
                while(1) {
                    rightMotion(800, 800);
                    delay_ms(10);
                    stopMotion();
                    delay_ms(20);
                    motionValue = maxValue();
                    if (selValue < motionValue) {
                        selValue = motionValue; 

                    } else {
                        leftMotion(800, 800);
                        delay_ms(10);
                        break;
                    }
                }
            } else {
                leftMotion(800, 800);
                while(1) {
                    leftMotion(800, 800);
                    delay_ms(10);
                    stopMotion();
                    delay_ms(20);
                    motionValue = maxValue();
                    if (selValue < motionValue) {
                        selValue = motionValue; 

                    } else {
                        rightMotion(800, 800);
                        delay_ms(10);
                        break;
                    }
                }
            }
            stopMotion();
            P8OUT |= BIT2;
            while(1);
        break;
/////////////////////////////////////////////////////////////////////////////

        case test :
            delay_ms(300);
            forntMotion(800, 800);
            delay_ms(20);
            stopMotion();
            while(1);
        break;
        
        case coilTest :
            
        break;
        default : 
        ;
        break;
    }
}

void main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
/**
    SetVCoreUp(1);
    SetVCoreUp(2);
    SetVCoreUp(3);*/
    
//   initClock();
    //UCSCTL4|=SELS_5;
    initPort();
    initSPI();
    initLDC1000();
    initUART();
    initMotion();
    initPWM();
    _EINT();
    //read all registers using extended SPI
    while (1) {
        motionControl();
       __no_operation();
    }
}

void SetVCoreUp (unsigned int level)
{
        // Open PMM registers for write access
        PMMCTL0_H = 0xA5;
        // Make sure no flags are set for iterative sequences
//      while ((PMMIFG & SVSMHDLYIFG) == 0);
//      while ((PMMIFG & SVSMLDLYIFG) == 0);
        // Set SVS/SVM high side new level
        SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
        // Set SVM low side to new level
        SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
        // Wait till SVM is settled
        while ((PMMIFG & SVSMLDLYIFG) == 0);
        // Clear already set flags
        PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
        // Set VCore to new level
        PMMCTL0_L = PMMCOREV0 * level;
        // Wait till new level reached
        if ((PMMIFG & SVMLIFG))
        while ((PMMIFG & SVMLVLRIFG) == 0);
        // Set SVS/SVM low side to new level
        SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
        // Lock PMM registers for write access
        PMMCTL0_H = 0x00;
}



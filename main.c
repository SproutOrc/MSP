#include <msp430f5529.h>
#include "LDC1000_cmd.h"
#include "spi.h"
#include "motion.h"
#include "pwm.h"

#define SAMPLE_COUNT 50
#define MOTION_COUNT 10
#define MAX_TURN_SPEED 700
#define MAX_FORNT_SPEED 700
//#define MOTION_SPEED 0
#define MOTION_ERROR 50

#define MAX_TIME 700

unsigned int MOTION_SPEED = 680;

void SetVCoreUp(unsigned int level);


///////////////////////////////////
unsigned char proximtyData[2];
unsigned char frequencyData[3];

unsigned char data[6];
unsigned char *sendDataValue;
unsigned char maxSendCount;

unsigned long fre;
unsigned int pro;
///////////////////////////////////

///////////////////////////////////
unsigned int comValue = 0;
unsigned int sideMaxValue = 0;
unsigned int sideMinValue = 0;
///////////////////////////////////

unsigned int filter[SAMPLE_COUNT] = {0};

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
          /*
            if (re == 0) {
                TA0CCR2 = UCA0RXBUF / 255.0 * 1500;
                re = 1;
            }
            else {
                TA0CCR3 = UCA0RXBUF / 255.0 * 1500;
                re = 0;
            }
          */
          MOTION_SPEED = UCA0RXBUF / 255.0 * 1500;
        break;
        case 4 : 
        break;
        default :break;
    }
}

void sendDataWithUART(unsigned char *data, unsigned char n)
{
    
    static unsigned char sendCount = 0;
    while(1) {
        if (sendCount >= n) {
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
    UCB1CTL1 |= UCSSEL_1;                 // CLOCK ACLK
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

typedef enum {
    fornt,
    getMotionValue,
    lag,
    right,
    left,
    wait,
    line,
    stop,
    side,
    test,
    speed_test,
    blinkLED,
    getComValue
}status;


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

void motionControl()
{
    static status nowState  = blinkLED;
    static status nextState = blinkLED;
    static status lastState = blinkLED;
    static char isInPane = 0;
    static unsigned long time = 0;
    static unsigned char flag = 0;
    
    static unsigned int lastTurnValue = 0;
    unsigned int motionValue;
    
   
    unsigned char dataIn; 
    
    int i;
    switch(nowState) {
        
        case blinkLED :
            dataIn = P2IN & BIT1;
            if (dataIn == 0 && flag == 0) {
                flag = 1;
                nowState = getComValue;
                break;
            } else if (dataIn == 0x02){
                flag = 0;
            }
            if ((P4OUT & (BIT7)) == BIT7) {
                P4OUT &= ~BIT7;
            } else {
                P4OUT |= BIT7;
            }
            for (c = 2; c > 0; --c){
                for (a = 500; a > 0; --a){
                    for (b = 50; b > 0; --b);
                }
            }
        break;
///////////////////////////////////////////////////////////////////////////        
        case getComValue:
            for (i = 0; i < SAMPLE_COUNT; ++i) {
                filter[i] = getProximtyData();
            }
            comValue = getMaxOfArray(filter, SAMPLE_COUNT);
            
            data[0] = 0xff;
            data[1] = (unsigned char)(comValue & 0x00ff); 
            data[2] = (unsigned char)((comValue >> 8) & 0x00ff); 
            sendDataWithUART(data, 5);
            
            nowState = fornt;
        break;
            
        case fornt :
            forntMotion(MAX_FORNT_SPEED, MAX_FORNT_SPEED);

            motionValue = getProximtyData();
            if(motionValue >= comValue + 50) {
                for (i = 0; i < SAMPLE_COUNT; ++i) {
                    filter[i] = getProximtyData();
                }
                sideMaxValue = getMaxOfArray(filter, SAMPLE_COUNT);
                sideMinValue = getMinOfArray(filter, SAMPLE_COUNT);
                if (sideMaxValue >= comValue + 50) {
                    data[0] = 0xff;
                    data[1] = (unsigned char)(sideMaxValue & 0x00ff); 
                    data[2] = (unsigned char)(sideMaxValue >> 8) & 0x00ff; 
                    sendDataWithUART(data, 5);
                    nowState = line;
                    lastState = left;
                    forntMotion(MAX_FORNT_SPEED, MAX_FORNT_SPEED);
                    flag = 0;
                    if (isInPane == 0) {
                        isInPane = 1;
                    } else {
                        isInPane = 0;
                    }
                    break;
                }
            }
        break;

        case side :
            motionValue = getProximtyData();
            if (motionValue <= (sideMaxValue + 200) 
                && motionValue >= (comValue + 50)) {
                for (a = 500; a > 0; --a) 
                    for (b = 50; b > 0; --b);
            } else if (lastState == left) {
                nowState = right;
                lastState = line;
                rightMotion(MAX_TURN_SPEED, MAX_TURN_SPEED);
            } else {
                nowState = left;
                lastState = line;
                leftMotion(MAX_TURN_SPEED,MAX_TURN_SPEED);
            }
        break;

        case line : 
            motionValue = getProximtyData();
            if(motionValue >= comValue + 50) {
                for (i = 0; i < SAMPLE_COUNT; ++i) {
                    filter[i] = getProximtyData();
                } 
                motionValue = getMaxOfArray(filter, SAMPLE_COUNT);
                // data[0] = 0xff;
                // data[1] = (unsigned char)(motionValue & 0x00ff); 
                // data[2] = (unsigned char)((motionValue >> 8) & 0x00ff);
                // sendDataWithUART(data, 5);
                if (motionValue >= sideMaxValue + 200) {
                    nowState = stop;
                    backMotion(MAX_FORNT_SPEED, MAX_FORNT_SPEED);
                    data[0] = 0xff;
                    data[1] = (unsigned char)(motionValue & 0x00ff); 
                    data[2] = (unsigned char)(motionValue >> 8) & 0x00ff; 
                    sendDataWithUART(data, 5);
                    break;
                } else if (motionValue <= (sideMaxValue + 200) 
                        && motionValue >= (comValue + 50) ) {
                    nowState = side;
                    forntMotion(MAX_FORNT_SPEED, MAX_FORNT_SPEED);
                    flag = 0;
                    if (isInPane == 0) {
                        isInPane = 1;
                    } else {
                        isInPane = 0;
                    }
                    break;
                } 
            }

        break;
        
        case lag :
            for (a = 500; a > 0; --a) 
              for (b = 50; b > 0; --b);
            nowState = nextState;
            forntMotion(MAX_FORNT_SPEED, MAX_FORNT_SPEED);
        break;
 /////////////////////////////////////////////////////////////////////////////       
        case left :
            time++;
            if (flag == 0) {
                    for (a = 500; a > 0; --a) 
                        for (b = 50; b > 0; --b);

                    flag = 1;
            }
            motionValue = getProximtyData();
            if(motionValue >= comValue + 50) {
                
                // leftMotion(MAX_TURN_SPEED,0);
                for (i = 0; i < SAMPLE_COUNT; ++i) {
                    filter[i] = getProximtyData();
                } 
                motionValue = getMaxOfArray(filter, SAMPLE_COUNT);

                if (motionValue >= sideMaxValue + 200 ) {
                    //sendDataWithUART(data, 5);
                    nowState = stop;
                    leftMotion(0, MAX_TURN_SPEED);
                    data[0] = 0xff;
                    data[1] = (unsigned char)(motionValue & 0x00ff); 
                    data[2] = (unsigned char)(motionValue >> 8) & 0x00ff; 
                    sendDataWithUART(data, 5);
                    break;
                } else if (motionValue < (sideMaxValue + 200) 
                        && motionValue >= (comValue + 50) ) {
                    nowState = lag;
                    if (lastState == line) {
                        nextState = line;
                    } else {
                        rightMotion(MAX_TURN_SPEED, MAX_TURN_SPEED);
                        nextState = right;
                    }
                    lastState = left;
                    flag = 0;
                    if (isInPane == 0) {
                        isInPane = 1;
                    } else {
                        isInPane = 0;
                    }
                    break;
                } else if (time > MAX_TIME) {
                    rightMotion(MAX_TURN_SPEED, MAX_TURN_SPEED);
                    nowState = right;
                    lastState = left;
                    time = 0;
                    flag = 0;
                    break;
                }
            }
        break;
//////////////////////////////////////////////////////////////////        
        case right :
            time++;
            if (flag == 0) {
                for (a = 500; a > 0; --a) 
                    for (b = 50; b > 0; --b);

                flag = 1;
            }
            motionValue = getProximtyData();
            if(motionValue >= comValue + 50) {
                for (i = 0; i < SAMPLE_COUNT; ++i) {
                    filter[i] = getProximtyData();
                } 
                motionValue = getMaxOfArray(filter, SAMPLE_COUNT);

                if (motionValue >= sideMaxValue + 200) {
                    nowState = stop;
                    leftMotion(MAX_TURN_SPEED, 0);
                    data[0] = 0xff;
                    data[1] = (unsigned char)(motionValue & 0x00ff); 
                    data[2] = (unsigned char)(motionValue >> 8) & 0x00ff; 
                    sendDataWithUART(data, 5);
                    break;
                } else if (motionValue < (sideMaxValue + 200) 
                        && motionValue >= (comValue + 50)) {
                    nowState = lag;
                    if (lastState == line) {
                        nextState = line;
                    } else {
                        leftMotion(MAX_TURN_SPEED,MAX_TURN_SPEED);
                        nextState = left;
                    }
                    
                    lastState = right;
                    flag = 0;
                    if (isInPane == 0) {
                        isInPane = 1;
                    } else {
                        isInPane = 0;
                    }
                    break;
                } else if (time > MAX_TIME) {
                    leftMotion(MAX_TURN_SPEED,MAX_TURN_SPEED);
                    nowState = left;
                    lastState = right;
                    time = 0;
                    flag = 0;
                    break;
                }
            }
        break;


        
 /////////////////////////////////////////////////////////////////////////////       
        case stop :
            for (a = 500; a > 0; --a) 
              for (b = 50; b > 0; --b);
            stopMotion();
            P8OUT |= BIT2;
        break;

        case test : 
            dataIn = P2IN & BIT1;
            if (dataIn == 0 && flag == 0) {
                flag = 1;
                motionValue = getProximtyData();
                data[0] = 0xff;
                data[1] = (unsigned char)(motionValue & 0x00ff); 
                data[2] = (unsigned char)((motionValue >> 8) & 0x00ff);
                sendDataWithUART(data, 5);
                break;
            } else if (dataIn == 0x02){
                flag = 0;
            }
            if ((P4OUT & (BIT7)) == BIT7) {
                P4OUT &= ~BIT7;
            } else {
                P4OUT |= BIT7;
            }
            for (c = 2; c > 0; --c){
                for (a = 500; a > 0; --a){
                    for (b = 50; b > 0; --b);
                }
            }

            
        break;

        case speed_test :
             dataIn = P2IN & BIT1;
             stopMotion();
            if (dataIn == 0 && flag == 0) {
                flag = 1;
                motionValue = getProximtyData();
                data[0] = 0xff;
                data[1] = (unsigned char)(motionValue & 0x00ff); 
                data[2] = (unsigned char)((motionValue >> 8) & 0x00ff);
                sendDataWithUART(data, 5);
                forntMotion(MAX_TURN_SPEED, MAX_TURN_SPEED);
                for (a = 500; a > 0; --a) 
                    for (b = 50; b > 0; --b);

                break;
            } else if (dataIn == 0x02){
                flag = 0;
            }
            if ((P4OUT & (BIT7)) == BIT7) {
                P4OUT &= ~BIT7;
            } else {
                P4OUT |= BIT7;
            }
            for (c = 2; c > 0; --c){
                for (a = 500; a > 0; --a){
                    for (b = 50; b > 0; --b);
                }
            }
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



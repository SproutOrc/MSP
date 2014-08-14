#include <msp430f5529.h>
#include "LDC1000_cmd.h"
#include "spi.h"
#include "motion.h"
#include "pwm.h"


void SetVCoreUp(unsigned int level);

unsigned char proximtyData[2];
unsigned char frequencyData[3];

unsigned char data[6];
unsigned char *sendDataValue;
unsigned char maxSendCount;

unsigned long fre;
unsigned int pro;

#define RPMIN 0x3a
#define RPMAX 0x13

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
    static char re = 0;
    switch(__even_in_range(UCA0IV, 4)) {
        case 0 :
        break;
        case 2 :
            if (re == 0) {
                TA0CCR2 = UCA0RXBUF / 255.0 * 1500;
                re = 1;
            }
            else {
                TA0CCR3 = UCA0RXBUF / 255.0 * 1500;
                re = 0;
            }
        break;
        case 4 : 
        break;
        default :break;
    }
}

void sendDataWithUART(unsigned char n)
{
    static unsigned char sendCount = 0;
    while(1) {
        if (sendCount >= n) {
            sendCount = 0;
            break;
        } else {
            UCA0TXBUF = data[sendCount];
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
    spi_writeByte(LDC1000_CMD_SENSORFREQ,  0x94);
    spi_writeByte(LDC1000_CMD_LDCCONFIG,   0x17);
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

void initTime()
{
    
}

void main(void) {
    int i;
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
        
        spi_readBytes(LDC1000_CMD_PROXLSB,&proximtyData[0],2);
        spi_readBytes(LDC1000_CMD_FREQCTRLSB,&frequencyData[0],3);
        pro = 0;
        pro += proximtyData[1];
        pro <<= 8;
        pro += proximtyData[0];
        
        fre = 0;
        fre += frequencyData[2];
        fre <<= 8;
        fre += frequencyData[1];
        fre <<= 8;
        fre += frequencyData[0];
        i++;
        data[0] = 0xff;
        data[1] = proximtyData[0];
        data[2] = proximtyData[1];;
        data[3] = frequencyData[0];
        data[4] = frequencyData[1];
        data[5] = frequencyData[2];
        
        sendDataWithUART(6);
        for (a = 500; a > 0; --a)
            for (b = 50; b > 0; --b);
        
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

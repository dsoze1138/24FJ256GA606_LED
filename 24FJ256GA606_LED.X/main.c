/*  
 *     File: main.c 
 *   Target: PIC24FJ256GA606
 *      IDE: MPLABX v4.00  
 * Compiler: XC16   v1.32
 *  
 * Description: 
 *  
 *  Validation code for GB206 to GA606
 *  port of the PWM modulators for the
 *  S700 scanner project.
 *
 * Notes: 
 *  Yellow LED on GPIO pin RB4. ON = 1, OFF = 0.
 * 
 *  To see the linker script file used add '--save-gld' to 
 *  the additional options of the xc16-ld properties.
 *
 *                                                   PIC24FJ256GA606
 *             +----------+               +----------+               +----------+               +----------+
 *       <>  1 : RE5      :   OCM7A <> 17 : RB6/RP6  :         <> 33 : RF3      :         <> 49 : RD1      :
 *       <>  2 : RE6      :   OCM5A <> 18 : RB7/RP7  :         <> 34 : RF2      :         <> 50 : RD2      :
 *       <>  3 : RE7      :     3v3 -> 19 : AVDD     :     3v3 -> 35 : RF6      :         <> 51 : RD3      :
 * OCM1A <>  4 : RG6      :     GND -> 20 : AVSS     :         <> 36 : RG3      :         <> 52 : RD4      :
 *       <>  5 : RG7      :   OCM4A <> 21 : RB8/RP8  :         <> 37 : RG2      :         <> 53 : RD5      :
 * OCM2A <>  6 : RG8      :     OC2 <> 22 : RB9/RP9  :     3v3 -> 38 : VDD      :         <> 54 : RD6      :
 *   VPP ->  7 : MCLR     :         <> 23 : RB10     :         <> 39 : RC12     :         <> 55 : RD7      :
 *       <>  8 : RG9      :         <> 24 : RB11     :         <> 40 : RC15     :    10uF -> 56 : VCAP     :
 *   GND ->  9 : VSS      :     GND -> 25 : VSS      :     GND -> 41 : VSS      :     3v3 -> 57 : N/C      :
 *   3v3 -> 10 : VDD      :     3v3 -> 26 : VDD      :     OC1 <> 42 : RD8/RP2  :         <> 58 : RF0      :
 * OCM3A <> 11 : RB5      :         <> 27 : RB12     :         <> 43 : RD9      :         <> 59 : RF1      :
 *   LED <> 12 : RB4      :         <> 28 : RB13     :         <> 44 : RD10     :         <> 60 : RE0      :
 *       <> 13 : RB3      :         <> 29 : RB14     :         <> 45 : RD11     :         <> 61 : RE1      :
 *       <> 14 : RB2      :         <> 30 : RB15     :   OCM6A <> 46 : RD0/RP11 :         <> 62 : RE2      :
 *   PGC <> 15 : RB1/PGC1 :         <> 31 : RF4      :         <> 47 : RC13     :         <> 63 : RE3      :
 *   PGD <> 16 : RB0/PGD1 :         <> 32 : RF5      :         <> 48 : RC14     :         <> 64 : RE4      :
 *             +----------+               +----------+               +----------+               +----------+
 *                                                    TQFP-64
 *  
 */  

#pragma config BTMODE = SINGLE          /* Boot Mode Configuration bits (Device is in Single Boot (legacy) mode) */
#pragma config BWRP = OFF               /* Boot Segment Write-Protect bit (Boot Segment may be written) */
#pragma config BSS = DISABLED           /* Boot Segment Code-Protect Level bits (No Protection (other than BWRP)) */
#pragma config BSEN = OFF               /* Boot Segment Control bit (No Boot Segment) */
#pragma config GWRP = OFF               /* General Segment Write-Protect bit (General Segment may be written) */
#pragma config GSS = DISABLED           /* General Segment Code-Protect Level bits (No Protection (other than GWRP)) */
#pragma config CWRP = OFF               /* Configuration Segment Write-Protect bit (Configuration Segment may be written) */
#pragma config CSS = DISABLED           /* Configuration Segment Code-Protect Level bits (No Protection (other than CWRP)) */
#pragma config AIVTDIS = OFF            /* Alternate Interrupt Vector Table bit (Disabled AIVT) */
#pragma config BSLIM = 0x1FFF           /* Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit) */
#pragma config FNOSC = FRCPLL           /* Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) ) */
#pragma config PLLMODE = PLL96DIV2      /* PLL Mode Selection (96 MHz PLL. (8 MHz input)) */
#pragma config IESO = OFF               /* Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source) */
#pragma config POSCMD = NONE            /* Primary Oscillator Mode Select bits (Primary Oscillator disabled) */
#pragma config OSCIOFCN = ON            /* OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin) */
#pragma config SOSCSEL = OFF            /* SOSC Power Selection Configuration bits (Digital (SCLKI) mode) */
#pragma config PLLSS = PLL_FRC          /* PLL Secondary Selection Configuration bit (PLL is fed by the on-chip Fast RC (FRC) oscillator) */
#pragma config IOL1WAY = OFF            /* Peripheral pin select configuration bit (Allow multiple reconfigurations) */
#pragma config FCKSM = CSDCMD           /* Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled) */
#pragma config WDTPS = PS32768          /* Watchdog Timer Postscaler bits (1:32,768) */
#pragma config FWPSA = PR128            /* Watchdog Timer Prescaler bit (1:128) */
#pragma config FWDTEN = ON_SWDTEN       /* Watchdog Timer Enable bits (WDT Enabled/Disabled (controlled using SWDTEN bit)) */
#pragma config WINDIS = OFF             /* Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode) */
#pragma config WDTWIN = WIN25           /* Watchdog Timer Window Select bits (WDT Window is 25% of WDT period) */
#pragma config WDTCMX = WDTCLK          /* WDT MUX Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits) */
#pragma config WDTCLK = LPRC            /* WDT Clock Source Select bits (WDT uses LPRC) */
#pragma config BOREN = SBOREN           /* Brown Out Enable bit (Controlled by SBOREN) */
#pragma config LPCFG = OFF              /* Low power regulator control (No Retention Sleep) */
#pragma config DNVPEN = ENABLE          /* Downside Voltage Protection Enable bit (Downside protection enabled using ZPBOR when BOR is inactive) */
#pragma config ICS = PGD1               /* ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1) */
#pragma config JTAGEN = OFF             /* JTAG Enable bit (JTAG is disabled) */
#pragma config BTSWP = OFF              /* BOOTSWP Disable (BOOTSWP instruction disabled) */
#pragma config ALTCMPI = DISABLE        /* Alternate Comparator Input Enable bit (C1INC, C2INC, and C3INC are on their standard pin locations) */
#pragma config TMPRPIN = OFF            /* Tamper Pin Enable bit (TMPRN pin function is disabled) */
#pragma config SOSCHP = ON              /* SOSC High Power Enable bit (valid only when SOSCSEL = 1 (Enable SOSC high power mode (default)) */
#pragma config ALTVREF = ALTREFEN       /* Alternate Voltage Reference Location Enable bit (VREF+ and CVREF+ on RA10, VREF- and CVREF- on RA9) */

#include <xc.h>
#include "init.h"
    
/* define map input pin numbers */ 
enum 
{   
    RPI_RB0  = 0,  /* RPI00 */ 
    RPI_RB1 ,      /* RPI01 */ 
    RPI_RD8 ,      /* RPI02 */ 
    RPI_RD10,      /* RPI03 */ 
    RPI_RD9 ,      /* RPI04 */ 
    RPI_RB6  = 6,  /* RPI06 */ 
    RPI_RB7 ,      /* RPI07 */ 
    RPI_RB8 ,      /* RPI08 */ 
    RPI_RB9 ,      /* RPI09 */ 
    RPI_RF4 ,      /* RPI10 */ 
    RPI_RD0 ,      /* RPI11 */ 
    RPI_RD11,      /* RPI12 */ 
    RPI_RB2 ,      /* RPI13 */ 
    RPI_RB14,      /* RPI14 */ 
    RPI_RF3  = 16, /* RPI16 */ 
    RPI_RF5 ,      /* RPI17 */ 
    RPI_RB5 ,      /* RPI18 */ 
    RPI_RG8 ,      /* RPI19 */ 
    RPI_RD5 ,      /* RPI20 */ 
    RPI_RG6 ,      /* RPI21 */ 
    RPI_RD3 ,      /* RPI22 */ 
    RPI_RD2 ,      /* RPI23 */ 
    RPI_RD1 ,      /* RPI24 */ 
    RPI_RD4 ,      /* RPI25 */ 
    RPI_RG7 ,      /* RPI26 */ 
    RPI_RG9 ,      /* RPI27 */ 
    RPI_RB4 ,      /* RPI28 */ 
    RPI_RB15,      /* RPI29 */ 
    RPI_RF2 ,      /* RPI30 */ 
    RPI_RC14 = 37, /* RPI37 */ 
    RPI_NONE = 0x3f 
};  
    
/* define map output function numbers */ 
enum 
{   
    RPO_NONE    = 0,    /* (Pin Disabled)            */
    RPO_C1OUT   = 1,    /* Comparator 1 Output       */
    RPO_C2OUT   = 2,    /* Comparator 2 Output       */
    RPO_C3OUT   = 26,   /* Comparator 3 Output       */
    RPO_SDO1    = 7,    /* SPI1 Data Output          */
    RPO_SCK1OUT = 8,    /* SPI1 Clock Output         */
    RPO_SS1OUT  = 9,    /* SPI1 Slave Select Output  */
    RPO_SDO2    = 10,   /* SPI2 Data Output          */
    RPO_SCK2OUT = 11,   /* SPI2 Clock Output         */
    RPO_SS2OUT  = 12,   /* SPI2 Slave Select Output  */
    RPO_SDO3    = 23,   /* SPI3 Data Output          */
    RPO_SCK3OUT = 24,   /* SPI3 Clock Output         */
    RPO_SS3OUT  = 25,   /* SPI3 Slave Select Output  */
    RPO_OC1     = 13,   /* Output Compare 1          */
    RPO_OC2     = 14,   /* Output Compare 2          */
    RPO_OC3     = 15,   /* Output Compare 3          */
    RPO_OCM4    = 16,   /* CCP4 Output Compare       */
    RPO_OCM5    = 17,   /* CCP5 Output Compare       */
    RPO_OCM6    = 18,   /* CCP6 Output Compare       */
    RPO_OCM7    = 27,   /* CCP7 Output Compare       */
    RPO_U1TX    = 3,    /* UART1 Transmit            */
    RPO_U1RTS   = 4,    /* UART1 Request-to-Send     */
    RPO_U2TX    = 5,    /* UART2 Transmit            */
    RPO_U2RTS   = 6,    /* UART2 Request-to-Send     */
    RPO_U3TX    = 19,   /* UART3 Transmit            */
    RPO_U3RTS   = 20,   /* UART3 Request-to-Send     */
    RPO_U4TX    = 21,   /* UART4 Transmit            */
    RPO_U4RTS   = 22,   /* UART4 Request-to-Send     */
    RPO_REFO    = 28,   /* Reference Clock Output    */
    RPO_CLC1OUT = 29,   /* CLC1 Output               */
    RPO_CLC2OUT = 30,   /* CLC2 Output               */
    RPO_RTCC    = 31,   /* RTCC Output               */
};  
    
/* Initialize this PIC */ 
void PIC_init(void) 
{   
    unsigned int ClockSwitchTimeout;

    /* 
    ** Disable all interrupt sources 
    */ 
    __builtin_disi(0x3FFF); /* disable interrupts for 16383 cycles */ 
    IEC0 = 0; 
    IEC1 = 0; 
    IEC2 = 0; 
    IEC3 = 0; 
    IEC4 = 0; 
    IEC5 = 0; 
    IEC6 = 0; 
    IEC7 = 0; 
    __builtin_disi(0x0000); /* enable interrupts */ 
    
    /*
     * At Power On Reset the configuration words set the system clock
     * to use the FRC oscillator. At this point we need to enable the
     * PLL to get the system clock running at 32MHz.
     * 
     * Clock switching on the 24FJ family with the PLL can be a bit tricky.
     * 
     * First we need to check if the configuration words enabled clock
     * switching at all, then turn off the PLL, then setup the PLL and
     * finally enable it. Sounds simple, I know. Make sure you verify this 
     * clock setup on the real hardware.
     */

    if(!OSCCONbits.CLKLOCK) /* if primary oscillator switching is unlocked */
    {
        /* Select primary oscillator as FRC */
        __builtin_write_OSCCONH(0b000);

        /* Request switch primary to new selection */
        __builtin_write_OSCCONL(OSCCON | (1 << _OSCCON_OSWEN_POSITION));

        /* wait, with timeout, for clock switch to complete */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && OSCCONbits.OSWEN;);

        CLKDIV   = 0x0000; /* set for FRC clock 8MHZ operations */

        /* Select primary oscillator as FRCPLL */
        __builtin_write_OSCCONH(0b001);
        /* Request switch primary to new selection */
        __builtin_write_OSCCONL(OSCCON | (1 << _OSCCON_OSWEN_POSITION));
        
        /* ALERT: This may be required only when the 96MHz PLL is used */
        CLKDIVbits.PLLEN = 1;

        /* wait, with timeout, for clock switch to complete */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && OSCCONbits.OSWEN;);

        /* wait, with timeout, for the PLL to lock */
        for(ClockSwitchTimeout=10000; --ClockSwitchTimeout && !OSCCONbits.LOCK;);
        
        /* at this point the system oscillator should be 32MHz */
    }
    
    ANSB   =  0x0000; /* Set for digital I/O */ 
    ANSC   =  0x0000; /* Set for digital I/O */ 
    ANSD   =  0x0000; /* Set for digital I/O */ 
    ANSE   =  0x0000; /* Set for digital I/O */ 
    ANSF   =  0x0000; /* Set for digital I/O */ 
    ANSG   =  0x0000; /* Set for digital I/O */ 
    
    CM1CON  = 0x0000; 
    CM2CON  = 0x0000; 
    CM3CON  = 0x0000; 
    
    _NSTDIS = 1;    /* disable interrupt nesting */ 
        
    TRISB   = 0xFFFF; 
    TRISC   = 0xFFFF; 
    TRISD   = 0xFFFF; 
    TRISE   = 0xFFFF; 
    TRISF   = 0xFFFF; 
    TRISG   = 0xFFFF; 
    
    /* Unlock Registers */ 
    __builtin_write_OSCCONL(OSCCON & 0xBF); 
    
    /* map all inputs */ 
    _INT1R    = RPI_NONE;       /*  External Interrupt 1    */ 
    _INT2R    = RPI_NONE;       /*  External Interrupt 2    */ 
    _INT3R    = RPI_NONE;       /*  External Interrupt 3    */ 
    _INT4R    = RPI_NONE;       /*  External Interrupt 4    */ 
    _IC1R     = RPI_NONE;       /*  Input Capture 1         */ 
    _IC2R     = RPI_NONE;       /*  Input Capture 2         */ 
    _IC3R     = RPI_NONE;       /*  Input Capture 3         */ 
    _OCFAR    = RPI_NONE;       /*  Output Compare Fault A  */ 
    _OCFBR    = RPI_NONE;       /*  Output Compare Fault B  */ 
    _SCK1R    = RPI_NONE;       /*  SPI1 Clock Input        */ 
    _SDI1R    = RPI_NONE;       /*  SPI1 Data Input         */ 
    _SS1R     = RPI_NONE;       /*  SPI1 Slave Select Input */ 
    _SCK2R    = RPI_NONE;       /*  SPI2 Clock Input        */ 
    _SDI2R    = RPI_NONE;       /*  SPI2 Data Input         */ 
    _SS2R     = RPI_NONE;       /*  SPI2 Slave Select Input */ 
    _SCK3R    = RPI_NONE;       /*  SPI3 Clock Input        */
    _SDI3R    = RPI_NONE;       /*  SPI3 Data Input         */
    _SS3R     = RPI_NONE;       /*  SPI3 Slave Select Input */
    _T2CKR    = RPI_NONE;       /*  Timer2 External Clock   */ 
    _T3CKR    = RPI_NONE;       /*  Timer3 External Clock   */ 
    _T4CKR    = RPI_NONE;       /*  Timer4 External Clock   */ 
    _T5CKR    = RPI_NONE;       /*  Timer5 External Clock   */ 
    _U1RXR    = RPI_NONE;       /*  UART1 Clear To Send     */ 
    _U1CTSR   = RPI_NONE;       /*  UART1 Receive           */ 
    _U2RXR    = RPI_NONE;       /*  UART2 Clear To Send     */ 
    _U2CTSR   = RPI_NONE;       /*  UART2 Receive           */ 
    _U3RXR    = RPI_NONE;       /*  UART3 Clear To Send     */ 
    _U3CTSR   = RPI_NONE;       /*  UART3 Receive           */ 
    _U4RXR    = RPI_NONE;       /*  UART4 Clear To Send     */ 
    _U4CTSR   = RPI_NONE;       /*  UART4 Receive           */ 
    _OCTRIG1R = RPI_NONE;       /*                          */ 
    _OCTRIG2R = RPI_NONE;       /*                          */ 
    _TCKIAR   = RPI_NONE;       /*                          */ 
    _TCKIBR   = RPI_NONE;       /*                          */ 
    _TXCKR    = RPI_NONE;       /*                          */ 
    _CLCINAR  = RPI_NONE;       /*                          */ 
    _CLCINBR  = RPI_NONE;       /*                          */ 

    /*                                       PWM out */
    /* map all outputs                        Fixed  */ 
    _RP0R   =   RPO_NONE;       /*  pin RB0          */ 
    _RP1R   =   RPO_NONE;       /*  pin RB1          */ 
    _RP2R   =   RPO_NONE;       /*  pin RD8          */ 
    _RP3R   =   RPO_NONE;       /*  pin RD10         */ 
    _RP4R   =   RPO_NONE;       /*  pin RD9          */ 
    _RP6R   =   RPO_NONE;       /*  pin RB6          */ 
    _RP7R   =   RPO_NONE;       /*  pin RB7          */ 
    _RP8R   =   RPO_NONE;       /*  pin RB8          */ 
    _RP9R   =   RPO_NONE;       /*  pin RB9          */ 
    _RP10R  =   RPO_NONE;       /*  pin RF4          */ 
    _RP11R  =   RPO_NONE;       /*  pin RD0          */ 
    _RP12R  =   RPO_NONE;       /*  pin RD11         */ 
    _RP13R  =   RPO_NONE;       /*  pin RB2          */ 
    _RP14R  =   RPO_NONE;       /*  pin RB14         */ 
    _RP16R  =   RPO_NONE;       /*  pin RF3          */ 
    _RP17R  =   RPO_NONE;       /*  pin RF5          */ 
    _RP18R  =   RPO_NONE;       /*  pin RB5   OCM3A  */ 
    _RP19R  =   RPO_NONE;       /*  pin RG8   OCM2A  */ 
    _RP20R  =   RPO_NONE;       /*  pin RD5          */ 
    _RP21R  =   RPO_NONE;       /*  pin RG6   OCM1A  */ 
    _RP22R  =   RPO_NONE;       /*  pin RD3          */ 
    _RP23R  =   RPO_NONE;       /*  pin RD2          */ 
    _RP24R  =   RPO_NONE;       /*  pin RD1          */ 
    _RP25R  =   RPO_NONE;       /*  pin RD4          */ 
    _RP26R  =   RPO_NONE;       /*  pin RG7   OCM1B  */ 
    _RP27R  =   RPO_NONE;       /*  pin RG9   OCM2B  */ 
    _RP28R  =   RPO_NONE;       /*  pin RB4   OCM3B  */ 
    _RP29R  =   RPO_NONE;       /*  pin RB15         */ 
    _RP30R  =   RPO_NONE;       /*  pin RF2          */ 
    
    /* Lock Registers */ 
    __builtin_write_OSCCONL(OSCCON | 0x40); 
}
    
/*
 * WARNING: Not a portable function.
 *          Maximum 16MHz instruction cycle clock.
 *          Minimum  8Khz instruction cycle clock.
 */
void delay_ms( unsigned long delay )
{
    do
    {
        asm("repeat  %0\n clrwdt\n":: "r" (FCYC/1000L-7L));
    } while(delay--);
}

#define GPIO_OUT 0
#define GPIO_IN  1

#define LED_ON   1
#define LED_OFF  0

#define YELLOW_LED_DIR   TRISBbits.TRISB4
#define YELLOW_LED       LATBbits.LATB4


int main() 
{
    PIC_init();
    
    YELLOW_LED_DIR = GPIO_OUT;

    /* loop forever, main never exits */
    for(;;)
    {
        if (YELLOW_LED == LED_OFF)
        {
            YELLOW_LED = LED_ON;
        }
        else
        {
            YELLOW_LED = LED_OFF;
        }
        delay_ms(500);    
    }
    return 0; 
}   

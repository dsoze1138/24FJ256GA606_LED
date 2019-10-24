/*
 * File:   pwm.c
 * 
 * Target: PIC24FJ256GA606
 * 
 * Description:
 * 
 *  Validation code for GB206 to GA606
 *  port of the PWM modulators for the
 *  S700 scanner project.
 *
 */

#include <xc.h>
#include "init.h"
#include "pwm.h"
extern void delay_ms( unsigned long delay );

/*
 *
 */
void PWM_Init(void) 
{   
    /*
     * Setup PWM using CCP1 for OCM1B on pin RG6    SOC_SOC_RED
     * Dual Edge Compare (16-bit buffered) PWM Mode
     */
    _CCP1IE   = 0;  /* Disable interrupts */
    CCP1CON1L = 0;  /* Stop CCP1 */
    CCP1CON1H = 0;
    CCP1CON2L = 0;
    CCP1CON2H = 0;
    CCP1CON3L = 0;
    CCP1CON3H = 0;
    CCP1TMRL  = 0;  /* Initialize timer prior to enable module. */
    CCP1TMRH  = 0;
    CCP1PRL   = 0;
    CCP1PRH   = 0;
    CCP1RA    = 0;
    CCP1RB    = 0;
    CCP1BUFL  = 0;
    CCP1BUFH  = 0;
    CCP1CON1Lbits.TMRSYNC = 0;          /* time base clock is not synchronized */
    CCP1CON1Lbits.CLKSEL  = 0b000;      /* select FCYC as clock source */
    CCP1CON1Lbits.TMRPS   = 0b00;       /* select 1:1 prescaler */
    CCP1CON1Lbits.T32     = 0;          /* select 16-bit time base */
    CCP1CON1Lbits.CCSEL   = 0;          /* select Output mode */
    CCP1CON1Lbits.MOD     = 0b0101;     /* select Dual Edge Compare mode, buffered (PWM) */
    CCP1CON1Hbits.RTRGEN  = 1;          /* Time base can be retriggered */
    CCP1CON1Hbits.ONESHOT = 0;          /* One-Shot Trigger mode is disabled */
    CCP1CON1Hbits.SYNC    = 0b00000;    /* Select Sync/Trigger source (Self-sync) */
    CCP1CON2Hbits.OENSYNC = 0;          /* Update by output enable bits occurs on the next Time Base Reset or rollover */
    CCP1CON3Hbits.OUTM    = 0b000;      /* Steerable Single Output mode */
    CCP1CON3Hbits.POLACE  = 1;          /* OCM1A Output pin polarity is active-low */
    
    /* WARNING: Period and duty cycle must be set AFTER the CCPxCON1Lbits.MOD is selected */
    CCP1PRL = PWM_PERIOD - 1;           /* Configure timebase period */
    CCP1RA  = 0;                        /* Set the rising edge compare value */
    CCP1RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP1CON1Lbits.CCPON   = 1;          /* Turn on CCP1 module */
    CCP1CON2Hbits.OCAEN   = 1;          /* OCM1A pin is controlled by the CCP1 module and produces a PWM output */
    
    /*
     * Setup PWM using CCP2 for OCM2A on pin RG8    SOC_GREEN
     * Dual Edge Compare (16-bit buffered) PWM Mode
     */
    _CCP2IE   = 0;  /* Disable interrupts */
    CCP2CON1L = 0;  /* Stop CCP2 */
    CCP2CON1H = 0;
    CCP2CON2L = 0;
    CCP2CON2H = 0;
    CCP2CON3L = 0;
    CCP2CON3H = 0;
    CCP2TMRL  = 0;  /* Initialize timer prior to enable module. */
    CCP2TMRH  = 0;
    CCP2PRL   = 0;
    CCP2PRH   = 0;
    CCP2RA    = 0;
    CCP2RB    = 0;
    CCP2BUFL  = 0;
    CCP2BUFH  = 0;
    CCP2CON1Lbits.TMRSYNC = 0;          /* time base clock is not synchronized */
    CCP2CON1Lbits.CLKSEL  = 0b000;      /* select FCYC as clock source */
    CCP2CON1Lbits.TMRPS   = 0b00;       /* select 1:1 prescaler */
    CCP2CON1Lbits.T32     = 0;          /* select 16-bit time base */
    CCP2CON1Lbits.CCSEL   = 0;          /* select Output mode */
    CCP2CON1Lbits.MOD     = 0b0101;     /* select Dual Edge Compare mode, buffered (PWM) */
    CCP2CON1Hbits.RTRGEN  = 1;          /* Time base can be retriggered */
    CCP2CON1Hbits.ONESHOT = 0;          /* One-Shot Trigger mode is disabled */
    CCP2CON1Hbits.SYNC    = 0b00000;    /* Select Sync/Trigger source (Self-sync) */
    CCP2CON2Hbits.OENSYNC = 0;          /* Update by output enable bits occurs on the next Time Base Reset or rollover */
    CCP2CON3Hbits.OUTM    = 0b000;      /* Steerable Single Output mode */
    CCP2CON3Hbits.POLACE  = 1;          /* OCM2A Output pin polarity is active-low */
    
    /* WARNING: Period and duty cycle must be set AFTER the CCPxCON1Lbits.MOD is selected */
    CCP2PRL = PWM_PERIOD - 1;           /* Configure timebase period */
    CCP2RA  = 0;                        /* Set the rising edge compare value */
    CCP2RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP2CON1Lbits.CCPON   = 1;          /* Turn on CCP2 module */
    CCP2CON2Hbits.OCAEN   = 1;          /* OCM2A pin is controlled by the CCP2 module and produces a PWM output */
    
    /*
     * Setup PWM using CCP3 for OCM3A on pin RB5    SOC_BLUE
     * Dual Edge Compare (16-bit buffered) PWM Mode
     */
    _CCP3IE   = 0;  /* Disable interrupts */
    CCP3CON1L = 0;  /* Stop CCP3 */
    CCP3CON1H = 0;
    CCP3CON2L = 0;
    CCP3CON2H = 0;
    CCP3CON3L = 0;
    CCP3CON3H = 0;
    CCP3TMRL  = 0;  /* Initialize timer prior to enable module. */
    CCP3TMRH  = 0;
    CCP3PRL   = 0;
    CCP3PRH   = 0;
    CCP3RA    = 0;
    CCP3RB    = 0;
    CCP3BUFL  = 0;
    CCP3BUFH  = 0;
    CCP3CON1Lbits.TMRSYNC = 0;          /* time base clock is not synchronized */
    CCP3CON1Lbits.CLKSEL  = 0b000;      /* select FCYC as clock source */
    CCP3CON1Lbits.TMRPS   = 0b00;       /* select 1:1 prescaler */
    CCP3CON1Lbits.T32     = 0;          /* select 16-bit time base */
    CCP3CON1Lbits.CCSEL   = 0;          /* select Output mode */
    CCP3CON1Lbits.MOD     = 0b0101;     /* select Dual Edge Compare mode, buffered (PWM) */
    CCP3CON1Hbits.RTRGEN  = 1;          /* Time base can be retriggered */
    CCP3CON1Hbits.ONESHOT = 0;          /* One-Shot Trigger mode is disabled */
    CCP3CON1Hbits.SYNC    = 0b00000;    /* Select Sync/Trigger source (Self-sync) */
    CCP3CON2Hbits.OENSYNC = 0;          /* Update by output enable bits occurs on the next Time Base Reset or rollover */
    CCP3CON3Hbits.OUTM    = 0b000;      /* Steerable Single Output mode */
    CCP3CON3Hbits.POLACE  = 1;          /* OCM3A Output pin polarity is active-low */
    
    /* WARNING: Period and duty cycle must be set AFTER the CCPxCON1Lbits.MOD is selected */
    CCP3PRL = PWM_PERIOD - 1;           /* Configure timebase period */
    CCP3RA  = 0;                        /* Set the rising edge compare value */
    CCP3RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP3CON1Lbits.CCPON   = 1;          /* Turn on CCP3 module */
    CCP3CON2Hbits.OCAEN   = 1;          /* OCM3A pin is controlled by the CCP2 module and produces a PWM output */
    
    /*
     * Setup PWM using CCP4 for OCM4 on pin RB8/RP8 SCAN_RED
     * Dual Edge Compare (16-bit buffered) PWM Mode
     */
    IEC5bits.CCP4IE = 0;  /* Disable interrupts */
    CCP4CON1L = 0;  /* Stop CCP4 */
    CCP4CON1H = 0;
    CCP4CON2L = 0;
    CCP4CON2H = 0;
    CCP4CON3L = 0;
    CCP4CON3H = 0;
    CCP4TMRL  = 0;  /* Initialize timer prior to enable module. */
    CCP4TMRH  = 0;
    CCP4PRL   = 0;
    CCP4PRH   = 0;
    CCP4RA    = 0;
    CCP4RB    = 0;
    CCP4BUFL  = 0;
    CCP4BUFH  = 0;
    CCP4CON1Lbits.TMRSYNC = 0;          /* time base clock is not synchronized */
    CCP4CON1Lbits.CLKSEL  = 0b000;      /* select FCYC as clock source */
    CCP4CON1Lbits.TMRPS   = 0b00;       /* select 1:1 prescaler */
    CCP4CON1Lbits.T32     = 0;          /* select 16-bit time base */
    CCP4CON1Lbits.CCSEL   = 0;          /* select Output mode */
    CCP4CON1Lbits.MOD     = 0b0101;     /* select Dual Edge Compare mode, buffered (PWM) */
    CCP4CON1Hbits.RTRGEN  = 1;          /* Time base can be retriggered */
    CCP4CON1Hbits.ONESHOT = 0;          /* One-Shot Trigger mode is disabled */
    CCP4CON1Hbits.SYNC    = 0b00000;    /* Select Sync/Trigger source (Self-sync) */
    CCP4CON2Hbits.OENSYNC = 0;          /* Update by output enable bits occurs on the next Time Base Reset or rollover */

    CCP4CON3Hbits.POLACE  = 1;          /* OCM4 Output pin polarity is active-low */
    
    /* WARNING: Period and duty cycle must be set AFTER the CCPxCON1Lbits.MOD is selected */
    CCP4PRL = PWM_PERIOD - 1;           /* Configure timebase period */
    CCP4RA  = 0;                        /* Set the rising edge compare value */
    CCP4RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP4CON1Lbits.CCPON   = 1;          /* Turn on CCP4 module */
    CCP4CON2Hbits.OCAEN   = 1;          /* OCM4 pin is controlled by the CCP4 module and produces a PWM output */
    
    /*
     * Setup PWM using CCP5 for OCM5 on pin RB7/RP7 SCAN_GREEN
     * Dual Edge Compare (16-bit buffered) PWM Mode
     */
    _CCP5IE   = 0;  /* Disable interrupts */
    CCP5CON1L = 0;  /* Stop CCP5 */
    CCP5CON1H = 0;
    CCP5CON2L = 0;
    CCP5CON2H = 0;
    CCP5CON3L = 0;
    CCP5CON3H = 0;
    CCP5TMRL  = 0;  /* Initialize timer prior to enable module. */
    CCP5TMRH  = 0;
    CCP5PRL   = 0;
    CCP5PRH   = 0;
    CCP5RA    = 0;
    CCP5RB    = 0;
    CCP5BUFL  = 0;
    CCP5BUFH  = 0;
    CCP5CON1Lbits.TMRSYNC = 0;          /* time base clock is not synchronized */
    CCP5CON1Lbits.CLKSEL  = 0b000;      /* select FCYC as clock source */
    CCP5CON1Lbits.TMRPS   = 0b00;       /* select 1:1 prescaler */
    CCP5CON1Lbits.T32     = 0;          /* select 16-bit time base */
    CCP5CON1Lbits.CCSEL   = 0;          /* select Output mode */
    CCP5CON1Lbits.MOD     = 0b0101;     /* select Dual Edge Compare mode, buffered (PWM) */
    CCP5CON1Hbits.RTRGEN  = 1;          /* Time base can be retriggered */
    CCP5CON1Hbits.ONESHOT = 0;          /* One-Shot Trigger mode is disabled */
    CCP5CON1Hbits.SYNC    = 0b00000;    /* Select Sync/Trigger source (Self-sync) */
    CCP5CON2Hbits.OENSYNC = 0;          /* Update by output enable bits occurs on the next Time Base Reset or rollover */

    CCP5CON3Hbits.POLACE  = 1;          /* OCM5 Output pin polarity is active-low */
    
    /* WARNING: Period and duty cycle must be set AFTER the CCPxCON1Lbits.MOD is selected */
    CCP5PRL = PWM_PERIOD - 1;           /* Configure timebase period */
    CCP5RA  = 0;                        /* Set the rising edge compare value */
    CCP5RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP5CON1Lbits.CCPON   = 1;          /* Turn on CCP5 module */
    CCP5CON2Hbits.OCAEN   = 1;          /* OCM1B pin is controlled by the CCP5 module and produces a PWM output */
    
    /*
     * Setup PWM using CCP6 for OCM6 on pin RD0/RP11 SCAN_BLUE
     * Dual Edge Compare (16-bit buffered) PWM Mode
     */
    _CCP6IE   = 0;  /* Disable interrupts */
    CCP6CON1L = 0;  /* Stop CCP6 */
    CCP6CON1H = 0;
    CCP6CON2L = 0;
    CCP6CON2H = 0;
    CCP6CON3L = 0;
    CCP6CON3H = 0;
    CCP6TMRL  = 0;  /* Initialize timer prior to enable module. */
    CCP6TMRH  = 0;
    CCP6PRL   = 0;
    CCP6PRH   = 0;
    CCP6RA    = 0;
    CCP6RB    = 0;
    CCP6BUFL  = 0;
    CCP6BUFH  = 0;
    CCP6CON1Lbits.TMRSYNC = 0;          /* time base clock is not synchronized */
    CCP6CON1Lbits.CLKSEL  = 0b000;      /* select FCYC as clock source */
    CCP6CON1Lbits.TMRPS   = 0b00;       /* select 1:1 prescaler */
    CCP6CON1Lbits.T32     = 0;          /* select 16-bit time base */
    CCP6CON1Lbits.CCSEL   = 0;          /* select Output mode */
    CCP6CON1Lbits.MOD     = 0b0101;     /* select Dual Edge Compare mode, buffered (PWM) */
    CCP6CON1Hbits.RTRGEN  = 1;          /* Time base can be retriggered */
    CCP6CON1Hbits.ONESHOT = 0;          /* One-Shot Trigger mode is disabled */
    CCP6CON1Hbits.SYNC    = 0b00000;    /* Select Sync/Trigger source (Self-sync) */
    CCP6CON2Hbits.OENSYNC = 0;          /* Update by output enable bits occurs on the next Time Base Reset or rollover */

    CCP6CON3Hbits.POLACE  = 1;          /* OCM6 Output pin polarity is active-low */
    
    /* WARNING: Period and duty cycle must be set AFTER the CCPxCON1Lbits.MOD is selected */
    CCP6PRL = PWM_PERIOD - 1;           /* Configure timebase period */
    CCP6RA  = 0;                        /* Set the rising edge compare value */
    CCP6RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP6CON1Lbits.CCPON   = 1;          /* Turn on CCP6 module */
    CCP6CON2Hbits.OCAEN   = 1;          /* OCM1B pin is controlled by the CCP6 module and produces a PWM output */
    
    /*
     * Setup PWM using CCP7 for OCM7 on pin RB6/RP6 CONNECT_BLUE
     * Dual Edge Compare (16-bit buffered) PWM Mode
     */
    _CCP7IE   = 0;  /* Disable interrupts */
    CCP7CON1L = 0;  /* Stop CCP7 */
    CCP7CON1H = 0;
    CCP7CON2L = 0;
    CCP7CON2H = 0;
    CCP7CON3L = 0;
    CCP7CON3H = 0;
    CCP7TMRL  = 0;  /* Initialize timer prior to enable module. */
    CCP7TMRH  = 0;
    CCP7PRL   = 0;
    CCP7PRH   = 0;
    CCP7RA    = 0;
    CCP7RB    = 0;
    CCP7BUFL  = 0;
    CCP7BUFH  = 0;
    CCP7CON1Lbits.TMRSYNC = 0;          /* time base clock is not synchronized */
    CCP7CON1Lbits.CLKSEL  = 0b000;      /* select FCYC as clock source */
    CCP7CON1Lbits.TMRPS   = 0b00;       /* select 1:1 prescaler */
    CCP7CON1Lbits.T32     = 0;          /* select 16-bit time base */
    CCP7CON1Lbits.CCSEL   = 0;          /* select Output mode */
    CCP7CON1Lbits.MOD     = 0b0101;     /* select Dual Edge Compare mode, buffered (PWM) */
    CCP7CON1Hbits.RTRGEN  = 1;          /* Time base can be retriggered */
    CCP7CON1Hbits.ONESHOT = 0;          /* One-Shot Trigger mode is disabled */
    CCP7CON1Hbits.SYNC    = 0b00000;    /* Select Sync/Trigger source (Self-sync) */
    CCP7CON2Hbits.OENSYNC = 0;          /* Update by output enable bits occurs on the next Time Base Reset or rollover */

    CCP7CON3Hbits.POLACE  = 1;          /* OCM7 Output pin polarity is active-low */
    
    /* WARNING: Period and duty cycle must be set AFTER the CCPxCON1Lbits.MOD is selected */
    CCP7PRL = PWM_PERIOD - 1;           /* Configure timebase period */
    CCP7RA  = 0;                        /* Set the rising edge compare value */
    CCP7RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP7CON1Lbits.CCPON   = 1;          /* Turn on CCP7 module */
    CCP7CON2Hbits.OCAEN   = 1;          /* OCM7A pin is controlled by the CCP7 module and produces a PWM output */
    
    /*
     * Setup PWM using OC1 to be mapped on pin RD8/RP2 MOTOR_OUT
     */
    _OC1IE  = 0;    /* Disable interrupts */
    OC1CON1 = 0;    /* Stop OC1 */
    OC1CON2 = 0;
    OC1TMR  = 0;
    OC1RS   = PWM_PERIOD - 1;           /* Configure timebase period */
    OC1R    = PWM_DEFAULT_DUTY_CYCLE;
    OC1CON2bits.SYNCSEL = 0x1F;         /* Select Sync/Trigger source (Self-sync) */
    OC1CON2bits.OCINV   = 0;            /* OC1 Output pin polarity is active-high */
    OC1CON1bits.OCTSEL  = 0b111;        /* Select FCYC as clock source */
    OC1CON1bits.OCM     = 0b110;        /* This selects and starts the Edge Aligned PWM mode*/

    /*
     * Setup PWM using OC2 to be mapped on pin RB9/RP9
     */
    _OC2IE  = 0;    /* Disable interrupts */
    OC2CON1 = 0;    /* Stop OC2 */
    OC2CON2 = 0;
    OC2TMR  = 0;
    OC2RS   = PWM_PERIOD - 1;           /* Configure timebase period */
    OC2R    = PWM_DEFAULT_DUTY_CYCLE;
    OC2CON2bits.SYNCSEL = 0x1F;         /* Select Sync/Trigger source (Self-sync) */
    OC2CON2bits.OCINV   = 0;            /* OC2 Output pin polarity is active-high */
    OC2CON1bits.OCTSEL  = 0b111;        /* Select FCYC as clock source */
    OC2CON1bits.OCM     = 0b110;        /* This selects and starts the Edge Aligned PWM mode*/

    /*
     * Setup Output toggle using OC3 to be mapped on pin NOT USED
     */
    _OC3IE  = 0;    /* Disable interrupts */
    OC3CON1 = 0;    /* Stop OC3 */
    OC3CON2 = 0;
    OC3TMR  = 0;
    OC3RS   = SOUND_DEFAULT_PERIOD - 1;           /* Configure timebase period */
    OC3R    = SOUND_DEFAULT_DUTY_CYCLE;
    OC3CON2bits.SYNCSEL = 0x1F;         /* Select Sync/Trigger source (Self-sync) */
    OC3CON2bits.OCINV   = 1;            /* OC3 Output pin polarity is active-high */
    OC3CON1bits.OCTSEL  = 0b111;        /* Select FCYC as clock source */
    OC3CON1bits.OCM     = 0b110;        /* This selects and starts the Edge Aligned PWM mode*/
}
/*
 * 
 */
void PWM_Test(void)
{
    delay_ms(10);
    CCP1RB  = PWM_TEST_DUTY_CYCLE;
    delay_ms(10);
    CCP2RB  = PWM_TEST_DUTY_CYCLE;
    CCP3RB  = PWM_TEST_DUTY_CYCLE;
    CCP4RB  = PWM_TEST_DUTY_CYCLE;
    CCP5RB  = PWM_TEST_DUTY_CYCLE;
    CCP6RB  = PWM_TEST_DUTY_CYCLE;
    CCP7RB  = PWM_TEST_DUTY_CYCLE;
    OC1R    = PWM_TEST_DUTY_CYCLE;
    OC2RS   = SOUND_DEFAULT_PERIOD;
    OC2R    = SOUND_DEFAULT_DUTY_CYCLE;

    delay_ms(10);
    /* Test sound mode start frequency 1 */
    OC2RS   = SOUND_TEST1_PERIOD - 1;
    OC2R    = SOUND_TEST1_DUTY_CYCLE;
    delay_ms(10);
    /* Test sound mode start frequency 2 */
    OC2RS   = SOUND_TEST2_PERIOD - 1;
    OC2R    = SOUND_TEST2_DUTY_CYCLE;
    delay_ms(10);

    CCP1RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP2RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP3RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP4RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP5RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP6RB  = PWM_DEFAULT_DUTY_CYCLE;
    CCP7RB  = PWM_DEFAULT_DUTY_CYCLE;
    OC1R    = PWM_DEFAULT_DUTY_CYCLE;
    OC2R    = SOUND_DEFAULT_DUTY_CYCLE;
}

void __attribute__((interrupt, auto_psv)) 
_CompInterrupt(
    void
    )
{
    _CMIF = 0;
}

/*
 * File:   pwm.h
 * Author: 
 * Target: DM240001-3, Explorer 16/32, PIC24FJ1024GA610
 *
 */
#ifndef PWM_H
#define PWM_H

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/* Select 4KHz as PWM frequency. Using 1:1 prescaler */
#define PWM_PRESCALE_COUNTS (1L)
#define PWM_CYCLES_PER_SECOND (4000L)
#define PWM_PERIOD (FCYC/(PWM_PRESCALE_COUNTS*PWM_CYCLES_PER_SECOND))
#define PWM_DEFAULT_DUTY_CYCLE (0)
#define PWM_TEST_DUTY_CYCLE (PWM_PERIOD/20L)

#define SOUND_DEFAULT_PERIOD (FCYC/3000)
#define SOUND_DEFAULT_DUTY_CYCLE (0)
#define SOUND_TEST1_PERIOD (FCYC/2000)
#define SOUND_TEST1_DUTY_CYCLE (SOUND_TEST1_PERIOD>>1)
#define SOUND_TEST2_PERIOD (FCYC/1000)
#define SOUND_TEST2_DUTY_CYCLE (SOUND_TEST2_PERIOD>>1)

void PWM_Init(void);
void PWM_Test(void);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif

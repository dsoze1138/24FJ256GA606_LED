/*
 * File:   init.h
 * Author: 
 * Target: PIC24FJ256GA606
 *
 */
#ifndef INIT_H
#define INIT_H
#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif

/* 
 * Define the target system clock frequency.
 * 
 * The initialization MUST set the system clock to support these definitions.
 * 
 */
#define FSYS (32000000UL)
#define FCYC (FSYS/2UL)

#define MAJOR_REV (1)
#define MINOR_REV (1)

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif

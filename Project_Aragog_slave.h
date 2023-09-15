/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Header file for the setup and communication used by
 * the SLAVE PIC.
*/

#ifndef PROJECT_ARAGOG_SLAVE_H
#define	PROJECT_ARAGOG_SLAVE_H

#ifdef	__cplusplus
extern "C" {
#endif

    void slaveSetup(void);
    void __attribute__((__interrupt__, __auto_psv__)) _SPI1Interrupt(void);
    
#endif

#ifdef	__cplusplus
}
#endif

//#endif	/* PROJECT_ARAGOG_SLAVE_H */


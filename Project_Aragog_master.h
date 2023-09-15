/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Header file for the setup and communication used by
 * the MASTER PIC.
*/

#ifndef PROJECT_ARAGOG_MASTER_SETUP_H
#define	PROJECT_ARAGOG_MASTER_SETUP_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <stdint.h>
    
void masterSetup(void);
void readByteFromSlave(uint8_t whatToRead, uint8_t *pReadData);


#ifdef	__cplusplus
}
#endif

#endif	/* PROJECT_ARAGOG_MASTER_SETUP_H */


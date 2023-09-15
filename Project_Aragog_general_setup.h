/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Header file for the setup and communication used by
 * both the MASTER and SLAVE PICs. Sets up Clock frequency and digital pins,
 * along with setting up servos and including their movement functions.
*/

#ifndef PROJECT_ARAGOG_GENERAL_SETUP_H
#define	PROJECT_ARAGOG_GENERAL_SETUP_H

#ifdef	__cplusplus
extern "C" {
#endif
    
void basicSetup(void);
void delay_ms(int ms);
void servoSetup(void);
void setServo(int servo, int val);

#ifdef	__cplusplus
}
#endif

#endif	/* PROJECT_ARAGOG_GENERAL_SETUP_H */


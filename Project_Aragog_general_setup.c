/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Function file for both PICs. Contains a setup
 * function for Internal clock settings and digital pin functions.
 * Also sets up and implements servos for movement and usage.
*/

#include <p24FJ64GA002.h>
#include "xc.h"
#include "Project_Aragog_general_setup.h"

/*
Function: basicSetup
---------------------------------------
Simply sets the PIC24 internal clock to 16MHz, and assigns all pins to digital.
 */
void basicSetup(void){
    ////PIC24 basic setup
    CLKDIVbits.RCDIV = 0;   //Set RCDIV as 1:1 (default 2:1) 32MHz or FCY/2=16M
    AD1PCFG = 0xffff;       //set all pins digital
}

/*
Function: delay_ms
---------------------------------------
Simply delays any and all processing for the desired number of mili-seconds
ms: the number of mili-seconds to wait
Used for movement and timing purposes purposes.
 */
void delay_ms(int ms){
    while(ms-- > 0){
        asm("repeat #15998");
        asm("nop");
    }
}

/*
 Function: servoSetup
 -------------------------------
 Sets pins 26,25,24,23,22 as Output Compare 1,2,3,4,5 respectively, which send
 * out a defined Pulse Width of 20 miliseconds. Timer 3 is also set up to be
 * used for the OC, with a Pre-scalar of 1:8 and set its Period to the 20 ms
 * value (39999). 
 */
void servoSetup(void){                      //Servo basic setup
    
    //Pin assignment begin
    
    TRISBbits.TRISB15 = 0;                  //Pin 26 (RP15) is output
    TRISBbits.TRISB14 = 0;                  //Pin 25 (RP14) is output
    TRISBbits.TRISB13 = 0;                  //Pin 24 (RP13) is output
    TRISBbits.TRISB12 = 0;                  //Pin 23 (RP12) is output
    TRISBbits.TRISB11 = 0;                  //Pin 22 (RP11) is output
    
    //Setup the Peripheral Pin Select
    __builtin_write_OSCCONL(OSCCON & 0xBF); //Unlocks the PPS for OC selection
    //Setup Output Compare
    RPOR7bits.RP15R = 18;                   //Set RP15 as OC-1
    RPOR7bits.RP14R = 19;                   //Set RP14 as OC-2
    RPOR6bits.RP13R = 20;                   //Set RP13 as OC-3
    RPOR6bits.RP12R = 21;                   //Set RP12 as OC-4
    RPOR5bits.RP11R = 22;                   //Set RP11 as OC-5
    __builtin_write_OSCCONL(OSCCON & 0x40); //Re-locks the PPS
    
    //Pin assignment end
    
    //Setup Timer 3 for Output Compare
    
    T3CON = 0;                  //Stop timer and resets control register
    T3CONbits.TCKPS = 0b01;     //Pre-Scalar. 0b01 = 1:8 pre-scale (PG. 14-6)
    TMR3 = 0;                   //Clear contents of timer register
    PR3 = 40000 - 1;            //Period register. Period is = PRx_value + 1
                                //40000 cycles at 1:8 makes a 20 mil sec period
    
    //Setup the Output Compare which uses TMR 3
    
    //Output Compare 1 setup
    OC1CON = 0;                 //Turn off OC-1 for now
    OC1R = 3000;                //Servo start position at 90 degrees
    OC1RS = 3000;               //The next value to be written to the OC1R
    OC1CONbits.OCTSEL = 0b01;   //Use Timer 3 for compare source
    OC1CONbits.OCM = 0b110;     //OC-1 mode is PWM without faults

    //Output Compare 2 setup
    OC2CON = 0;                 //Turn off OC-2 for now
    OC2R = 3000;                //Servo start position at 90 degrees
    OC2RS = 3000;               //The next value to be written to the OC1R
    OC2CONbits.OCTSEL = 0b01;   //Use Timer 3 for compare source
    OC2CONbits.OCM = 0b110;     //OC-2 mode is PWM without faults

    //Output Compare 3 setup
    OC3CON = 0;                 //Turn off OC-3 for now
    OC3R = 3000;                //Servo start position at 90 degrees
    OC3RS = 3000;               //The next value to be written to the OC1R
    OC3CONbits.OCTSEL = 0b01;   //Use Timer 3 for compare source
    OC3CONbits.OCM = 0b110;     //OC-3 mode is PWM without faults

    //Output Compare 4 setup
    OC4CON = 0;                 //turn off OC-4 for now
    OC4R = 3000;                //Servo start position at 90 degrees
    OC4RS = 3000;               //The next value to be written to the OC1R
    OC4CONbits.OCTSEL = 0b01;   //Use Timer 3 for compare source
    OC4CONbits.OCM = 0b110;     //OC-4 mode is PWM without faults

    //Output Compare 5 setup
    OC5CON = 0;                 //Turn off OC-5 for now
    OC5R = 3000;                //Servo start position at 90 degrees
    OC5RS = 3000;               //The next value to be written to the OC1R
    OC5CONbits.OCTSEL = 0b01;   //Use Timer 3 for compare source
    OC5CONbits.OCM = 0b110;     //OC-5 mode is PWM without faults
    //Timer 3 and Output Compare 1-5 Setup Complete
    T3CONbits.TON = 1;          //Starts the timer
}

/*
Function: setServo
---------------------------------------
Updates the OC1RS register with Val. Val is an integer from 0 to PR3 (39999).
    40,000 counts is 20 ms, so there is a 20 ms pulse width. Val can be
    computed as Val = (desired_num_ms_HIGH)*2000. This is because 40,000 counts
    in 20 ms, so 2000 counts/ms.
 * servo: an integer value from 1 to 5, selecting OC 1-5 respectively
 * val: cycles of the PWM duty cycle which are set as high
 */
void setServo(int servo, int val){
    if (servo == 1) OC1RS = val;
    if (servo == 2) OC2RS = val;
    if (servo == 3) OC3RS = val;
    if (servo == 4) OC4RS = val;
    if (servo == 5) OC5RS = val;
}

/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Main file assembling all of the other files
 * together so that a complete working mech can be presented. There is much that
 * could be improved, and it should be noted that this is simply a prototype at
 * this stage in its development. 
 * MASTER should be defined if it is meant to control the LEFT legs, otherwise
 * if it is not defined, the SLAVE controls the RIGHT legs.
 * MASTER waits for the press of a button, then when it is pressed, sends a code
 * to the slave telling it to run its movement script. Meanwhile, master runs
 * its own movement script.
*/


#define MASTER //Uncommenting makes this PIC Program for the Master, else slave

#include "Project_Aragog_general_setup.h"
#ifdef MASTER
#include "Project_Aragog_master.h"
#include "Project_Aragog_defines_master.h"
#else
#include "Project_Aragog_slave.h"
#include "Project_Aragog_defines_slave.h"
#endif

///////////////////////////////START BOILERPLATE///////////////////////////////
#include <p24FJ64GA002.h>
#include "xc.h"
// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)
// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
                                       // Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL      // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))
///////////////////////////////END BOILERPLATE/////////////////////////////////

//Function: delay_ms
//---------------------------------------
//Simply delays any and all processing for the desired number of mili-seconds
//ms: the number of mili-seconds to wait
//Used for debugging purposes.
// */


///*
//Function: basicSetup
//---------------------------------------
//Simply sets the PIC24 internal clock to 16MHz, and assigns all pins to digital.
// */

/*
Function: slaveSetup
---------------------------------------
 * Configures pins 6,7,11,14 as Data Out,Data In, Clock In, Slave Select In,
 * respectively. Then configures the Slave's SPI settings.
 */



// Function: servoSetup
// -------------------------------
// Sets pins 26,25,24,23,22 as Output Compare 1,2,3,4,5 respectively, which send
// * out a defined Pulse Width of 20 miliseconds. Timer 3 is also set up to be
// * used for the OC, with a Pre-scalar of 1:8 and set its Period to the 20 ms
// * value (39999). 
// */

//Function: setServo
//---------------------------------------
//Updates the OC1RS register with Val. Val is an integer from 0 to PR3 (39999).
//    40,000 counts is 20 ms, so there is a 20 ms pulse width. Val can be
//    computed as Val = (desired_num_ms_HIGH)*2000. This is because 40,000 counts
//    in 20 ms, so 2000 counts/ms.
// * servo: an integer value from 1 to 5, selecting OC 1-5 respectively
// * val: cycles of the PWM duty cycle which are set as high
// */

int main(void) {
basicSetup();    //both PICS need this setup
servoSetup();
    
#ifdef MASTER     //If master is defined, set it up, otherwise it's a Slave
    masterSetup();
#else 
    slaveSetup();
#endif
    while (1) {     //Forever Loop
#ifdef MASTER
        uint8_t whatToRead = 0x57;
        uint8_t readData;

        while (PORTBbits.RB6 == 0);       //Until the button gets pressed
        
        readByteFromSlave(whatToRead, &readData);

        if (readData == 0x75) {
            delay_ms(300);
            setServo(BackLeftHip, 2000);            // Back Left Hip set
            setServo(BackLeftKnee, KneeOrigin);     // Back Left Knee set
            setServo(LeftFeet, FeetOrigin);         // Back Left Foot set
            setServo(FrontLeftKnee, KneeOrigin);    // Front Left Knee set
            setServo(FrontLeftHip, 4000);           // Front Left Hip set

            delay_ms(300);
            delay_ms(600);
            setServo(BackLeftKnee, 2000);           // Raise Back Left Knee
            delay_ms(300);
            setServo(BackLeftHip, 1200);            // Back Left hip rotate forward
            delay_ms(300);
            setServo(BackLeftKnee, KneeOrigin);     // Back Left Knee set
            delay_ms(900);

            delay_ms(300);
            setServo(FrontLeftKnee, 4000);          // Raise Front Left Knee 
            delay_ms(300);
            setServo(FrontLeftHip, 3000);           // Front Left Hip rotate forward
            delay_ms(300);
            setServo(FrontLeftKnee, KneeOrigin);    // Front Left Knee set
            delay_ms(900);

            delay_ms(300);
            setServo(BackLeftKnee, 4000);           // Lower Back Left Knee
            delay_ms(300);
            setServo(BackLeftHip, BackLeftHipOrigin);// Back Left hip rotate back to set
            delay_ms(300);
            setServo(BackLeftKnee, KneeOrigin);     // Back Left Knee set
            delay_ms(900);


            delay_ms(300);
            setServo(FrontLeftKnee, 2000);          // Lower Front Left Knee
            delay_ms(300);
            setServo(FrontLeftHip, FrontLeftHipOrigin);// Front Left hip rotate back to set
            delay_ms(300);
            setServo(FrontLeftKnee, KneeOrigin);    // Raise Front Left Knee
        } 
        
        else if (readData == 0x8C) {
            // Debugging code, used if connection from master to slave becomes eroded; Standby

            delay_ms(300);
            setServo(BackLeftHip, 2000);              // Back Left Hip set
            setServo(BackLeftKnee, KneeOrigin);              // Back Left Knee set
            setServo(LeftFeet, FeetOrigin);              // Back Left Foot set
            setServo(FrontLeftKnee, KneeOrigin);              // Front Left Knee set
            setServo(FrontLeftHip, 4000);              // Front Left Hip set
        }
        
        else { // Debugging code, no connection between master and slave
            setServo(1, 2000);
            delay_ms(300);
            setServo(1, 1800);
        }
#endif
    }
}

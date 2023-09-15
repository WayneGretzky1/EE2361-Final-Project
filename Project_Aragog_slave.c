/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Function file for the SLAVE PIC. Contains a setup
 * function for SPI, and also contains a slaves SPI interrupt function.
*/

#include <p24FJ64GA002.h>
#include "xc.h"
#include "Project_Aragog_slave.h"
#include "Project_Aragog_general_setup.h"
#include "Project_Aragog_defines_slave.h"

/*Function: slaveSetup
---------------------------------------
 * Configures pins 6,7,11,14 as Data Out,Data In, Clock In, Slave Select In,
 * respectively. Then configures the Slave's SPI settings.
 */
void slaveSetup(void){
    //Pin configuration begin
    TRISBbits.TRISB2 = 0;                   //Pin 6 (RP2) is output
    TRISBbits.TRISB3 = 1;                   //Pin 7 (RP3) is input
    TRISBbits.TRISB4 = 1;                   //Pin 11 (RP4) is input
    TRISBbits.TRISB5 = 1;                   //Pin 14 (RP5) is input
    
    //Setup the Peripheral Pin Select
    __builtin_write_OSCCONL(OSCCON & 0xBF); //Unlocks the PPS for SPI selection
    //Configure Slave Input Functions for SPI
    RPOR1bits.RP2R = 7;                     //Pin 6 (RP2) is bind Data Output
    RPINR20bits.SDI1R = 3;                  //Pin 7 (RP3) is Data Input
    RPINR20bits.SCK1R = 4;                  //Pin 11 (RP4) is Clock Input
    RPINR21bits.SS1R = 5;                   //Pin 14 (RP5) is Slave Select In
    __builtin_write_OSCCONL(OSCCON & 0x40); //Re-locks the PPS
    
    //Pin configuration end
    
    //Slave SPI configuration begin
    SPI1CON1bits.MSTEN = 0;                 //Enable Slave mode
    SPI1CON1bits.MODE16 = 0;                //Enable in 8-bit mode
    SPI1CON1bits.SMP = 0;                   //Input data sampled at middle of
                                                //data output time
    SPI1CON1bits.CKE = 1;                   //Enable clock Active state as high
    SPI1CON1bits.CKP = 0;                   //Enable clock Idle state as low
    SPI1CON1bits.PPRE = 2;                  //Primary pre-scale is 4:1
    SPI1CON1bits.SPRE = 4;                  //Secondary pre-scale is 4:1
    
    /************************ADDED****************************/
    SPI1CON1bits.SSEN = 1;                  //Enable the Slave Select Pin
    SPI1STATbits.SPIEN = 1;                 //Turn on SPI
    /*********************************************************/

    IFS0bits.SPI1IF = 0;                    //Clear the SPI interrupt flag
    IEC0bits.SPI1IE = 1;                    //Enable the SPI interrupt
    //Slave SPI configuration end
}

/*Function: _SPIInterrupt
---------------------------------------
 * When the SS pin is pulled low, this interrupt is triggered, and causes the
 * SLAVE PIC to perform its movement cycle. Sends back a code to master when the
 *  call is acknowledged.
 */
void __attribute__((__interrupt__,__auto_psv__)) _SPI1Interrupt(void) {
    IFS0bits.SPI1IF = 0;                // Clear interrupt flag
 
    uint8_t dataToSend;
    uint8_t junk = 0xAA;
    SPI1BUF = junk;
    
    while(!SPI1STATbits.SPIRBF);
    uint8_t whatToSend = SPI1BUF;
    
    if (whatToSend == 0x57) {
        dataToSend = 0x75; 
    ////////// Right Half (Slave) Movement script ~ Set, Move forward, Set
        
        setServo(BackRightHip, BackRightHipOrigin);     // Back Right Hip set
        setServo(BackRightKnee, KneeOrigin);            // Back Right Knee set
        setServo(RightFeet, FeetOrigin);                // Back Right Foot set
        setServo(FrontRightKnee, KneeOrigin);           // Front Right Knee set
        setServo(FrontRightHip, FrontRightHipOrigin);   // Front Right Hip set

        delay_ms(300);
        setServo(BackRightKnee, 4000);                  // Raise Back Right Knee
        delay_ms(300);
        setServo(BackRightHip, 4800);                   // Rotate Back Right Hip forward
        delay_ms(300);
        setServo(BackRightKnee, KneeOrigin);            // Back Right Knee set
        delay_ms(900);

        delay_ms(300);
        setServo(FrontRightKnee, 2000);                 // Raise Front Right Knee
        delay_ms(300);
        setServo(FrontRightHip, 2800);                  // Rotate Front Right Hip forward
        delay_ms(300);
        setServo(FrontRightKnee, KneeOrigin);           // Front Right Knee set
        delay_ms(900);

        delay_ms(300);
        setServo(BackRightKnee, 2000);                  // Lower Back Right Knee
        delay_ms(300);
        setServo(BackRightHip, BackRightHipOrigin);     // Back Right Hip set
        delay_ms(300);
        setServo(BackRightKnee, KneeOrigin);            // Back Right Knee set
        delay_ms(900);

        delay_ms(300);
        setServo(FrontRightKnee, 4000);                 // Lower Front Right Knee
        delay_ms(300);
        setServo(FrontRightHip, FrontRightHipOrigin);   // Front Right Hip set
        delay_ms(300);
        setServo(FrontRightKnee, KneeOrigin);           // Front Right Knee set
    }
    
    else {
        // Debugging code, if the call issued by the master is not clear. Standby mode
        dataToSend = 0x8C;
        
        setServo(BackRightHip, BackRightHipOrigin);      // Back Right Hip set
        setServo(BackRightKnee, KneeOrigin);      // Back Right Knee set
        setServo(RightFeet, FeetOrigin);      // Back Right Foot set
        setServo(FrontRightKnee, KneeOrigin);      // Front Right Knee set
        setServo(FrontRightHip, FrontRightHipOrigin);      // Front Right Hip set
    }
    
    SPI1BUF = dataToSend;
    while(!SPI1STATbits.SPIRBF);
    junk = SPI1BUF;
}
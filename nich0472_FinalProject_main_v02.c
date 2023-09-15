/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , ,
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Main file assembling all of the other files
 * together such that ....
*/

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

#define MASTER //Uncommenting makes this PIC Program for the Master, else slave
#define SERVOS //To control servos

/*
Function: delay_ms
---------------------------------------
Simply delays any and all processing for the desired number of mili-seconds
ms: the number of mili-seconds to wait
Used for debugging purposes.
 */
void delay_ms(int ms){
    while(ms-- > 0){
        asm("repeat #15998");
        asm("nop");
    }
}

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
Function: masterSetup
---------------------------------------
 * Configures pins 6,7,11,14 as Data Out,Data In, Clock Out, Slave Select Out,
 * respectively. Then configures the Master's SPI settings.
 */
void masterSetup(void){
    //Pin configuration begin
    TRISBbits.TRISB2 = 0;                   //Pin 6 (RP2) is output
    TRISBbits.TRISB3 = 1;                   //Pin 7 (RP3) is input
    TRISBbits.TRISB4 = 0;                   //Pin 11 (RP4) is output
    TRISBbits.TRISB5 = 0;                   //Pin 14 (RP5) is output
    
    /************************ADDED**********************************/
    _LATB5 = 1;                             //Slave select is enabled high by
                                                //default until ready
    /***************************************************************/
  
    //Setup the Peripheral Pin Select
    __builtin_write_OSCCONL(OSCCON & 0xBF); // Unlocks the PPS for OC selection
    //Configure Master Input Functions for SPI
    RPOR1bits.RP2R = 7;                     //Pin 6 (RP2) is Data Output
    RPINR20bits.SDI1R = 3;                  //Pin 7 (RP3) is Data Input
    RPOR2bits.RP4R = 8;                     //Pin 11 (RP4) is Clock Output
    /**********************************************************************/
//    RPOR2bits.RP5R = 9;                     //Pin 14 (RP5) is Slave Select Out
    /**********************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0x40); //Re-locks the PPS
    
    //Pin configuration end
    
    //Master SPI configuration begin
    SPI1BUF = 0;                            //Clear SPI buffer
    IEC0bits.SPI1IE = 0;                    //Disable interrupt
    IFS0bits.SPI1IF = 0;                    //Clear interrupt flag
    SPI1STATbits.SPIROV = 0;                //Clear overflow flag
    
    SPI1CON1bits.MSTEN = 1;                 //Enable Master mode
    SPI1CON1bits.DISSCK	= 0;                //Enable clock output pin
    SPI1CON1bits.DISSDO	= 0;                //Enable Data out pin
    SPI1CON1bits.MODE16 = 0;                //Enable in 8-bit mode
    SPI1CON1bits.SMP = 0;                   //Input data sampled at middle of
                                                //data output time
    SPI1CON1bits.CKE = 1;                   //Enable clock Active state as high
    SPI1CON1bits.CKP = 0;                   //Enable clock Idle state as low
    SPI1CON1bits.PPRE = 2;                  //Primary pre-scale is 4:1
	SPI1CON1bits.SPRE = 4;                  //Secondary pre-scale is 4:1
    //Master SPI configuration end
    
    SPI1STATbits.SPIEN = 1;                 //Enable SPI port, clear status
    
    IEC0bits.SPI1IE = 0;                    //Enable the interrupt
}

/*
Function: slaveSetup
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
//////////////////////// End of Generalized Setups ///////////////////////////
//////////////////////////////////////////////////////////////////////////////

/*****************************************************************************/
/**
 * An example of a typical SPI transaction - this follows a "call and response" 
 * structure where the master requests some data then the slave responds with it
 * 
 * @param whatToRead - the master typically requests some specific data from the
 * slave, this byte is sent first to indicate what that data is
 * @param pReadData - pointer to read data, the desired response will be written
 * to this variable, this can also be a return value
 */
void readByteFromSlave(uint8_t whatToRead, uint8_t *pReadData){
    // pull SS line low, initiating SPI transfer
    _LATB5 = 0;
    
    /** first data transfer (call) **/
    // ask the slave for some data
    SPI1BUF = whatToRead;
    // receive junk data from slave 
    while(!SPI1STATbits.SPIRBF);
    uint8_t junk = SPI1BUF;
    
    /** second data transfer (response) **/
    // send junk data back so we can receive what we want
    SPI1BUF = junk;
    // receive the byte we wanted
    while(!SPI1STATbits.SPIRBF);
    *pReadData = SPI1BUF;
    
    // pull SS line high, ending SPI transfer
    _LATB5 = 1;
}

#ifndef MASTER
void __attribute__((__interrupt__,__auto_psv__)) _SPI1Interrupt(void) {
    IFS0bits.SPI1IF = 0;                // Clear interrupt flag
    
    // Perform necessary actions
    // ...
    
    /////////// Used for debugging of circuit and code
    uint8_t dataToSend;
        
    // typically both devices send data at the first clock pulse
    // if you set up the read function on the master right you might not need to read junk data
    uint8_t junk = 0xAA;
    SPI1BUF = junk;
    while(!SPI1STATbits.SPIRBF);
    uint8_t whatToSend = SPI1BUF;
    
    if (whatToSend == 0x57) {
        dataToSend = 0x75; 
    }else {
        dataToSend = 0x8C;
    }
    
    SPI1BUF = dataToSend;
    while(!SPI1STATbits.SPIRBF);
    junk = SPI1BUF;
    ////////// End of Debugging code
}
#endif
/******************************************************************************/

int main(void) {
    basicSetup(); //both PICS need this setup
#ifdef SERVOS     //If servos are not neccecary to run, then don't set them up 
    servoSetup();
#endif
    
#ifdef MASTER     //If master is defined, set it up, otherwise it's a Slave
    masterSetup();
#else 
    slaveSetup();
#endif
    
    //Initialize both PICs to start with only RED LED high
    setServo(1,20000);
    setServo(2,0);
    setServo(3,0);
    while (1) {     //Forever Loop
#ifdef MASTER
        // request specific data from slave (maybe 0x57 means "read from motor 1", you get to pick)
        uint8_t whatToRead = 0x57;
        uint8_t readData;

        readByteFromSlave(whatToRead, &readData);

        if (readData == 0x75) {
            // do something
            setServo(1,0);              //RED LED low
            setServo(2,20000);              //Green LED high
        } else if (readData == 0x8C) {
            // do something else
            setServo(3,20000);
//            setServo(2,20000);
        }else {
            // error
            setServo(1,20000);
        }
        delay_ms(1000);
        setServo(1,0);                  
        setServo(2,0);
        setServo(3,0);
        delay_ms(1000);
#else
        //If is a slave, goes here instead
#endif
    }
}

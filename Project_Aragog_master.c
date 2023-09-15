/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Function file for the MASTER PIC. Contains a setup
 * function for SPI, and also contains a master's SPI call function.
*/


#include <p24FJ64GA002.h>
#include <stdint.h>
#include "xc.h"
#include "Project_Aragog_master.h"
#include "Project_Aragog_defines_master.h"


/*Function: masterSetup
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

/**
 * Function: readByteFromSlave:
 * Follows a "call and response" structure where the master requests some data,
 * then the slave responds with different data.
 * 
 * @param whatToRead - the master typically requests some specific data from the
 * slave, this byte is sent first to indicate what that data is
 * 
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
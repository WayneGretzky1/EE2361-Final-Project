/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Header file for the definitions of servo names and
 * origin positions used by the SLAVE PIC.
*/

#ifndef PROJECT_ARAGOG_DEFINES_SLAVE_H
#define	PROJECT_ARAGOG_DEFINES_SLAVE_H

#ifdef	__cplusplus
extern "C" {
#endif

#define BackRightHip 1
#define BackRightKnee 2
#define RightFeet 3
#define FrontRightKnee 4
#define FrontRightHip 5

#define KneeOrigin 3000
#define FeetOrigin 3000
#define BackRightHipOrigin 4000
#define FrontRightHipOrigin 2000


#ifdef	__cplusplus
}
#endif

#endif	/* PROJECT_ARAGOG_DEFINES_SLAVE_H */


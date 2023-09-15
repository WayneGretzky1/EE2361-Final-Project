/*
Date: 02 Apr 2023
Name: William Nichols, Brock Bye, Jake Borham
Student ID number: 5667529 , 5583303 , 5686531
Course number: EE-2361
Term: Spring 2023
Lab/assignment number: Final Project
Short Program Description: Header file for the definitions of servo names and
 * origin positions used by the MASTER PIC.
*/

#ifndef PROJECT_ARAGOG_DEFINES_MASTER_H
#define	PROJECT_ARAGOG_DEFINES_MASTER_H
#ifdef	__cplusplus
extern "C" {
#endif
#define BackLeftHip 1
#define BackLeftKnee 2
#define LeftFeet 3
#define FrontLeftKnee 4
#define FrontLeftHip 5

#define KneeOrigin 3000
#define FeetOrigin 3000
#define BackLeftHipOrigin 2000
#define FrontLeftHipOrigin 4000



#ifdef	__cplusplus
}
#endif

#endif	/* PROJECT_ARAGOG_DEFINES_MASTER_H */


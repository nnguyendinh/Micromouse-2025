/*
 * controller.c
 */

#include "main.h"
#include "controller.h"
#include "motors.h"
#include "pid3.h"
#include "encoders.h"

#include <math.h>


void move(int16_t mm) {	// Make n 90 degree turns (no acceleration)


	resetPID();
	setPIDGoalD(mm);
	setPIDGoalA(0);

	while(!PIDdone())
	{

	}

	resetPID();

}


void turn(int16_t degrees) {	// Make n 90 degree turns (no acceleration)


	resetPID();
	setPIDGoalD(0);
	setPIDGoalA(degrees);

	while(!PIDdone())
	{

	}

	resetPID();

}


void frontCorrection() {

//	int16_t forward_left = 0;
//	int16_t forward_right = 0;
//
//	while(1) {
////		if (fabs(forward_left - goal_forward_left) > 200 && fabs(forward_right - goal_forward_right) > 200) {
////			if (forward_left - goal_forward_left < -250 || forward_right - goal_forward_right < -250) {
////				moveEncoderCount(15);
////			}
////			else if (forward_left - goal_forward_left > 150 || forward_right - goal_forward_right > 150) {
////				moveEncoderCount(-15);
////			}
////			if ((forward_left - goal_forward_left) - (forward_right - goal_forward_right) < -300) {
////				turnEncoderCount(30);
////			}
////			else if ((forward_left - goal_forward_left) - (forward_right - goal_forward_right) < -300) {
////				turnEncoderCount(-30);
////			}
////		}
//		if (forward_left - goal_forward_left > 250/* || forward_right - goal_forward_right > 300*/) {
//			moveEncoderCount(-15);
//
////			if ((forward_left - goal_forward_left) - (forward_right - goal_forward_right) < -300) {
////				turnEncoderCount(30);
////			}
////			else if ((forward_left - goal_forward_left) - (forward_right - goal_forward_right) < -300) {
////				turnEncoderCount(-30);
////			}
//		}
//		else if (forward_left - goal_forward_left < -250/* || forward_right - goal_forward_right > 300*/) {
//			moveEncoderCount(15);
//		}
//		else {
//			break;
//		}
//	}
//
//	resetPID();

}


/*
 * controller.c
 */

#include "main.h"
#include "controller.h"
#include "motors.h"
#include "pid3.h"
#include "encoders.h"

#include <math.h>

float constant = 1.01;
float constant2 = 50000;


void move(int16_t mm) {	// Make n 90 degree turns (no acceleration)


	resetPID();
	setPIDGoalD(mm * constant + (mm * mm / constant2));
	setPIDGoalA(0);

	setState(MOVING);

	while(!PIDdone())
	{

	}

	resetPID();

}


void turn(int16_t degrees) {	// Make n 90 degree turns (no acceleration)


	resetPID();

	setState(TURNING);

	setPIDGoalD(0);
	setPIDGoalA(0.99 * degrees);

	while(!PIDdone())
	{

	}

	resetPID();

}


void frontCorrection() {

	int16_t left_temp = TIM3->CNT;
	int16_t right_temp = TIM8->CNT;

	setState(FRONTING);

	while(!PIDdone())
	{

	}

	TIM3->CNT = left_temp;
	TIM8->CNT = right_temp;

	resetPID();



}


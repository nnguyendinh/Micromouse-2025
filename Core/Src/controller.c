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


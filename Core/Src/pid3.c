/*
 * pid.c
 */

#include <math.h>
#include "pid3.h"
#include "main.h"
#include "motors.h"
#include "encoders.h"
#include "gyro.h"
#include "utility.h"
#include "irs.h"

// Constants
float kPw = 0.015; // 0.02
float kDw = 0.3;
float kPx = 0.02; // 0.015
float kDx = 1.5; // 1.5

const float PWMMaxx = 0.3; // 0.3
const float PWMMaxw = 0.16;	//0.16

const float PWMMinx = 0.25;	// 0.25
const float PWMMinw = 0.25;	// 0.25
const float PWMMin = 0.23;	// 0.23

const float xacceleration = 0.0005;


// Goal Parameters
float goal_distance = 0;
float goal_angle = 0;

// Error Parameters
float angle_error = 0;
float old_angle_error = 0;
float old_angle_errors[10] = {0};
float angle_correction = 0;

float distance_error = 0;
float old_distance_error = 0;
float old_distance_errors[10] = {0};
float distance_correction = 0;
float old_distance_correction = 0;

// IR Adjustments
float kPir = 0.01;		// 0.01	for 2 walls
float kPir2 = 0.015;		// 0.015 for 1 wall

float IRadjustment = 0;

int16_t IRAngleOffset = 0;
int16_t goal_front_left;
int16_t goal_front_right;
int16_t goal_left;
int16_t goal_right;

float adjusted_angle;

// Fronting
const float front_kPx = 0.0002; // 0.3
const float front_kPw = 0.35; // 0.2

// Miscellaneous
STATE state = REST;
float left_PWM_value = 0;
float right_PWM_value = 0;
int goal_reached_timer = 0;

float gyro_angle = 0;
float gyro_vel = 0;

void setPIDGoalD(int16_t distance) {
	goal_distance += distance;
}
void setPIDGoalA(int16_t angle) {
	goal_angle += angle;
}

void setState(STATE curr_state) {
	state = curr_state;
}

void setIRGoals(int16_t front_left_goal, int16_t front_right_goal, int16_t left_goal, int16_t right_goal) {

	IRAngleOffset = left_goal - right_goal;
	goal_front_left = front_left_goal;
	goal_front_right = front_right_goal;
	goal_left = left_goal;
	goal_right = right_goal;

}

// TODO: CHANGE TO USE WALL CHECK FUNCTIONS
void setIRAngle(float left, float right){

	if (left > 600 && right > 600 && state == MOVING)
	{
		IRadjustment = (kPir * ((left - right) - IRAngleOffset));
	}
	else if (left > 600 && state == MOVING)
	{
		IRadjustment = (kPir2 * (left - goal_left));
	}
	else if (right > 600 && state == MOVING)
	{
		IRadjustment = (kPir2 * (goal_right - right));
	}
	else
		IRadjustment = 0;
}

void PDController() {

	if (state == START) {
		return;
	}

//////////////////////////	CALCULATE DISTANCE AND ANGLE CORRECTION /////////////////////////

	setIRAngle(ir_left, ir_right);

	adjusted_angle = goal_angle + IRadjustment;

	if (state == TURNING || state == FRONTING) {
		adjusted_angle = goal_angle;
	}

	readGyro(&gyro_vel);
	gyro_vel /= 2000;
	gyro_angle += gyro_vel;

	angle_error = adjusted_angle - gyro_angle;
	angle_correction = kPw * angle_error + kDw * (angle_error - old_angle_error);

	distance_error = goal_distance - ((getLeftEncoderCounts() + getRightEncoderCounts()) / (2 * ENC_TO_MM));

	distance_correction = kPx * distance_error + kDx * (distance_error - old_distance_error);

	if (state == FRONTING) {
		distance_correction = front_kPx * (goal_front_left - ir_front_left);
	}

/////////////////////////////////	APPLY ACCELERATION	///////////////////////////////

	if (fabs(distance_error) > 100)
	{		// If we're going straight and not at the end, apply acceleration
		if (fabs(distance_correction - old_distance_correction) > xacceleration)
		{
			distance_correction = old_distance_correction + (xacceleration * sign(distance_correction - old_distance_correction));
		}
	}

////////////////////// ROUND DISTANCE AND ANGLE CORRECTION	//////////////////////////

	if (fabs(distance_correction) > PWMMaxx)		// Upper Limit for PWM
		distance_correction = sign(distance_correction) * PWMMaxx;

	if (fabs(angle_correction) > PWMMaxw)
		angle_correction = sign(angle_correction) * PWMMaxw;

	left_PWM_value = (distance_correction + angle_correction);
	right_PWM_value = (distance_correction - angle_correction);

}

void updatePID() {

///// CALCULATE PREVIOUS ANGLE AND DISTANCE ERRORS //////////////////////////

//////////////////////	CALCULATE MOTOR PWM VALUES	/////////////////////

	PDController();

	if (fabs(left_PWM_value) > 0.03) {
		left_PWM_value += sign(left_PWM_value) * PWMMin;

	}

	if (fabs(right_PWM_value) > 0.03) {
		right_PWM_value += sign(right_PWM_value) * PWMMin;

	}

//////////////////	SET PWM VALUES AND CHECK FOR GOAL REACHED ////////////////////////

	setMotorLPWM(left_PWM_value);
	setMotorRPWM(right_PWM_value);

	if((angle_error < 2.5 && angle_error > -2.5 && distance_error < 2 && distance_error > -2) || (state == FRONTING && distance_correction < 0.1 && distance_correction > -0.1))
		goal_reached_timer++;					// Increments goal reached timer when errors are within a certain threshold

	else
		goal_reached_timer = 0;

///////////////////// UPDATE PREVIOUS ANGLE AND DISTANCE ERRORS //////////////////////////

	old_angle_error = angle_error;

	old_distance_error = distance_error;

	old_distance_correction = distance_correction;

	for(int i = 9; i > 0; i--)
		old_angle_errors[i] = old_angle_errors[i-1];	// Adds the newest angle_error to array and shifts everything to the right
	old_angle_errors[0] = angle_error;

	for(int i = 9; i > 0; i--)
		old_distance_errors[i] = old_distance_errors[i-1];	// Adds the newest distance_error to array and shifts everything right
	old_distance_errors[0] = distance_error;

}

int8_t PIDdone(){ // There is no bool type in C. True/False values are represented as 1 or 0.

	if (goal_reached_timer >= 50)
	{
		resetPID();
		return 1;
	}
	else
		return 0;

}

void resetPID() {

//////////////	RESET ALL ANGLE AND DISTANCE ERRORS TO 0	////////////////
	angle_error = 0;
	old_angle_error = 0;
	angle_correction = 0;

	for (int i = 0; i < 10; i++)
		old_angle_errors[i] = 0;

	distance_error = 0;
	old_distance_error = 0;
	distance_correction = 0;

	for (int i = 0; i < 10; i++)
		old_distance_errors[i] = 0;

////////////// 	RESET ALL GOALS AND ENCODER COUNTS TO 0 	///////////////////
//	goal_angle = 0;
//	gyro_angle = 0;
//	goal_distance = 0;
	goal_reached_timer = 0;

//	resetEncoders();


	if (getLeftEncoderCounts() > 60000 || getRightEncoderCounts() > 60000) {
		resetEncoders();
		goal_distance = 0;
	}

	resetMotors();
	setState(REST);
}

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
const float kPw = 0.04;
const float kDw = 0.03;
const float kPx = 0.015;
const float kDx = 0.015;

const float PWMMaxx = 0.25; // 0.65
const float PWMMaxw = 0.1;	//0.35

const float PWMMinx = 0.22;	// 0.32
const float PWMMinw = 0.22;	// 0.32
const float PWMMin = 0.21;	// 0.28

const float xacceleration = 0.001;


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
const float kPir = 0.001;		// 0.03	for 2 walls
const float kPir2 = 0.003;		// 0.05 for 1 wall

float IRadjustment = 0;

int16_t IRAngleOffset = 0;
int16_t goal_forward_left;
int16_t goal_forward_right;
int16_t goal_left;
int16_t goal_right;

// Miscellaneous
STATE state = REST;
float left_PWM_value = 0;
float right_PWM_value = 0;
int goal_reached_timer = 0;

float gyro_angle = 0;
float gyro_vel = 0;

void setPIDGoalD(int16_t distance) {
	goal_distance = distance;
}
void setPIDGoalA(int16_t angle) {
	goal_angle += angle;
}

void setState(STATE curr_state) {
	state = curr_state;
}

void setIRGoals(int16_t front_left_goal, int16_t front_right_goal, int16_t left_goal, int16_t right_goal) {

	IRAngleOffset = left_goal - right_goal;
	goal_forward_left = front_left_goal;
	goal_forward_right = front_right_goal;
	goal_left = left_goal;
	goal_right = right_goal;

}

// TODO: CHANGE TO USE WALL CHECK FUNCTIONS
void setIRAngle(float left, float right){

	if (left > 600 && right > 600 && goal_angle == 0)
	{
		IRadjustment = (kPir * ((left - right) - IRAngleOffset));
	}
	else if (left > 600 && goal_angle == 0)
	{
		IRadjustment = (kPir2 * (left - goal_left));
	}
	else if (right > 600 && goal_angle == 0)
	{
		IRadjustment = (kPir2 * (goal_right - right));
	}
	else
		IRadjustment = 0;
}

void PDController() {

//////////////////////////	CALCULATE DISTANCE AND ANGLE CORRECTION /////////////////////////

	setIRAngle(ir_left, ir_right);

	float adjusted_angle = goal_angle + IRadjustment;

	readGyro(&gyro_vel);
	gyro_vel /= 1000;
	gyro_angle += gyro_vel;

	angle_error = adjusted_angle - gyro_angle;
	angle_correction = kPw * angle_error + kDw * (angle_error - old_angle_error);

	distance_error = goal_distance - ((getLeftEncoderCounts() + getRightEncoderCounts()) / (2 * ENC_TO_MM));

	distance_correction = kPx * distance_error + kDx * (distance_error - old_distance_error);

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

	if(angle_error < 2 && angle_error > -2 && distance_error < 2 && distance_error > -2)
		goal_reached_timer++;					// Increments goal reached timer when errors are within a certain threshold

	else
		goal_reached_timer = 0;

///////////////////// UPDATE PREVIOUS ANGLE AND DISTANCE ERRORS //////////////////////////

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
	goal_distance = 0;
	goal_reached_timer = 0;

	resetEncoders();
	resetMotors();
	setState(REST);
}

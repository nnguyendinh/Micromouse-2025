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

// Constants
const float kPw = 0.01;
const float kDw = 0.0;
const float kPx = 0.005;
const float kDx = 0.0;

const float PWMMaxx = 0.65; // 0.65
const float PWMMaxw = 0.4;	//0.35
const float PWMMinx = 0.32;	// 0.32
const float PWMMinw = 0.32;	// 0.32
const float PWMMin = 0.3;	// 0.28

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

void PDController() {

//////////////////////////	CALCULATE DISTANCE AND ANGLE CORRECTION /////////////////////////

	float adjusted_angle = goal_angle/* + IRadjustment*/;

	readGyro(&gyro_vel);
	gyro_vel /= 1000;
	gyro_angle += gyro_vel;

	angle_error = adjusted_angle - gyro_angle;
	angle_correction = kPw * angle_error + kDw * (angle_error - old_angle_error);

	distance_error = goal_distance - ((getLeftEncoderCounts() + getRightEncoderCounts()) / (2 * 3.38333333333));

	distance_correction = kPx * distance_error + kDx * (distance_error - old_distance_error);

/////////////////////////////////	APPLY ACCELERATION	///////////////////////////////

	if (fabs(distance_error) > 100)
	{		// If we're going straight and not at the end, apply acceleration
		if (fabs(distance_correction - old_distance_correction) > xacceleration)
		{
			distance_correction = old_distance_correction + (xacceleration * sign(distance_correction - old_distance_correction));
		}
	}

////////////////////// ROUND DISTANCE OR ANGLE CORRECTION	//////////////////////////

	switch(state) {		// Apply lower limits of PWM for various states
		case MOVING:
			if (fabs(distance_correction) > 0.01 && fabs(distance_correction) < PWMMinx)
				distance_correction = sign(distance_correction) * PWMMinx;
			break;
		case TURNING:
			if (fabs(angle_correction) > 0.01 && fabs(angle_correction) < PWMMinw)
				angle_correction = sign(angle_correction) * PWMMinw;
			break;
		default:
			break;
	}

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

////////////////////	NORMALIZE LEFT AND RIGHT PWM VALUES ////////////////

	// If we are kinda seriously trying to move)
	if (fabs(left_PWM_value) > 0.01 || fabs(right_PWM_value) > 0.01) {

		// If the left PWM is greater than the right, need left PWM to be higher than the minimum
		if (state == MOVING && fabs(left_PWM_value) > fabs(right_PWM_value) && fabs(right_PWM_value) < PWMMin) {
			left_PWM_value += sign(right_PWM_value) * (PWMMin - fabs(right_PWM_value));
			right_PWM_value = sign(right_PWM_value) * PWMMin;
		}

		// If the right PWM is greater than the left, need right PWM to be higher than the minimum
		else if (state == MOVING && fabs(right_PWM_value) > fabs(left_PWM_value) && fabs(left_PWM_value) < PWMMin) {
			right_PWM_value += sign(left_PWM_value) * (PWMMin - fabs(left_PWM_value));
			left_PWM_value = sign(left_PWM_value) * PWMMin;
		}

		// If not moving, apply regular basic limits
		else {
			if (fabs(left_PWM_value) < PWMMin){
				left_PWM_value = sign(left_PWM_value) * PWMMin;
			}

			if (fabs(right_PWM_value) < PWMMin){
				right_PWM_value = sign(right_PWM_value) * PWMMin;
			}
		}
	}

//////////////////	SET PWM VALUES AND CHECK FOR GOAL REACHED ////////////////////////

	setMotorLPWM(left_PWM_value);
	setMotorRPWM(right_PWM_value);

	if(angle_error < 1 && angle_error > -1 && distance_error < 2 && distance_error > -2)
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

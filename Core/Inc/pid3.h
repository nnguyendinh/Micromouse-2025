/*
 * pid3.h
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"
typedef enum
{
	REST = 0,
	MOVING = 1,
	TURNING = 2,
	FRONTING = 3,
	START = 4,
}STATE;

#define ENC_TO_MM 3.4 // 3.38

// Functions
void setIRGoals(int16_t front_left_goal, int16_t front_right_goal, int16_t left_goal, int16_t right_goal);
//void setIRDistance(int16_t curr_forward_left, int16_t curr_forward_right);
void setState(STATE curr_state);

void resetPID(void);
void updatePID(void);
void setPIDGoalD(int16_t distance);
void setPIDGoalA(int16_t angle);
int8_t PIDdone();


#endif /* INC_PID_H_ */

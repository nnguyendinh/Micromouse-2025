/*
 * irs.h
 */

#include "main.h"

#ifndef INC_IRS_H_
#define INC_IRS_H_


// Using this enumeration makes the code more readable
typedef enum
{
	IR_FORWARD_LEFT = 0,
	IR_LEFT = 1,
	IR_RIGHT = 2,
	IR_FORWARD_RIGHT = 3
}IR;

extern float ir_left, ir_front_left, ir_front_right, ir_right;

#endif /* INC_IRS_H_ */

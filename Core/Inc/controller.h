/*
 * controller.h
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "main.h"

#define MOVE_COUNTS 586 // 610
#define TURN_COUNTS 470 // 470 // changed from 435 on cloth table
//#define TURN_COUNTS 85
#define INIT_COUNTS 300

void move(int16_t mm);
void turn(int16_t degrees);

#endif /* INC_CONTROLLER_H_ */

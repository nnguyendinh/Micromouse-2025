/*
 * utility.c
 */

#include "utility.h"
#include "irs.h"

int16_t left_wall_threshold = 550;
int16_t right_wall_threshold = 500;
int16_t front_wall_threshold = 450;

int16_t left_wall = 0;
int16_t right_wall = 0;
int16_t front_wall = 0;

void setLeftWall(int wall) { left_wall_threshold = wall; }
void setRightWall(int wall) { right_wall_threshold = wall; }
void setFrontWall(int wall) { front_wall_threshold = wall; }

int16_t leftWallCheck() {
	if (ir_left > left_wall_threshold) {
		left_wall = 1;
	}
	else {
		left_wall = 0;
	}
	return left_wall;
}

int16_t rightWallCheck() {
	if (ir_right > right_wall_threshold) {
		right_wall = 1;
	}
	else {
		right_wall = 0;
	}
	return right_wall;
}

int16_t frontWallCheck() {
	if (ir_front_right > front_wall_threshold) {
		front_wall = 1;
	}
	else {
		front_wall = 0;
	}
	return front_wall;
}

int16_t sign(float x) {
	if (x > 0)
	{
		return 1;
	}
	else if (x < 0)
	{
		return -1;
	}
	else
	{
		return 0;
	}
}

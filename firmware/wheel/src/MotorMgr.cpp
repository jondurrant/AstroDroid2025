/*
 * MotorMgr.cpp
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */

#include "MotorMgr.h"
#include <math.h>

MotorMgr::MotorMgr(uint8_t gpCW, uint8_t gpCCW,
		uint8_t gpA, uint8_t gpB) {
	xGPCW= gpCW;
	xGPCCW = gpCCW;
	xGPA = gpA;
	xGPB = gpB;

	gpio_init(xGPCW);
	gpio_set_function(xGPCW, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPCW, 0);
	uint slice_num = pwm_gpio_to_slice_num(xGPCW);
	pwm_set_enabled(slice_num, true);

	gpio_init(xGPCCW);
	gpio_set_function(xGPCCW, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGPCCW, 0);
	slice_num = pwm_gpio_to_slice_num(xGPCCW);
	pwm_set_enabled(slice_num, true);

	GPIOInputMgr::getMgr()->addObserver(xGPA, this);
	GPIOInputMgr::getMgr()->addObserver(xGPB, this);

}

MotorMgr::~MotorMgr() {
	// TODO Auto-generated destructor stub
}


void MotorMgr::setThrottle(float percent, bool cw){
	xThrottle = percent;
	xCW = cw;

	//printf("setThrottle(%.2f, %d)\n", percent, cw);

	if (xThrottle < 0 ){
		printf("Throttle was < 0, resetting to zero\n");
		xThrottle == 0.0;
	}

	if (xThrottle == 0.0){
		xActRadPerSec = 0.0;
		xMvAvgRadPerSec = 0.0;
		xLastTime = 0;
		pwm_set_gpio_level(xGPCW, 0);
		pwm_set_gpio_level(xGPCCW, 0);
		//printf("PWM STOP\n");
		return;
	}

	if (xThrottle > 1.0 ){
		xThrottle = 1.0;
	}

	int pwm = (int)((float)(0xffff) * xThrottle);
	if (cw){
		pwm_set_gpio_level(xGPCCW, 0);
		pwm_set_gpio_level(xGPCW, pwm);
		//printf("GP%d, pwm %d\n", xGPCW, pwm);
	} else {
		pwm_set_gpio_level(xGPCW, 0);
		pwm_set_gpio_level(xGPCCW, pwm);
		//printf("GP%d, pwm %d\n", xGPCCW, pwm);
	}
	//printf("PWM %d, Throttle %0.2f\n", pwm, xThrottle);
}

void MotorMgr::handleGPIO(uint gpio, uint32_t events){

	uint8_t c;
	c = gpio_get(xGPA);
	c = c << 1;
	c = (gpio_get(xGPB)) | c;

	if (xRotEncCW[xLast] == c){
		xCount++;
		if (xCount > 3){
			xPos++;
			xDeltaPos++;
			if (xPos == xNumTicks){
				xPos = 0;
			}
			//printf("Clockwise %d %d\n", xPos, xCount);
			handleRotate(true);
			xCount = 0;
		}

		xLast = c;
	}

	if (xRotEncCCW[xLast] == c){
		xCount-- ;
		if (xCount < -3){
			xPos--;
			xDeltaPos--;
			if (xPos == -1){
				xPos = xNumTicks - 1;
			}
			//printf("Withershins %d %d\n", xPos, xCount);
			handleRotate(false);
			xCount = 0;
		}

		xLast = c;
	}
}

void MotorMgr::handleRotate(bool cw){
	uint64_t now = to_us_since_boot (get_absolute_time ());

	if (xLastTime != 0){
		uint64_t us = now - xLastTime;

		 xActRadPerSec = xRadTick /  ((float)us / 1000000.0);

		 xMvAvgRadPerSec = ((xMvAvgRadPerSec * 8.0) + (xActRadPerSec * 2.0)) / 10.0;

	}
	xLastTime = now;

	if (xDoDelta){
		xDoDeltaPos--;
		if (xDoDeltaPos <= 0){
			setThrottle(0, true);
			xDoDelta = false;
			targetStop();
		}
	}
}



float MotorMgr::getThrottle(){
	return xThrottle;
}


bool MotorMgr::isCW(){
	return xCW;
}


/***
 * Radian possition of wheel
 * @return 0.0 >= r < 2* PI
 */
float MotorMgr::getRadians(){
	float rad = (float)xPos / (float)xNumTicks;
	rad = rad * (2.0 * M_PI);
	return rad;
}


float MotorMgr::getRadPerSec(){
	checkStopped();
	return xActRadPerSec;
}

float MotorMgr::getAvgRadPerSec(){
	checkStopped();
	return xMvAvgRadPerSec;
}

void MotorMgr::checkStopped(){
		if (xLastTime != 0){
			uint64_t now = to_us_since_boot (get_absolute_time ());
			uint64_t us = now - xLastTime;

			if (us > 1000 * 250){
				//Stopped
				xActRadPerSec = 0.0;
				xMvAvgRadPerSec = 0.0;
			}
		}
}


/***
 * Get the delta of ticks since last cleared
 * @param clear - if true will be reset to zero after call
 * @return number of ROTENC ticks
 */
int32_t MotorMgr::getDeltaPos(bool clear){
	int32_t res = xDeltaPos;
	if (clear){
		xDeltaPos = 0;
	}
	return res;
}

/***
 * Get the delta of radians since last cleared
 * @param clear - if true will be reset to zero after call
 * @return Radians turn since last call (>0 CW, <0 CCW).
 */
float MotorMgr::getDeltaRadians(bool clear){
	float res = (float)getDeltaPos(clear);
	res = res / (float)xNumTicks;
	res = res * (2.0 * M_PI);
	return res;
}

void MotorMgr::turnDeltaPos(int32_t pos, float throttle, bool cw ){
	xDoDelta = true;
	xDoDeltaPos = pos;
	setThrottle(throttle, cw);
}

void MotorMgr::turnDeltaRadians(float rad, float throttle, bool cw){

	int32_t pos = (int32_t) (rad /  ( 2 *M_PI)  * (float)xNumTicks );
	turnDeltaPos( pos,  throttle,  cw );

}


void MotorMgr::printConfig(){
	printf("Max number of Ticks %u\n", xNumTicks);
	printf("Radians per Tick %f\n", xRadTick);
}

void MotorMgr::targetStop(){
	//NOP
}


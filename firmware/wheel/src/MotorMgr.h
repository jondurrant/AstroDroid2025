/*
 * MotorMgr.h
 *
 *  Created on: 28 May 2023
 *      Author: jondurrant
 */

#ifndef ENCODER_SRC_MOTORMGR_H_
#define ENCODER_SRC_MOTORMGR_H_

#include <pico/stdlib.h>
#include "stdio.h"
#include "hardware/pwm.h"
#include "GPIOInputMgr.h"
#include "GPIOObserver.h"
#include <math.h>

#define SPEED_MIN 2.3
#define SPEED_MAX 5.0

class MotorMgr : public GPIOObserver{
public:
	/***
	 * Constructor
	 * @param gpCW - GP Pad for PWM for Clockwise
	 * @param gpCCW - GP Pad for PWM for Clockwise
	 * @param gpA - GP Pad for ROTENC A
	 * @param gpB - GP Pad for ROTENC AB
	 */
	MotorMgr(uint8_t gpCW, uint8_t gpCCW,
			uint8_t gpA, uint8_t gpB);
	virtual ~MotorMgr();

	/***
	 * Set throttle as a percentage
	 * @param percent: 0.0 < p <= 1.0
	 * @param cw - true of clockwise
	 */
	void setThrottle(float percent, bool cw);

	/***
	 * Get the throttle speed
	 * @return 0.0 < o <= 1.0
	 */
	float getThrottle();

	/***
	 * Is currently set to clockwise rotation
	 * @return
	 */
	bool isCW();


	float getRadPerSec();
	float getAvgRadPerSec();


	/***
	 * Radian possition of wheel
	 * @return 0.0 >= r < 2* PI
	 */
	float getRadians();

	/***
	 * Get the delta of ticks since last cleared
	 * @param clear - if true will be reset to zero after call
	 * @return number of ROTENC ticks
	 */
	int32_t getDeltaPos(bool clear=true);


	/***
	 * Get the delta of radians since last cleared
	 * @param clear - if true will be reset to zero after call
	 * @return Radians turn since last call (>0 CW, <0 CCW).
	 */
	float getDeltaRadians(bool clear=true);


	void turnDeltaPos(int32_t pos, float throttle, bool cw = true);

	void turnDeltaRadians(float rad, float throttle, bool cw = true);

	void printConfig();

protected:
	/***
	 * Handle GPIO event on teh ROTENC
	 * @param gpio
	 * @param events
	 */
	virtual void handleGPIO(uint gpio, uint32_t events);

	/***
	 * Process rotation
	 * @param cw
	 */
	virtual void handleRotate(bool cw);


	virtual void checkStopped();

	bool xCW = true;

private:


	uint8_t xGPCW=0;
	uint8_t xGPCCW=0;
	uint8_t xGPA;
	uint8_t xGPB;
	uint32_t xSlotTime = 0;

	float xThrottle = 0.0;
	float xActRadPerSec = 0.0;
	float xMvAvgRadPerSec = 0.0;

	// Current position
	int16_t xPos = 0;
	int32_t xDeltaPos = 0;

	// Number of ticks to track in total rotation
	uint16_t xNumTicks=16 * 131 + 4;
	float xRadTick = (2 * M_PI) / (float)xNumTicks;


	// Last position of switches
	uint8_t xLast=0;

	// Count way through the signal
	int8_t xCount=0;

	//These are lookups for the sequence to see in CW and CCW motion
	uint8_t xRotEncCW[4] ={2,0,3,1};
	uint8_t xRotEncCCW[4]={1,3,0,2};

	uint64_t xLastTime = 0;


	bool xDoDelta = false;
	int32_t xDoDeltaPos = 0;
};

#endif /* ENCODER_SRC_MOTORMGR_H_ */

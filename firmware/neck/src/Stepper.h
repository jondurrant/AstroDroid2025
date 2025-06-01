/*
 * Stepper.h
 *
 * Stepper driver via a Driver chip providing step and direction.
 *
 *  Created on: 11 Mar 2024
 *      Author: jondurrant
 */

#ifndef EXPS_EXPPWMDETECT_SRC_STEPPER_H_
#define EXPS_EXPPWMDETECT_SRC_STEPPER_H_

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include "hardware/clocks.h"

#ifndef GEAR_RATIO
#define GEAR_RATIO 4
#endif

enum StepperMode {
	StepperCommand,
	StepperFindZero,
	StepperStopped
};

class Stepper {
public:
	/***
	 *
	 * @param gpStep - GPIO of the Pin providing step pulses to the controller
	 * @param gpDir - GPIO of the direction Pin
	 * @param gpStepDetect - Pin being used to detect steps this will be linked the gpStep pin
	 * @param gpSlotDetect - pin used with slot detector
	 * @param invert - Invert the motor controls
	 *
	 */
	Stepper(
			uint8_t gpStep,
			uint8_t gpDir,
			uint8_t gpStepDetect,
			uint8_t gpSlotDetect,
			bool invert=false);
	virtual ~Stepper();

	/***
	 * Continuously rotate at speed
	 * @param cw - True if clockwise
	 * @param radPerSec - Speed in Radians per Second.
	 */
	void continuous(bool cw,  float radPerSec);

	/***
	 * Move by a specified number of steps
	 * @param steps - Number of steps > 0
	 * @param cw - True if clockwise rotation
	 * @param ms - Time in miliseconds to complete the rotation. If zero then default speed is used
	 */
	void moveBySteps(uint steps, bool cw, uint ms=0);

	/***
	 * Move to a specific rotation step
	 * @param step - between 0 and MaxNumberOfSteps - 1 (for a Nema 17 with no gears and Full steps this would be 200)
	 * @param cw - True if clockwise rotation to possition
	 * @param ms - Time in miliseconds to complete the rotation. if zero a default speed is used
	 */
	void moveToStep(uint step, bool cw, uint ms=0);

	/***
	 * Move to an specific angle specified in Radians
	 * @param rad - Target angle to move to in Radians
	 * @param cw - True if clockwise
	 * @param ms - Time in miliseconds to complete the rotation. if zero a default speed is used
	 */
	void moveToRad(float rad, bool cw, uint ms=0);


	/***
	 * get the Possition of the Stepper in steps
	 * @return - return number of steps: Between 0 and Max -1
	 */
	uint getPos();

	/***
	 * Get the Stepper possition in Radians
	 * @return - Radians
	 */
	float getPosRad();


	/***
	 * Stop Rotation
	 */
	void stop();


	/***
	 * Set the number of steps in a 360 rotation
	 * @param steps
	 */
	void setMaxSteps(uint steps);

	/***
	 * Get the number of steps in a 360 rotation
	 * @return
	 */
	uint getMaxSteps();


	/***
	 * Return current speed
	 * @return Radians Per Second
	 */
	float getSpeed();


	/****
	 * Is direction Clockwise
	 * @return true if clockwise
	 */
	bool isCW();


	/***
	 * Configure to Invert direction due to gear box or connection
	 * Only the functions talking Radians are effected.
	 * @param inv - True if inverted
	 */
	void configInvert(bool inv);


	/***
	 * Configure Micro Steps GPIO. Optional.
	 * @param m0
	 * @param m1
	 * @param m2
	 */
	void configMicroStepGp(uint8_t m0=255, uint8_t m1=255, uint8_t  m2=255);

	void configSleepGp(uint8_t gp=255);

	void configMaxRadPS(float radPerSec);

	void findZero(bool cw=true,  float radPerSec=1.0);

	uint calibrate();

	bool isStopped();

protected:

	/***
	 * Move by a specified number of micro steps
	 * @param micro steps - Number of micro steps > 0 (32 micro steps to a step)
	 * @param cw - True if clockwise rotation
	 * @param ms - Time in miliseconds to complete the rotation. If zero then default speed is used
	 */
	void moveByMicroSteps(uint steps, bool cw, uint ms=0);

	/***
	 * Convert step number into Radians
	 * @param steps >=0 < Max Steps
	 * @return return Radians
	 */
	virtual float stepsToRad(uint steps);

	/***
	 * Convert Radians to steps
	 * @param rad - Radians
	 * @return step
	 */
	virtual uint radToSteps(float rad);

	/***
	 * Set Speed of rotation in radians per second
	 * @param radPerSec
	 * @param microstep - should we potential use microsteps
	 */
	void setSpeed(float radPerSec, bool microstep = true);

	/***
	 * Set direction
	 * @param cw - True is Clockwise
	 */
	void setDir(bool cw);

	/***
	 * Call back to calculate the pos and handle zero detect
	 * @param gpio
	 * @param events
	 */
	static void gpioCallback (uint gpio, uint32_t events);

	/***
	 * Call back to calculate the pos and handle zero detect
	 * @param gpio
	 * @param events
	 */
	void handleGPIO(uint gpio, uint32_t events);

	/***
	 * Calculate a movement of a number of steps
	 * @param start - Starting possition
	 * @param steps - number of steps
	 * @param cw - direction - True is Clockwise
	 * @param speed - Speed in terms of the number of steps per pulse
	 * @return new step position
	 */
	virtual uint calcStep(uint start, uint steps, bool cw, uint speed=1);


	/***
	 * Set stepper fraction
	 * @param fraction: 1, 2, 4, 8, 16, 32
	 * Default to 1 if  other value
	 */
	virtual void microStep(uint8_t fraction);

	/***
	 * Set GP Levels if configured
	 * @param m0
	 * @param m1
	 * @param m2
	 */
	virtual void microStepGP(bool m0, bool m1, bool m2);

	/***
	 * Set the best microStep approach based on speed
	 * @param radPerSec
	 */
	virtual void chooseMicroStep(float radPerSec);

	void sleep(bool sleep);


private:
	static Stepper *pSelf;
	uint8_t xGpStep;
	uint8_t xGpDir;
	uint8_t xGpStepDetect;
	uint8_t xGpM0 = 255;
	uint8_t xGpM1 = 255;
	uint8_t xGpM2 = 255;
	uint8_t xGpSlotDetect;
	uint8_t xGpSleep = 255;

	bool xCW = true;
	int xPos = 0;
	int xMoveSteps = 0;
	uint xSlicer;
	uint xMaxSteps = 200 * 32 * GEAR_RATIO;
	uint xMicroStep = 32;
	uint xCalcMaxSteps = 0;

	float xSpeed = 0.0;
	bool xInvert;
	float xMaxRadPS = 0.3;

	StepperMode xMode = StepperFindZero;


};

#endif /* EXPS_EXPPWMDETECT_SRC_STEPPER_H_ */

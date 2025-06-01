/*
 * Stepper.cpp
 *
 *  Created on: 11 Mar 2024
 *      Author: jondurrant
 */

#include "Stepper.h"
#define _USE_MATH_DEFINES
#include <math.h>

Stepper * Stepper::pSelf = NULL;

Stepper::Stepper(
		uint8_t gpStep,
		uint8_t gpDir,
		uint8_t gpStepDetect,
		uint8_t gpSlotDetect,
		bool invert) {
	xGpStep = gpStep;
	xGpDir = gpDir;
	xGpStepDetect = gpStepDetect;
	xGpSlotDetect = gpSlotDetect;

	gpio_init(xGpDir);
	gpio_set_dir(xGpDir, GPIO_OUT);
	gpio_put(xGpDir, 0);

	gpio_init(xGpStep);
	gpio_set_function(xGpStep, GPIO_FUNC_PWM);
	pwm_set_gpio_level(xGpStep, 0);
	xSlicer = pwm_gpio_to_slice_num(xGpStep);

	gpio_init(xGpStepDetect);
	gpio_set_dir(xGpStepDetect, GPIO_IN);
	gpio_set_irq_enabled_with_callback(
			xGpStepDetect,
			 GPIO_IRQ_EDGE_FALL,
			true,
			Stepper::gpioCallback
		);

	gpio_init(xGpSlotDetect);
	gpio_set_dir(xGpSlotDetect, GPIO_IN);
	gpio_set_irq_enabled_with_callback(
			xGpSlotDetect,
			 GPIO_IRQ_EDGE_FALL,
			true,
			Stepper::gpioCallback
		);


	pSelf = this;

	configInvert(invert);

	if (xMode == StepperFindZero){
		setDir(true);
		setSpeed(M_PI);
	}

}

Stepper::~Stepper() {
	// TODO Auto-generated destructor stub
}


void Stepper::setSpeed(float radPerSec, bool microstep){
	uint pps;

	xMode = StepperCommand;

	if ((radPerSec <= 0.0) || (radPerSec > xMaxRadPS)){
		xSpeed = xMaxRadPS;
	}  else {
		xSpeed = radPerSec;
	}

	if (microstep){
		chooseMicroStep(xSpeed);
	} else {
		xMicroStep = 1;
		microStep(xMicroStep);
	}

	pps = (xSpeed / (2.0 * M_PI)) * (float)(xMaxSteps / xMicroStep);
	//printf("Set speed %f  or %u pps\n",
	//		radPerSec,
	//		pps);

	// Get clock speed and compute divider for 50 hz
	uint32_t clk = clock_get_hz(clk_sys);
	//uint32_t div = clk / (20000 * 50);
	uint32_t div = clk / (20000 * pps);

	// Check div is in range
	if ( div < 1 ){
		div = 1;
	}
	if ( div > 255 ){
		div = 255;
	}

	this->sleep(false);
	pwm_config config = pwm_get_default_config();
	pwm_config_set_clkdiv(&config, (float)div);

	// Set wrap so the period is 20 ms
	pwm_config_set_wrap(&config, 20000);

	// Load the configuration
	pwm_init(xSlicer, &config, false);

	pwm_set_enabled(xSlicer, true);


	pwm_set_gpio_level(xGpStep,20000/2);

}

void Stepper::stop(){
	pwm_set_gpio_level(xGpStep, 0);
	xMoveSteps = 0;
	xSpeed  = 0.0;
	xMode = StepperStopped;
	this->sleep(true);
}



void Stepper::gpioCallback (uint gpio, uint32_t events){
	if (Stepper::pSelf != NULL){
		Stepper::pSelf->handleGPIO(gpio, events);
	}
}

void Stepper::handleGPIO(uint gpio, uint32_t events){


	if (gpio == xGpStepDetect){
		if (xMoveSteps > 0){
			xMoveSteps-= xMicroStep;
			if (xMoveSteps <= 0){
				stop();
			}
		}

		//if (xMode == StepperCommand){
			xPos = calcStep(xPos, 1, xCW, xMicroStep);
		//}
	}

	if (gpio == xGpSlotDetect){

		if (xMode == StepperFindZero){
			uint p = xPos;
			if (!xCW){
				p = xMaxSteps - xPos;
			}
			if (p > xCalcMaxSteps){
				xCalcMaxSteps = p;
			}
			stop();
			//xMode = StepperCommand;
		}
		if (xPos != 0){
			//printf("xPos %u zero-ed\n", xPos);
			xPos = 0;
		}
	}

}

uint Stepper::calcStep(uint start, uint steps, bool cw, uint speed){
	uint res;

	if (cw){
		res = start + (steps * speed);
		if  (res > xMaxSteps){
			res = res % xMaxSteps;
		}
	} else {
		res = (steps * speed);
		if ( res <= start){
			res = start - res;
		} else {
			res = xMaxSteps - (res - start);
		}
	}

	return res % xMaxSteps;
}


void Stepper::moveBySteps(uint steps, bool cw, uint ms){
	moveByMicroSteps(steps * 32, cw, ms);
}

void Stepper::moveByMicroSteps(uint steps, bool cw, uint ms){
	stop();
	setDir(cw);
	if (steps > 0){
		xMoveSteps = steps;

		float rad;
		if (ms == 0 ){
			rad = 0.0;
		} else {
			rad = stepsToRad(xMoveSteps);
			rad = rad / ((float)ms / 1000.0);
		}
		//printf("rad per sec = %f\n", rad);

		setSpeed(rad);
	}
}

uint Stepper::getPos(){
	return xPos;
}


float Stepper::getPosRad(){
	return stepsToRad(xPos);
}

float Stepper::stepsToRad(uint steps){
	uint s = steps % xMaxSteps;
	float res = (float)s / (float)xMaxSteps;
	res = res * (2.0 * M_PI);
	if (xInvert){
		res = (2.0 * M_PI) - res;
	}
	return res;
}

uint Stepper::radToSteps(float rad){
	float r = rad;
	if ( r < 0.0){
		r = 0.0;
	}
	if (r > (M_PI * 2.0)){
		r = M_PI * 2.0;
	}
	if (xInvert){
		r = (2.0 * M_PI) - r;
	}
	uint res =  r  / (2.0 * M_PI) * (float) xMaxSteps/32;
	return (res % (xMaxSteps /32));
}

void Stepper::moveToStep(uint step, bool cw, uint ms){
	uint target = (step * 32)% xMaxSteps;

	uint steps;
	stop();

	if (cw){
		steps = ((target + xMaxSteps) - xPos) % xMaxSteps;
	} else {
		steps = ((xPos + xMaxSteps) - target) %xMaxSteps;
	}

	//printf("From %u to %u dir %u\n", xPos, target, cw);

	moveByMicroSteps(steps, cw, ms);
}

void Stepper::moveToRad(float rad, bool cw, uint ms){
	uint step = radToSteps(rad);
	//printf("Rad %f = Step %u\n", rad, step);
	moveToStep(step, cw, ms);
}


void Stepper::continuous(bool cw,  float radPerSec){
	stop();
	setDir(cw);
	setSpeed(radPerSec);
}

void Stepper::findZero(bool cw,  float radPerSec){
	stop();
	setDir(cw);
	setSpeed(radPerSec, false);
	xMode = StepperFindZero;
}

uint Stepper::calibrate(){
	xMaxSteps = 9000000;
	float speed = 0.015;
	int tests = 4;

	uint avgMax = 0;

	for (int i=1; i <= tests; i++){
		bool cw = i % 2;
		findZero(cw, speed);
		while (xMode != StepperStopped){
			sleep_ms(200);
		}
		//printf("Found start\n");
		findZero(cw, speed);
		while (xMode != StepperStopped){
			sleep_ms(200);
		}
		printf("Stopped max count is %u\n", xCalcMaxSteps);
		avgMax += xCalcMaxSteps;
		xCalcMaxSteps = 0;
	}
	avgMax = avgMax / tests;
	printf("Avg Max = %u\n", avgMax);
	return avgMax;
}

void Stepper::setDir(bool cw){
	if (xInvert){
		xCW = ! cw;
	} else {
		xCW = cw;
	}
	gpio_put(xGpDir, xCW);
}


void Stepper::setMaxSteps(uint steps){
	if (steps > 0){
		//Counting MicroSteps
		xMaxSteps = steps * 32;
	}
}

uint Stepper::getMaxSteps(){
	return xMaxSteps /32;
}


float Stepper::getSpeed(){
	return xSpeed;
}

bool Stepper::isCW(){
	return xCW;
}

void Stepper::configInvert(bool inv){
	xInvert = inv;
}

void Stepper::configMicroStepGp(
		uint8_t m0,
		uint8_t m1,
		uint8_t  m2){
	if (m0 <= 28 ){
		xGpM0 = m0;
		gpio_init(xGpM0);
		gpio_set_dir(xGpM0, GPIO_OUT);
		gpio_put(xGpM0, 0);
	}
	if (m1 <= 28 ){
		xGpM1 = m1;
		gpio_init(xGpM1);
		gpio_set_dir(xGpM1, GPIO_OUT);
		gpio_put(xGpM1, 0);
	}
	if (m2 <= 28 ){
		xGpM2 = m2;
		gpio_init(xGpM2);
		gpio_set_dir(xGpM2, GPIO_OUT);
		gpio_put(xGpM2, 0);
	}
}



void Stepper::microStep(uint8_t fraction){
	//No Configured MicroStepping
	if (xGpM0 > 28){
		xMicroStep = 32;
		return;
	}

	switch(fraction){
	case 1:
		xMicroStep = 32;
		microStepGP(false, false, false);
		break;
	case 2:
		xMicroStep = 16;
		microStepGP(true, false, false);
		break;
	case 4:
		xMicroStep = 8;
		if (xGpM1 > 28){
			// Minimum configurable is half step
			microStep(2);
		} else {
			microStepGP(false, true , false);
		}
		break;
	case 8:
		xMicroStep = 4;
		if (xGpM1 > 28){
			// Minimum configurable is half step
			microStep(2);
		} else {
			microStepGP(true, true , false);
		}
		break;
	case 16:
		xMicroStep = 16;
		if (xGpM2 > 28){
			// Minimum configurable is 8th
			microStep(8);
		} else {
			microStepGP(false, false , true);
		}
		break;
	case 32:
		xMicroStep = 1;
		if (xGpM2 > 28){
			// Minimum configurable is 8th
			microStep(8);
		} else {
			microStepGP(true, true , true);
		}
		break;
	default:
		xMicroStep = 32;
		microStepGP(false, false, false);
	}
}

void Stepper::microStepGP(bool m0, bool m1, bool m2){
	if (xGpM0 <= 28)
		gpio_put(xGpM0, m0);
	if (xGpM1 <= 28)
		gpio_put(xGpM1, m1);
	if (xGpM2 <= 28)
		gpio_put(xGpM2, m2);
}

void Stepper::chooseMicroStep(float radPerSec){
	if (radPerSec > 1.8){
		microStep(1);
	} else if (radPerSec > 1.0){
		microStep(2);
	} if (radPerSec > 0.6){
		microStep(4);
	} else {
		microStep(8);
	}
}


bool Stepper::isStopped(){
	return xMode == StepperStopped;
}

void Stepper::configMaxRadPS(float radPerSec){
	xMaxRadPS = radPerSec;
}


void Stepper::configSleepGp(uint8_t gp){
	xGpSleep = gp;
	gpio_init(xGpSleep);
	gpio_set_dir(xGpSleep, GPIO_OUT);
	sleep(true);
}

void Stepper::sleep(bool sleep){
	if (xGpSleep <= 28){
		gpio_put(xGpSleep, !sleep);
	}
}

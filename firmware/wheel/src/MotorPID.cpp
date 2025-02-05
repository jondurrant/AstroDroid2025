/*
 * MotorPID.cpp
 *
 *  Created on: 29 May 2023
 *      Author: jondurrant
 */

#include "MotorPID.h"
#include <cmath>

MotorPID::MotorPID(uint8_t gpCW, uint8_t gpCCW,
		uint8_t gpA, uint8_t gpB) :
		MotorMgr(gpCW, gpCCW, gpA, gpB){
	// NOP

}

MotorPID::~MotorPID() {
	// TODO Auto-generated destructor stub
}



void MotorPID::setSpeedRadPS(float rps, bool cw){
	xTargetRadPS = rps;
	xCW = cw;
	xCumErr = 0.0;
	xLastErr = 0.0;
}

void MotorPID::handleRotate(bool cw){
	MotorMgr::handleRotate(cw);
	//doPID();
}

void MotorPID::configPID(float kP, float kI, float kD){
	xKP = kP;
	xKI = kI;
	xKD = kD;
}


float MotorPID::doPID(float *pP, float *pI, float *pD){

	// Throttle 1.0 = 320RPM
	// Throttle 0.2 = 40RPM
	float error ;
	float sp, pv;
	float p, i, d;

	float *lpP = &p;
	float *lpI = &i;
	float *lpD = &d;

	if (pP != NULL){
		lpP = pP;
	}
	if (pI != NULL){
		lpI = pI;
	}
	if (pD != NULL){
		lpD = pD;
	}

	float pid = this->pid(sp, pv, error, *lpP, *lpI, *lpD);
	xCumErr += error;
	xLastErr = error;


	//printf("PID err: %0.2f p: %0.2f\n", error, p);


	//float delt = pid / 320.0;
	float delt = pid ;
	float t = getThrottle() + delt;
	//printf("d=%0.2f t=%0.2f\n", delt, t);
	if (t < 0.0){
		t = 0.0;
	}

	setThrottle(t, xCW);
	return t;
}

float MotorPID::pid (float &sp, float &pv, float &err,
		float &p, float &i, float &d){
	sp = xTargetRadPS;
	pv = getRadPerSec();
	err = sp - pv;
	p = err * xKP;

	//xCumErr += err;

	i = xKI * (xCumErr + err);


	d = xKD * (err - xLastErr);

	return p + i + d;

}

void MotorPID::getKPID(float &kp, float &ki, float &kd){
	kp = xKP;
	ki = xKI;
	kd = xKD;
}

float MotorPID::getTargetSpeedRadPS(){
	return xTargetRadPS;
}

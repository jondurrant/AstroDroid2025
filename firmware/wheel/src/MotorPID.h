/*
 * MotorPID.h
 *
 *  Created on: 29 May 2023
 *      Author: jondurrant
 */

#ifndef ENCODER_SRC_MOTORPID_H_
#define ENCODER_SRC_MOTORPID_H_

#include "MotorMgr.h"

class MotorPID : public MotorMgr {
public:
	MotorPID(uint8_t gpCW, uint8_t gpCCW,
			uint8_t gpA, uint8_t gpB);
	virtual ~MotorPID();


	void setSpeedRadPS(float rps, bool cw);
	float getTargetSpeedRadPS();

	void setDeltaRadPS(float deltaRad, float rps, bool cw);

	void configPID(float kP, float kI, float kD);

	float pid (float &sp, float &pv, float &err,
			float &p, float &i, float &d);

    float doPID(float *pP = NULL, float *pI = NULL, float *pD=NULL);

    void getKPID(float &kp, float &ki, float &kd);
protected:

	virtual void handleRotate(bool cw);

	virtual void targetStop();



	float xTargetRadPS = 0.0;



private:

	float xPrevError;
	float xKP = 1;
	float xKI = 1;
	float xKD = 1;
	float xCumErr = 0.0;
	float xLastErr = 0.0;

};

#endif /* ENCODER_SRC_MOTORPID_H_ */

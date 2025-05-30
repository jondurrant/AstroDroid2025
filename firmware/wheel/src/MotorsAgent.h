/*
 * MotorsAgent.h
 *
 *  Created on: 6 Aug 2023
 *      Author: jondurrant
 */

#ifndef FIRMWARE_SRC_MOTORSAGENT_H_
#define FIRMWARE_SRC_MOTORSAGENT_H_

#include "pico/stdlib.h"
#include "Agent.h"
#include "MotorPID.h"
#include "uRosEntities.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <control_msgs/msg/pid_state.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <control_msgs/msg/joint_jog.h>
}

#ifndef NUM_MOTORS
#define NUM_MOTORS 1
#endif

#define JOINT_TOPIC "/Astro/joint_state"
#define VELOCITY_TOPIC "/velocity"
#define PID_TOPIC "/pid"
#define JOINT_JOG_TOPIC "/Astro/wheels_jog"
#define MOTOR_LEFT "LEFT"
#define MOTOR_RIGHT "RIGHT"


class MotorsAgent : public Agent, public uRosEntities {
public:
	MotorsAgent();
	virtual ~MotorsAgent();

	/***
	 * Add Motor
	 * @param index - Index of the Motor
	 * @param gpCW - CW power terminal on Motor
	 * @param gpCCW - CCW power terminal on Motor
	 * @param gpA - RotEnc input A
	 * @param gpB - RotEnc input B
	 */
	void addMotor(uint index,
			uint8_t gpCW, uint8_t gpCCW,
			uint8_t gpA, uint8_t gpB);

	/***
	 * Return specific motor or NULL if none
	 * @param index
	 * @return
	 */
	MotorPID * getMotor(uint index);

	/***
	 * Configure PID for motor
	 * @param index - of the motor
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	void configPID(uint index,
			float kP, float kI, float kD);

	/***
	 * Configure PID for all the motors
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	void configAllPID(float kP, float kI, float kD);

	/***
	 * Set the speed of the motor to be controlled
	 * @param index of the motor
	 * @param rpm rev per minute
	 * @param cw direction - true if clockwise
	 */
	void setSpeedRPM(uint index,
			float rpm, bool cw);

	/***
	 * Set the speed of the motor to be controlled
	 * @param index of the motor
	 * @param rps radians per second
	 * @param cw direction - true if clockwise
	 */
	void setSpeedRadPS(uint index, float rps, bool cw);

	/***
	 * Turn to delta possition at speed
	 * @param index of the motor
	 * @param deltaRad radians to turn
	 * @param rps radians per second
	 * @param cw  direction - true if clockwise
	 */
	void setDeltaRadPS(uint index, float deltaRad, float rps, bool cw);

	/***
	 * Create the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Destroy the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Provide a count of the number of entities
	 * @return number of entities >=0
	 */
	virtual uint getCount();

	/***
	 * Return the number of handles needed by the executor
	 * @return
	 */
	virtual uint getHandles();


	/***
	 * Add subscribers, guards and timers to the executor
	 * @param executor
	 */
	virtual void addToExecutor(rclc_executor_t *executor);

protected:

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

	/***
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* context);


private:

	void initJointState();
	void pubJointState();

	void initPidState();
	void pubPidState(
			float err,
			float perr,
			float ierr,
			float derr,
			float kp,
			float ki,
			float kd,
			float velocity
			);

	void initJointJog();


	MotorPID *pMotors[NUM_MOTORS];
	rosidl_runtime_c__String xMotorsName[NUM_MOTORS];

	rcl_publisher_t xPubJoint;
	sensor_msgs__msg__JointState xJointStateMsg;
	char * pJointTopic = NULL;

	rcl_publisher_t xPubPid;
	control_msgs__msg__PidState xPidStateMsg;
	char * pPidTopic = NULL;

	rcl_subscription_t 					xSubVelocity;
	uRosSubContext_t   				xSubVelocityContext;
	std_msgs__msg__Float32 	xVelocityMsg;
	char * pVelocityTopic = NULL;

	rcl_subscription_t 					xSubJointJog;
	uRosSubContext_t   				xSubJointJogContext;
	control_msgs__msg__JointJog 	xJointJogMsg;
	char * pJointJogTopic = NULL;


	bool xIsLeft = false;

};

#endif /* FIRMWARE_SRC_MOTORSAGENT_H_ */

/*
 * MotorsAgent.cpp
 *
 *  Created on: 6 Aug 2023
 *      Author: jondurrant
 */

#include "MotorsAgent.h"

#include "uRosBridge.h"

#include <inttypes.h>
#include <cmath>
#include "ConfigEntity.h"
#include "NVSJson.h"


MotorsAgent::MotorsAgent() {
	for (uint i=0; i < NUM_MOTORS; i++){
		pMotors[i] = NULL;
	}
}

MotorsAgent::~MotorsAgent() {
	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL){
			delete pMotors[i];
		}
	}
}

/***
 * Add Motor
 * @param index - Index of the Motor
 * @param gpCW - CW power terminal on Motor
 * @param gpCCW - CCW power terminal on Motor
 * @param gpA - RotEnc input A
 * @param gpB - RotEnc input B
 */
void MotorsAgent::addMotor(uint index,
		uint8_t gpCW, uint8_t gpCCW,
		uint8_t gpA, uint8_t gpB){
	if (index < NUM_MOTORS){
		pMotors[index] = new MotorPID(gpCW, gpCCW, gpA, gpB);
	}
}

/***
 * Configure PID for motor
 * @param index - of the motor
 * @param kP
 * @param kI
 * @param kD
 */
void MotorsAgent::configPID(uint index,
		float kP, float kI, float kD){
	if (pMotors[index] != NULL){
		pMotors[index]->configPID(kP, kI, kD);
	}
}

/***
 * Configure PID for all the motors
 * @param kP
 * @param kI
 * @param kD
 */
void MotorsAgent::configAllPID(float kP, float kI, float kD){
	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL){
			pMotors[i]->configPID(kP, kI, kD);
		}
	}
}


/***
 * Set the speed of the motor to be controlled
 * @param index of the motor
 * @param rpm rev per minute
 * @param cw direction - true if clockwise
 */
void MotorsAgent::setSpeedRPM(uint index,
		float rpm, bool cw){
	setSpeedRadPS(
			index,
			rpm* 0.10472,
			cw);
}

/***
 * Set the speed of the motor to be controlled
 * @param index of the motor
 * @param rps radians per second
 * @param cw direction - true if clockwise
 */
void MotorsAgent::setSpeedRadPS(uint index,
		float rps, bool cw){
	if (pMotors[index] != NULL){
		if (rps >= 0.0){
			pMotors[index]->setSpeedRadPS(rps, cw);
		} else {
			pMotors[index]->setSpeedRadPS(fabs(rps), !cw);
		}
	}
}

/***
 * Run loop for the agent.
 */
void MotorsAgent::run(){

	float perr, ierr, derr, err, vel;
	float kp, ki, kd;

	initJointState();
	initPidState();

	initJointJog();

	for (;;){
		for (uint i=0; i < NUM_MOTORS; i++){
			if (pMotors[i] != NULL){
				float pid = pMotors[i]->doPID(&perr, &ierr, &derr);

				pMotors[i]->getKPID(kp, ki, kd);
				vel = pMotors[i]->getAvgRadPerSec();

				float err = pMotors[i]->getTargetSpeedRadPS() -
						pMotors[i]->getAvgRadPerSec();

				pubPidState(
						err,
						perr,
						ierr,
						derr,
						kp,
						ki,
						kd,
						vel
				);

			}
		}

		pubJointState();

		vTaskDelay(50);
	}
}


/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE MotorsAgent::getMaxStackSize(){
	return 1024;
}


/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void MotorsAgent::createEntities(
		rcl_node_t *node,
		rclc_support_t *support){

	//Joint State
	if (pJointTopic != NULL){
		vPortFree(pJointTopic);
	}
	NVSJson * nvs = NVSJson::getInstance();
	size_t topicLen = nvs->size(TOPIC_PREFIX) + strlen(JOINT_TOPIC);
	pJointTopic = (char *)pvPortMalloc(topicLen);
	if (pJointTopic != NULL){
		if (nvs->get_str(TOPIC_PREFIX, pJointTopic, &topicLen) == NVS_OK){
			strcpy(&pJointTopic[strlen(pJointTopic)], JOINT_TOPIC);

				rclc_publisher_init_default(
					&xPubJoint,
					node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
					pJointTopic);
		} else {
			printf("ERROR: MotorAgent can't read TOPIC Prefix from NVR\n");
		}
	} else {
		printf("ERROR: MotorAgent Malloc failed\n");
	}

	//Velocity
	if (pVelocityTopic != NULL) {
			vPortFree(pVelocityTopic);
	}
	size_t velocityLen = nvs->size(TOPIC_PREFIX) + strlen(VELOCITY_TOPIC);
	pVelocityTopic = (char *)pvPortMalloc(velocityLen);
	if ( pVelocityTopic != NULL){
		if (nvs->get_str(TOPIC_PREFIX, pVelocityTopic, &velocityLen) == NVS_OK){
					strcpy(&pVelocityTopic[strlen(pVelocityTopic)], VELOCITY_TOPIC);

					int res = rclc_subscription_init_default(
							  &xSubVelocity,
							  node,
							 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
							 pVelocityTopic
							  );

					if (res != RCL_RET_OK){
						printf("ERROR: MotorAgent::createEntities failed to init subscriber \n");
					}
		} else {
					printf("ERROR:MotorAgent  NVS does not contain TOPIC_PREFIX\n");
		}
	} else {
		printf("ERROR: MotorAgent Malloc failed\n");
	}

	//Pid State
	if (pPidTopic != NULL){
		vPortFree(pPidTopic);
	}
	topicLen = nvs->size(TOPIC_PREFIX) + strlen(PID_TOPIC);
	pPidTopic = (char *)pvPortMalloc(topicLen);
	if (pPidTopic != NULL){
		if (nvs->get_str(TOPIC_PREFIX, pPidTopic, &topicLen) == NVS_OK){
			strcpy(&pPidTopic[strlen(pPidTopic)], PID_TOPIC);

				rclc_publisher_init_default(
					&xPubPid,
					node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, PidState),
					pPidTopic);
		} else {
			printf("ERROR: MotorAgent can't read TOPIC Prefix from NVR\n");
		}
	} else {
		printf("ERROR: MotorAgent Malloc failed\n");
	}

	//JointJog
	int res = rclc_subscription_init_default(
				  &xSubJointJog,
				  node,
				 ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
				 JOINT_JOG_TOPIC
				  );
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void MotorsAgent::destroyEntities(
		rcl_node_t *node,
		rclc_support_t *support){

	rcl_publisher_fini(&xPubJoint, node);
	rcl_publisher_fini(&xPubPid, node);
	rcl_subscription_fini(&xSubVelocity, 	node);
	rcl_subscription_fini(&xSubJointJog, node);
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint MotorsAgent::getCount(){
	return 2;
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint MotorsAgent::getHandles(){
	return 4;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void MotorsAgent::addToExecutor(rclc_executor_t *executor){
	buildContext(&xSubVelocityContext, NULL);
	rcl_ret_t res = rclc_executor_add_subscription_with_context(
			executor,
			&xSubVelocity,
			&xVelocityMsg,
			uRosEntities::subscriptionCallback,
			&xSubVelocityContext,
			ON_NEW_DATA);
	if (res != RCL_RET_OK){
		printf("ERROR: MotorsAgent::addToExecutor failed to add executor \n");
	}

	buildContext(&xSubJointJogContext, NULL);
	res = rclc_executor_add_subscription_with_context(
				executor,
				&xSubJointJog,
				&xJointJogMsg,
				uRosEntities::subscriptionCallback,
				&xSubJointJogContext,
				ON_NEW_DATA);
		if (res != RCL_RET_OK){
			printf("ERROR: MotorsAgent::addToExecutor failed to add executor \n");
		}
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void MotorsAgent::handleSubscriptionMsg(
		const void* msg,
		uRosSubContext_t* context){

	if (context == &xSubVelocityContext){
			std_msgs__msg__Float32 * pFloatMsg = (std_msgs__msg__Float32 *) msg;

			printf("Velocity is %f\n", pFloatMsg->data);
			setSpeedRadPS(0, pFloatMsg->data, true);
	}

	if (context == &xSubJointJogContext){
		control_msgs__msg__JointJog * pJointJogMsg = (control_msgs__msg__JointJog *) msg;
		for (int i=0; i < pJointJogMsg->joint_names.size; i++){
			for (int j=0; j < NUM_MOTORS; j++){
				if (rosidl_runtime_c__String__are_equal(
							&xMotorsName[j],
							&pJointJogMsg->joint_names.data[i])
						){

					float vel = pJointJogMsg->velocities.data[i];
					if (xIsLeft){
						vel = vel * (-1.0);
					}
					if (pJointJogMsg->displacements.size == 0){
						//Velocity msg
						printf("Velocity(%d) is %f\n",
								i,
								pJointJogMsg->velocities.data[i]);
						setSpeedRadPS(
								0,
								vel,
								true);
					} else {
						//Displacement msg
						printf("Displacement(%d) to %.2f at %.2f\n",
								i,
								pJointJogMsg->displacements.data[i],
								vel);
						setDeltaRadPS(
								0,
								pJointJogMsg->displacements.data[i],
								vel,
								true);
					}
				} // if Name
			}// for motors
		} // for joing names in msg
	}// if Context
}


void MotorsAgent::initJointState(){
	sensor_msgs__msg__JointState__init(&xJointStateMsg);
	char name[32];

	//Possition
	rosidl_runtime_c__double__Sequence__init(
			&xJointStateMsg.position, NUM_MOTORS);
	xJointStateMsg.position.data[0] = 0.0;
	xJointStateMsg.position.size = NUM_MOTORS;
	xJointStateMsg.position.capacity = NUM_MOTORS;

	//Velocity
	rosidl_runtime_c__double__Sequence__init(
			&xJointStateMsg.velocity, NUM_MOTORS);
	xJointStateMsg.velocity.data[0] = 0.0;
	xJointStateMsg.velocity.size = NUM_MOTORS;
	xJointStateMsg.velocity.capacity = NUM_MOTORS;

	//Name
	rosidl_runtime_c__String__Sequence__init(
			&xJointStateMsg.name, NUM_MOTORS);
	for (uint i=0; i < NUM_MOTORS; i++){
		sprintf(name, "motor_%u", i);
		if (!rosidl_runtime_c__String__assign(
				&xJointStateMsg.name.data[i], name)){
			printf("ERROR: Joined assignment failed\n");
		}
	}
	xJointStateMsg.name.size=NUM_MOTORS;
	xJointStateMsg.name.capacity=NUM_MOTORS;
}


void MotorsAgent::pubJointState(){
	//Populate the Joint possition message
	int64_t time = rmw_uros_epoch_nanos();

	xJointStateMsg.header.stamp.sec = time / 1000000000;
	xJointStateMsg.header.stamp.nanosec = time % 1000000000;

	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL) {
			xJointStateMsg.position.data[i] =
					pMotors[i]->getRadians() - M_PI;

			xJointStateMsg.velocity.data[i] =
					pMotors[i]->getAvgRadPerSec();
		}
	}
	if (!uRosBridge::getInstance()->publish(&xPubJoint,
			&xJointStateMsg,
			this,
			NULL)){
		printf("Joint Pub failed\n");
	}
}

/***
 * Return specific motor or NULL if none
 * @param index
 * @return
 */
MotorPID * MotorsAgent::getMotor(uint index){
	if (index >= NUM_MOTORS){
		return NULL;
	}
	return pMotors[index];
}

void MotorsAgent::initPidState(){
	control_msgs__msg__PidState__init(&xPidStateMsg);
	xPidStateMsg.error = 0.0;
	xPidStateMsg.error_dot = 0.0;
	xPidStateMsg.p_error = 0.0;
	xPidStateMsg.i_error = 0.0;
	xPidStateMsg.d_error = 0.0;
	xPidStateMsg.p_term = 0.0;
	xPidStateMsg.i_term = 0.0;
	xPidStateMsg.d_term = 0.0;
	xPidStateMsg.i_max = 0.0;
	xPidStateMsg.i_min = 0.0;
	xPidStateMsg.output = 0.0;
}

void MotorsAgent::pubPidState(
		float err,
		float perr,
		float ierr,
		float derr,
		float kp,
		float ki,
		float kd,
		float velocity
		){
	//Populate the PID  message
	int64_t time = rmw_uros_epoch_nanos();
	xPidStateMsg.header.stamp.sec = time / 1000000000;
	xPidStateMsg.header.stamp.nanosec = time % 1000000000;

	xPidStateMsg.timestep.sec  = xPidStateMsg.header.stamp.sec ;
	xPidStateMsg.timestep.nanosec = xPidStateMsg.header.stamp.nanosec;
	xPidStateMsg.error = err;
	xPidStateMsg.p_error = perr;
	xPidStateMsg.i_error = ierr;
	xPidStateMsg.d_error = derr;
	xPidStateMsg.p_term = kp;
	xPidStateMsg.i_term = ki;
	xPidStateMsg.d_term = kd;
	xPidStateMsg.output = velocity;


	if (!uRosBridge::getInstance()->publish(&xPubPid,
			&xPidStateMsg,
			this,
			NULL)){
		printf("PID Pub failed\n");
	}
}

void MotorsAgent::initJointJog(){
	int jointCount = 2;
	control_msgs__msg__JointJog__init(&xJointJogMsg);
	rosidl_runtime_c__String__Sequence__init(&xJointJogMsg.joint_names, jointCount);
	xJointJogMsg.joint_names.size=jointCount;
	xJointJogMsg.joint_names.capacity=jointCount;
	for (int i=0; i < jointCount; i++){
		rosidl_runtime_c__String__assign(&xJointJogMsg.joint_names.data[i], "wheel");
	}
	rosidl_runtime_c__double__Sequence__init(&xJointJogMsg.displacements, jointCount);
	xJointJogMsg.displacements.size=jointCount;
	xJointJogMsg.displacements.capacity=jointCount;
	rosidl_runtime_c__double__Sequence__init(&xJointJogMsg.velocities, jointCount);
	xJointJogMsg.velocities.size=jointCount;
	xJointJogMsg.velocities.capacity=jointCount;

	//Motor name initialisation
	rosidl_runtime_c__String__init(&xMotorsName[0]);
	NVSJson *nvs = NVSJson::getInstance();
	nvs->get_bool(CONFIG_LEFT,  &xIsLeft);
	if (xIsLeft){
		rosidl_runtime_c__String__assign(
				&xMotorsName[0],
				MOTOR_LEFT
				);
		printf("I am %s\n",MOTOR_LEFT);
	} else {
		rosidl_runtime_c__String__assign(
				&xMotorsName[0],
				MOTOR_RIGHT
				);
		printf("I am %s\n",MOTOR_RIGHT);
	}
}

void MotorsAgent::setDeltaRadPS(uint index, float deltaRad, float rps, bool cw){
	if (pMotors[index] != NULL){
		if (rps >= 0.0){
			pMotors[index]->setDeltaRadPS(fabs(deltaRad), rps, cw);
		} else {
			pMotors[index]->setDeltaRadPS(fabs(deltaRad), fabs(rps), !cw);
		}
	}
}


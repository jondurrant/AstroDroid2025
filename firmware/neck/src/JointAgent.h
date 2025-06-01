/*
 * JointAgent.h
 *
 *  Created on: 31 May 2025
 *      Author: jondurrant
 */

#ifndef FIRMWARE_NECK_SRC_JOINTAGENT_H_
#define FIRMWARE_NECK_SRC_JOINTAGENT_H_

#include "pico/stdlib.h"
#include "Agent.h"
#include "uRosEntities.h"
#include "Stepper.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <control_msgs/msg/joint_jog.h>
}

#ifndef JOINT_TOPIC
#define JOINT_TOPIC "/Astro/joint_state"
#endif
#ifndef JOINT_JOG_TOPIC
#define JOINT_JOG_TOPIC "/Astro/body_jog"
#endif

#define NUM_JOINTS 2
#define JOINT_NAME_DOME "DomeZ"
#define JOINT_NAME_BODY "BodyY"

class JointAgent : public Agent, public uRosEntities {
public:
	JointAgent();
	virtual ~JointAgent();


	void addStepper(Stepper *s);

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
	void initJointJog();

	rcl_publisher_t xPubJoint;
	sensor_msgs__msg__JointState xJointStateMsg;

	rcl_subscription_t 					xSubJointJog;
	uRosSubContext_t   				xSubJointJogContext;
	control_msgs__msg__JointJog 	xJointJogMsg;

	Stepper *pStepper = NULL;

	rosidl_runtime_c__String xJointNameDome;
	rosidl_runtime_c__String xJointNameBody;

};

#endif /* FIRMWARE_NECK_SRC_JOINTAGENT_H_ */

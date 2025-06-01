/*
 * JointAgent.cpp
 *
 *  Created on: 31 May 2025
 *      Author: jondurrant
 */

#include "JointAgent.h"
#include "uRosBridge.h"

#include <inttypes.h>
#include <cmath>
#include "ConfigEntity.h"
#include "NVSJson.h"

JointAgent::JointAgent() {
	// TODO Auto-generated constructor stub

}

JointAgent::~JointAgent() {
	// TODO Auto-generated destructor stub
}

void JointAgent::addStepper(Stepper *s){
	pStepper = s;
}


void JointAgent::createEntities(rcl_node_t *node, rclc_support_t *support){
	rclc_publisher_init_default(
				&xPubJoint,
				node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
				JOINT_TOPIC);

	//JointJog
	int res = rclc_subscription_init_default(
				  &xSubJointJog,
				  node,
				 ROSIDL_GET_MSG_TYPE_SUPPORT(control_msgs, msg, JointJog),
				 JOINT_JOG_TOPIC
				  );
}

void JointAgent::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	rcl_publisher_fini(&xPubJoint, node);
	rcl_subscription_fini(&xSubJointJog, node);
}


uint JointAgent::getCount(){
	return 2;
}

uint JointAgent::getHandles(){
	return 1;
}


void JointAgent::addToExecutor(rclc_executor_t *executor){
	buildContext(&xSubJointJogContext, NULL);
	int res = rclc_executor_add_subscription_with_context(
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

void JointAgent::handleSubscriptionMsg(const void* msg, uRosSubContext_t* context){
	if (context == &xSubJointJogContext){
			control_msgs__msg__JointJog * pJointJogMsg = (control_msgs__msg__JointJog *) msg;
			for (int i=0; i < pJointJogMsg->joint_names.size; i++){
				if (rosidl_runtime_c__String__are_equal(
						&xJointNameDome,
						&pJointJogMsg->joint_names.data[i])
						){

						if (pJointJogMsg->velocities.size > i ){
							float vel = pJointJogMsg->velocities.data[i];
							bool cw = (vel >= 0.0);
							if (!cw){
								vel = vel * -1;
							}

							if (pStepper != NULL){
								pStepper->continuous(cw,   vel);
							}

						} else if (pJointJogMsg->displacements.size > i){
							bool cw = true;
							uint ms = (uint) (pJointJogMsg->duration * 1000.0);
							float dis = pJointJogMsg->displacements.data[i];
							if (dis <= (M_PI * 2.0)){
								if (pStepper != NULL){
									float c = pStepper->getPosRad();
									if (dis > c){
										if ((dis-c) > M_PI){
											cw = false;
										} else {
											cw = true;
										}
									} else {
										if ((c-dis) > M_PI){
											cw = true;
										} else {
											cw = false;
										}
									}
									pStepper->moveToRad( dis,  cw,  ms);
								}
							}


						}
				}

			} // for joing names in msg
		}// if Context
}

void JointAgent::initJointState(){
	sensor_msgs__msg__JointState__init(&xJointStateMsg);
	char name[32];

	//Possition
	rosidl_runtime_c__double__Sequence__init(
			&xJointStateMsg.position, NUM_JOINTS);
	xJointStateMsg.position.data[0] = 0.0;
	xJointStateMsg.position.size = NUM_JOINTS;
	xJointStateMsg.position.capacity = NUM_JOINTS;

	//Velocity
	rosidl_runtime_c__double__Sequence__init(
			&xJointStateMsg.velocity, NUM_JOINTS);
	xJointStateMsg.velocity.data[0] = 0.0;
	xJointStateMsg.velocity.size = NUM_JOINTS;
	xJointStateMsg.velocity.capacity = NUM_JOINTS;

	//Name
	rosidl_runtime_c__String__Sequence__init(
			&xJointStateMsg.name, NUM_JOINTS);
	if (!rosidl_runtime_c__String__assign(
			&xJointStateMsg.name.data[0], JOINT_NAME_DOME)){
		printf("ERROR: Joined assignment failed\n");
	}
	if (!rosidl_runtime_c__String__assign(
			&xJointStateMsg.name.data[1], JOINT_NAME_BODY)){
		printf("ERROR: Joined assignment failed\n");
	}


	xJointStateMsg.name.size=NUM_JOINTS;
	xJointStateMsg.name.capacity=NUM_JOINTS;

	//Joint name initialisation
	rosidl_runtime_c__String__init(&xJointNameDome);
	rosidl_runtime_c__String__assign(
			&xJointNameDome,
			JOINT_NAME_DOME
			);

	rosidl_runtime_c__String__init(&xJointNameBody);
	rosidl_runtime_c__String__assign(
			&xJointNameBody,
			JOINT_NAME_BODY
			);

}

void JointAgent:: pubJointState(){
	//Populate the Joint possition message
	int64_t time = rmw_uros_epoch_nanos();

	xJointStateMsg.header.stamp.sec = time / 1000000000;
	xJointStateMsg.header.stamp.nanosec = time % 1000000000;

	// Stepper
	if (pStepper != NULL){
			xJointStateMsg.position.data[0] =
					pStepper->getPosRad();

			float v = 	pStepper->getSpeed();
			if (!pStepper->isCW()){
				v = v * -1;
			}
			xJointStateMsg.velocity.data[0] = v;
	}


	if (!uRosBridge::getInstance()->publish(&xPubJoint,
			&xJointStateMsg,
			this,
			NULL)){
		printf("Joint Pub failed\n");
	}
}



void JointAgent::initJointJog(){
	int jointCount = NUM_JOINTS;
	control_msgs__msg__JointJog__init(&xJointJogMsg);
	rosidl_runtime_c__String__Sequence__init(&xJointJogMsg.joint_names, jointCount);
	xJointJogMsg.joint_names.size=jointCount;
	xJointJogMsg.joint_names.capacity=jointCount;
	rosidl_runtime_c__String__assign(&xJointJogMsg.joint_names.data[0], JOINT_NAME_DOME);
	rosidl_runtime_c__String__assign(&xJointJogMsg.joint_names.data[1], JOINT_NAME_BODY);

	rosidl_runtime_c__double__Sequence__init(&xJointJogMsg.displacements, jointCount);
	xJointJogMsg.displacements.size=jointCount;
	xJointJogMsg.displacements.capacity=jointCount;
	rosidl_runtime_c__double__Sequence__init(&xJointJogMsg.velocities, jointCount);
	xJointJogMsg.velocities.size=jointCount;
	xJointJogMsg.velocities.capacity=jointCount;
}


void JointAgent::run(){
	initJointJog();
	initJointState();



	for (;;){
		pubJointState();

		vTaskDelay(200);
	}
}

configSTACK_DEPTH_TYPE JointAgent::getMaxStackSize(){
	return 1024;
}




/*
 * ConfigEntity.cpp
 *
 *  Created on: 26 Jan 2025
 *      Author: jondurrant
 */

#include "ConfigEntity.h"

ConfigEntity::ConfigEntity() {
	initConfigMsg();

}

ConfigEntity::~ConfigEntity() {
	// TODO Auto-generated destructor stub
}

/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void ConfigEntity::createEntities(rcl_node_t *node, rclc_support_t *support){

	rclc_publisher_init_default(
			&xPubConfig,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
			"/astro/config");

	rclc_subscription_init_default(
			  &xSubConfig,
			  node,
			  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
			  "/astro/set_config");
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void ConfigEntity::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	rcl_subscription_fini(&xSubConfig, 	node);
	rcl_publisher_fini(&xPubConfig, node);
}


/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint ConfigEntity::getCount(){
	return 2;
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint ConfigEntity::getHandles(){
	uint res = 1;
	return res;
}

/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void ConfigEntity::addToExecutor(rclc_executor_t *executor){

	buildContext(&xSubConfigContext, NULL);
	rclc_executor_add_subscription_with_context(
			executor,
			&xSubConfig,
			&xConfigMsg,
			uRosEntities::subscriptionCallback,
			&xSubConfigContext,
			ON_NEW_DATA);
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void ConfigEntity::handleSubscriptionMsg(
		const void* msg,
		uRosSubContext_t* context){

	if (context == &xSubConfigContext){
		std_msgs__msg__String * pStringMsg = (std_msgs__msg__String *) msg;

		printf("Config Received: %s\n", pStringMsg->data);
	}
}

void ConfigEntity::initConfigMsg(){
	std_msgs__msg__String__init(&xConfigMsg);
}


void ConfigEntity::pubConfigMsg(){

}






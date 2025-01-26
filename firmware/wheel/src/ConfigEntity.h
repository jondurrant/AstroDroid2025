/*
 * ConfigEntity.h
 *
 *  Created on: 26 Jan 2025
 *      Author: jondurrant
 */

#ifndef FIRMWARE_WHEEL_SRC_CONFIGENTITY_H_
#define FIRMWARE_WHEEL_SRC_CONFIGENTITY_H_

#include "uRosEntities.h"

extern "C"{
#include <std_msgs/msg/string.h>
}

class ConfigEntity : public  uRosEntities {
public:
	ConfigEntity();
	virtual ~ConfigEntity();

	/***
	 * Create the entities (Publishers)
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Destroy the entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Return the number of entities
	 * @return
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
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* Context);

private:
	void initConfigMsg();
	void pubConfigMsg();

	rcl_publisher_t 					xPubConfig;
	rcl_subscription_t 			xSubConfig;
	uRosSubContext_t   		xSubConfigContext;
	std_msgs__msg__String xConfigMsg;


};

#endif /* FIRMWARE_WHEEL_SRC_CONFIGENTITY_H_ */

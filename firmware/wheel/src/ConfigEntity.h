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
#include <std_msgs/msg/header.h>
}


#define ROS2_NODE "ROS2NODE"
#define TOPIC_PREFIX "TOPIC_PREFIX"
#define FACTORY_RESET "FACTORY_RESET"
#define REBOOT "REBOOT"
#define CONFIG_TOPIC "/config"
#define CONFIG_SET_TOPIC "/set_config"
#define CONFIG_BUFFER_SIZE 1024
#define CONFIG_MAX_KEYS 12

#define CONFIG_PID_KP "PID_KP"
#define CONFIG_PID_KL "PID_KL"
#define CONFIG_PID_KD "PID_KD"



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


	virtual void setNeededDefaults();
	virtual void factoryReset();
	virtual void reboot();

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
	//std_msgs__msg__Header xConfigMsg;


	char xBuffer[CONFIG_BUFFER_SIZE];

	char * pConfigTopic = NULL;
	char * pSetConfigTopic = NULL;


};

#endif /* FIRMWARE_WHEEL_SRC_CONFIGENTITY_H_ */

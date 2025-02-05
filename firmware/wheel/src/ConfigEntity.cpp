/*
 * ConfigEntity.cpp
 *
 *  Created on: 26 Jan 2025
 *      Author: jondurrant
 */

#include "ConfigEntity.h"
#include "tiny-json.h"
#include "NVSJson.h"
#include "uRosBridge.h"
#include "pico/unique_id.h"
#include "hardware/watchdog.h"

ConfigEntity::ConfigEntity() {
	setNeededDefaults();
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
	rcl_ret_t res;

	NVSJson * nvs = NVSJson::getInstance();

	if (pConfigTopic != NULL) {
		vPortFree(pConfigTopic);
	}
	size_t configLen = nvs->size(TOPIC_PREFIX) + strlen(CONFIG_TOPIC);
	pConfigTopic = (char *)pvPortMalloc(configLen);
	if ( pConfigTopic != NULL){
		if (nvs->get_str(TOPIC_PREFIX, pConfigTopic, &configLen) == NVS_OK){
			strcpy(&pConfigTopic[strlen(pConfigTopic)], CONFIG_TOPIC);
			res = rclc_publisher_init_default(
					&xPubConfig,
					node,
					ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
					pConfigTopic);

			if (res != RCL_RET_OK){
				printf("ERROR: ConfigEntity::createEntities failed to init publisher \n");
			}
		} else {
			printf("ERROR: NVS does not contain TOPIC_PREFIX\n");
		}
	} else {
		printf("ERROR: Malloc failed\n");
	}

	if (pSetConfigTopic != NULL) {
		vPortFree(pSetConfigTopic);
	}
	configLen = nvs->size(TOPIC_PREFIX) + strlen(CONFIG_SET_TOPIC);
	pSetConfigTopic = (char *)pvPortMalloc(configLen);
	if ( pSetConfigTopic != NULL){
		if (nvs->get_str(TOPIC_PREFIX, pSetConfigTopic, &configLen) == NVS_OK){
					strcpy(&pSetConfigTopic[strlen(pSetConfigTopic)], CONFIG_SET_TOPIC);

					res = rclc_subscription_init_default(
							  &xSubConfig,
							  node,
							 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
							  pSetConfigTopic
							  );

					if (res != RCL_RET_OK){
						printf("ERROR: ConfigEntity::createEntities failed to init subscriber \n");
					}
		} else {
					printf("ERROR: NVS does not contain TOPIC_PREFIX\n");
		}
	} else {
		printf("ERROR: Malloc failed\n");
	}

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
	rcl_ret_t res = rclc_executor_add_subscription_with_context(
			executor,
			&xSubConfig,
			&xConfigMsg,
			uRosEntities::subscriptionCallback,
			&xSubConfigContext,
			ON_NEW_DATA);

	if (res != RCL_RET_OK){
		printf("ERROR: ConfigEntity::addToExecutor failed to add executor \n");
	}
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void ConfigEntity::handleSubscriptionMsg(
		const void* msg,
		uRosSubContext_t* context){

	printf("ConfigEntity::handleSubscriptionMsg\n");

	if (context == &xSubConfigContext){
		std_msgs__msg__String * pStringMsg = (std_msgs__msg__String *) msg;

		printf("Config Received: %s\n", pStringMsg->data.data);


		NVSJson *nvs = NVSJson::getInstance();
		json_t mem[CONFIG_MAX_KEYS];
		json_t const* json = json_create( pStringMsg->data.data, mem, sizeof mem / sizeof *mem );
		if ( !json ) {
			printf("Error json create.\n");
			return ;
		}

		//Check for Factory Reset
		json_t const *factoryCmd = json_getProperty(json, FACTORY_RESET);
		if (factoryCmd != NULL){
			factoryReset();
			pubConfigMsg();
			return;
		}

		bool doReboot = false;
		int64_t num64;
		int num;
		json_t const * sib = json_getChild(json);
		while (sib != NULL){
			printf("tag: %s ", json_getName(sib));
			if (strcmp(json_getName(sib), REBOOT) == 0 ){
				doReboot = true;
			} else {
				switch(json_getType(sib)){
				case JSON_TEXT:
					nvs->set_str (
							json_getName(sib),
							json_getValue(sib));
					printf("type String: %s\n", json_getValue(sib));
					break;
				case JSON_BOOLEAN:
					nvs->set_u8 (
							json_getName(sib),
							json_getBoolean(sib)
							);
					printf("type Bool: %d\n", json_getBoolean(sib));
					break;
				case JSON_INTEGER:
					num64 = json_getInteger(sib);
					nvs->set_i64 (
							json_getName(sib),
							num64
							);
					num = num64;
					printf("type Int: %d\n", num);
					break;
				case JSON_REAL:
					nvs->set_double (
							json_getName(sib),
							json_getReal(sib)
							);
					printf("type Float: %f\n", json_getReal(sib));
					break;
				default:
					printf("Unsupported JSON type for: %s\n", json_getName(sib));
				}
			}//if Reboot
			sib = json_getSibling(sib);
		} //White

		if (nvs->isDirty()){
			nvs->commit();
		}
		pubConfigMsg();

		if (doReboot){
			//Just wait until pub finishes
			vTaskDelay(2000);
			reboot();
		}



	}
}

void ConfigEntity::initConfigMsg(){
	//xConfigMsg.frame_id.data = xBuffer;
	//xConfigMsg.frame_id.capacity = CONFIG_BUFFER_SIZE;
	//std_msgs__msg__String__init(&xConfigMsg);
	xConfigMsg.data.data = xBuffer;
	xConfigMsg.data.capacity = CONFIG_BUFFER_SIZE;
}


void ConfigEntity::pubConfigMsg(){
	NVSJson::getInstance()->toJSON(xBuffer, CONFIG_BUFFER_SIZE);
	xConfigMsg.data.size = strlen(xBuffer);

	uRosBridge::getInstance()->publish(
			&xPubConfig,
			&xConfigMsg,
			 this,
			 NULL
			 );
}


void ConfigEntity::setNeededDefaults(){
	NVSJson *nvs = NVSJson::getInstance();

	pico_unique_board_id_t picoId;
	pico_get_unique_board_id (&picoId);
	uint16_t shortId =
			picoId.id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 1] +
			(picoId.id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 2] << 8);

	if (! nvs->contains(ROS2_NODE)){
		char nodeName[10];
		sprintf(nodeName, "Astro%04X", shortId);
		nvs->set_str(ROS2_NODE,  nodeName);
	}

	if (! nvs->contains(TOPIC_PREFIX)){
		char prefix[10];
		sprintf(prefix, "/Astro/Wheel%04X", shortId);
		nvs->set_str(TOPIC_PREFIX,  prefix);
	}

	if (! nvs->contains(CONFIG_PID_KP)){
		nvs->set_double(CONFIG_PID_KP,  0.3);
	}

	if (! nvs->contains(CONFIG_PID_KI)){
		nvs->set_double(CONFIG_PID_KI,  0.001);
	}

	if (! nvs->contains(CONFIG_PID_KD)){
		nvs->set_double(CONFIG_PID_KD,  0.01);
	}

	nvs->commit();
}

void ConfigEntity::factoryReset(){
	NVSJson *nvs = NVSJson::getInstance();
	nvs->erase_all();
	setNeededDefaults();
	reboot();
}

void ConfigEntity::reboot(){
	watchdog_enable(1, 1);
}






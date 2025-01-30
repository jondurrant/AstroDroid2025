/*
 * AstroWheelEntities.h
 *
 *  Created on: 30 Jan 2025
 *      Author: jondurrant
 */

#ifndef FIRMWARE_WHEEL_SRC_ASTROENTITIES_H_
#define FIRMWARE_WHEEL_SRC_ASTROENTITIES_H_

#include "uRosEntities.h"

extern "C"{
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/header.h>
}


#include "ConfigEntity.h"

#define ASTRO_MAX_NUM_SUBS 10

class AstroEntities : public uRosEntities {
public:
	AstroEntities();
	virtual ~AstroEntities();

	/***
	 * Add a uRosEntities object to this aggregate collection
	 * @param ent
	 */
	virtual void addEntity(uRosEntities *ent);

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

private:
	uRosEntities *pEntities[ASTRO_MAX_NUM_SUBS];
	uint xNumSubs = 0;

};

#endif /* FIRMWARE_WHEEL_SRC_ASTROENTITIES_H_ */

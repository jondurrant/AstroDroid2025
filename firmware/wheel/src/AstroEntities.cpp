/*
 * AstroEntities.cpp
 *
 *  Created on: 30 Jan 2025
 *      Author: jondurrant
 */

#include "AstroEntities.h"

AstroEntities::AstroEntities() {
}

AstroEntities::~AstroEntities() {
	// NOP
}

void AstroEntities::addEntity(uRosEntities *ent){
	if (ent != NULL){
		if (xNumSubs < ASTRO_MAX_NUM_SUBS){
			pEntities[xNumSubs] = ent;
			xNumSubs++;
		}
	}
}

void AstroEntities::createEntities(rcl_node_t *node, rclc_support_t *support){
	for (int i =0; i < xNumSubs; i++){
		pEntities[i]->createEntities( node,  support);
	}
}

void AstroEntities::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	for (int i =0; i < xNumSubs; i++){
		pEntities[i]->destroyEntities( node,  support);
	}
}

uint AstroEntities::getCount(){
	uint res = 0;
	for (int i =0; i < xNumSubs; i++){
		res += pEntities[i]->getCount();
	}
	return res;
}

uint AstroEntities::getHandles(){
	uint res = 0;
	for (int i =0; i < xNumSubs; i++){
		res += pEntities[i]->getHandles();
	}
	return res;
}


void AstroEntities::addToExecutor(rclc_executor_t *executor){
	for (int i =0; i < xNumSubs; i++){
			pEntities[i]->addToExecutor(executor);
		}
}


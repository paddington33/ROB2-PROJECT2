/*
 * RRTPlanner.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: daniel
 */

#include "RRTPlanner.h"
#include "rw/math.hpp"
#include "rw/math/Math.hpp"

RRTPlanner::RRTPlanner() {

}

RRTPlanner::~RRTPlanner() {

}

void RRTPlanner::Plan(rw::math::Q qInit, rw::math::Q qGoal)
{

	rw::common::Ptr<rw::models::WorkCell> workcell = robWorkStudio->getWorkCell();
	rw::common::Ptr<rw::models::Device> device = workcell->findDevice("KukaKr16");

	CollisionStrategy::Ptr cdstrategy = ProximityStrategyFactory::makeCollisionStrategy("PQP");
	CollisionDetector::Ptr collisionDetector = new CollisionDetector(workcell, cdstrategy);

	rw::pathplanning::QConstraint::Ptr constraint = rw::pathplanning::QConstraint::make(
			collisionDetector, device, state);

	double epsilon = 1e-4;

	RRT* aTree = new RRT();
	RRT* bTree = new RRT();

	RRT* currentTree = aTree;

	int K = 100; //Number of tries

	for(int i = 0; i < K ;i++)
	{
		rw::math::Q randQ = rw::math::Math::ranQ(device->getBounds());


		randQ -
	}
}

/*
 * RRTPlanner.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: daniel
 */

#include "RRTPlanner.h"
#include "rw/math.hpp"
#include "rw/math/Math.hpp"
#include "list"

RRTPlanner::RRTPlanner() {

}

RRTPlanner::~RRTPlanner() {

}

std::list<rw::math::Q> RRTPlanner::Plan(rw::math::Q qInit, rw::math::Q qGoal)
{
	//comments again

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

	std::list<rw::math::Q> path;


	for(int i = 0; i < K ;i++)
	{
		rw::math::Q randQ = rw::math::Math::ranQ(device->getBounds());
		RRTNode* closestNode = currentTree->getClosestNode(randQ);
		rw::math::Q closestQ = closestNode.getValue();

		rw::math::Q direction = randQ - closestQ;

		rw::math::Q newQ = cloestQ + direction*epsilon;

		if(!constraint->inCollision(newQ))
		{
			RRTNode* newNode = new RRTNode();
			newNode->setValue(newQ);
			newNode->setParrent(closestNode);
		}

		//Early tree swap
		currentTree = (currentTree == aTree) ? bTree : aTree;

		RRTNode* closestNodeInTheOtherTree = currentTree->getClosestNode(newQ);
		rw::math::Q closestQInTheOtherTree = closestNodeInTheOtherTree->getValue();

		rw::math::Q tempQ = newQ;
		rw::math::Q dirQ = closestQInTheOtherTree - newQ;
		rw::math::Q stepQ = epsilon*dirQ/dirQ.norm2();

		bool reached = false;

		do
		{
			if(((rw::math::Q)(closestQInTheOtherTree - tempQ)).norm2() < epsilon)
				reached = true;

			tempQ += stepQ;

		} while(!constraint->inCollision(tempQ) || reached);

		if(!reached)
		{
			RRTNode* newNode = new RRTNode();
			newNode->setValue(tempQ);
			newNode->setParrent(newNode);
		}
		else
		{
			RRTNode* iteratorPathNode = closestNodeInTheOtherTree;
			while(iteratorPathNode->getParrent() != null)
			{
				path.push_back(iteratorPathNode->getValue());
				iteratorPathNode = iteratorPathNode->getParrent();
			}

			iteratorPathNode = newNode;
			while(iteratorPathNode->getParrent() != null)
			{
				path.push_front(iteratorPathNode->getValue());
				iteratorPathNode = iteratorPathNode->getParrent();
			}
			break;
		}

	}

	return path;
}

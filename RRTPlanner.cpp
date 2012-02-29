/*
 * RRTPlanner.cpp
 *
 *  Created on: Feb 27, 2012
 *      Author: Daniel
 */

#include "RRTPlanner.h"
#include "RRT.h"
#include "RRTNode.h"
#include <rw/math.hpp>
#include <rw/math/Math.hpp>
#include <RobWorkStudio.hpp>
#include <rw/proximity/CollisionStrategy.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <list>

using namespace rws;
using namespace rw;

RRTPlanner::RRTPlanner() {
}

RRTPlanner::RRTPlanner(rws::RobWorkStudio* robWorkStudio) {
	_robWorkStudio = robWorkStudio;
}

RRTPlanner::~RRTPlanner() {

}

std::list<rw::math::Q> RRTPlanner::plan(rw::math::Q qInit, rw::math::Q qGoal)
{

	using namespace proximity;

	rw::common::Ptr<rw::models::WorkCell> workcell = _robWorkStudio->getWorkcell();
	rw::common::Ptr<rw::models::Device> device = workcell->findDevice("KukaKr16");

	rw::proximity::CollisionStrategy::Ptr cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
	CollisionDetector::Ptr collisionDetector = new CollisionDetector(workcell, cdstrategy);

	rw::pathplanning::QConstraint::Ptr constraint = rw::pathplanning::QConstraint::make(
			collisionDetector, device, workcell->getDefaultState());

	double epsilon = 1e-2;

	RRT* aTree = new RRT();

	RRTNode* initNode = new RRTNode();
	initNode->setValue(qInit);
	initNode->setParrent(NULL);
	aTree->addNodeToTree(initNode);


	RRT* bTree = new RRT();

	RRTNode* goalNode = new RRTNode();
	goalNode->setValue(qGoal);
	goalNode->setParrent(NULL);
	bTree->addNodeToTree(goalNode);

	RRT* currentTree = aTree;
	RRT* secondTree = bTree;

	int K = 100000; //Number of tries

	std::list<rw::math::Q> path;

	for(int i = 0; i < K ;i++)
	{
		rw::math::Q randQ = rw::math::Math::ranQ(device->getBounds());
		RRTNode* closestNode = currentTree->getClosestNode(randQ);

		rw::math::Q closestQ = closestNode->getValue();

		rw::math::Q direction = randQ - closestQ;

		rw::math::Q newQ = closestQ + direction*epsilon;

		//Early tree swap

		if(currentTree == aTree)
		{
			currentTree = bTree;
			secondTree = aTree;
		}
		else
		{
			currentTree = aTree;
			secondTree = bTree;
		}

		if(!constraint->inCollision(newQ))
		{

			RRTNode* newNode = new RRTNode();
			newNode->setValue(newQ);
			newNode->setParrent(closestNode);
			secondTree->addNodeToTree(newNode);

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

			} while(!constraint->inCollision(tempQ) && !reached);

			tempQ -= stepQ;


			if(!reached)
			{
				RRTNode* inewNode;
				inewNode = new RRTNode();
				inewNode->setValue(tempQ);
				inewNode->setParrent(newNode);
				secondTree->addNodeToTree(inewNode);
			}
			else
			{
				RRTNode* iteratorPathNode = closestNodeInTheOtherTree;
				while(iteratorPathNode != NULL)
				{
					path.push_back(iteratorPathNode->getValue());
					iteratorPathNode = iteratorPathNode->getParrent();
				}

				iteratorPathNode = newNode;
				while(iteratorPathNode != NULL)
				{
					path.push_front(iteratorPathNode->getValue());
					iteratorPathNode = iteratorPathNode->getParrent();
				}
				return path;
			}
		}

	}

	return path;
}

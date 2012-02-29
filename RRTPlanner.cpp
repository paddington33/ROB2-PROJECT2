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

RRTPlanner::RRTPlanner(rws::RobWorkStudio* robWorkStudio) {
	_robWorkStudio = robWorkStudio;
}

RRTPlanner::~RRTPlanner() {

}

std::list<rw::math::Q> RRTPlanner::plan(rw::math::Q qInit, rw::math::Q qGoal)
{

	using namespace proximity;

	rw::common::Ptr<rw::models::WorkCell> workcell = _robWorkStudio->getWorkCell();
	rw::common::Ptr<rw::models::Device> device = workcell->findDevice("KukaKr16");

	rw::proximity::CollisionStrategy::Ptr cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
	CollisionDetector::Ptr collisionDetector = new CollisionDetector(workcell, cdstrategy);

	rw::pathplanning::QConstraint::Ptr constraint = rw::pathplanning::QConstraint::make(
			collisionDetector, device, workcell->getDefaultState());

	double epsilon = 1e-3;

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

	std::cout << "RRTPlanner 1" << std::endl;

	for(int i = 0; i < K ;i++)
	{

		std::cout << "RRTPlanner " << i << std::endl;

		rw::math::Q randQ = rw::math::Math::ranQ(device->getBounds());
		RRTNode* closestNode = currentTree->getClosestNode(randQ);

		std::cout << "cnit-1 " << closestNode << std::endl;

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

			std::cout << "cnit0" << std::endl;

			RRTNode* newNode = new RRTNode();
			newNode->setValue(newQ);
			newNode->setParrent(closestNode);
			secondTree->addNodeToTree(newNode);

			std::cout << "cnit1" << std::endl;

			RRTNode* closestNodeInTheOtherTree = currentTree->getClosestNode(newQ);

			std::cout << "cnit2" << closestNodeInTheOtherTree << std::endl;

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

//				std::cout << "NQ" << newQ << std::endl;
//				std::cout << "CP" << closestQInTheOtherTree << std::endl;
//				std::cout << "TQ" <<tempQ << std::endl;
//				std::cout << "SI" << ((rw::math::Q)(closestQInTheOtherTree - tempQ)).norm2() << std::endl;
//				std::cout << reached << std::endl;

//				if(constraint->inCollision(tempQ))
//					std::cout << "COL" << std::endl;
//				else
//					std::cout << "slet ike CoL" << std::endl;


			} while(!constraint->inCollision(tempQ) && !reached);




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

					std::cout << "Næ jeg er faktisk her" << std::endl;
				}

				iteratorPathNode = newNode;
				while(iteratorPathNode != NULL)
				{
					path.push_front(iteratorPathNode->getValue());
					iteratorPathNode = iteratorPathNode->getParrent();

					std::cout << "Næ jeg er her" << std::endl;
				}
				return path;
			}
		}

	}

	return path;
}

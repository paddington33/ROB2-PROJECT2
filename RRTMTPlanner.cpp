/*
 * RRTMTPlanner.cpp
 *
 *  Created on: Feb 29, 2012
 *      Author: Daniel
 */

#include "RRTMTPlanner.h"

RRTMTPlanner::RRTMTPlanner(rws::RobWorkStudio* robWorkStudio, int connectN) :
	RRTPlanner(robWorkStudio) ,
	_connectN(connectN)
{

	_workcell = robWorkStudio->getWorkcell();
	_device = _workcell->findDevice("KukaKr16");
	_cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
	_collisionDetector = new rw::proximity::CollisionDetector(_workcell, _cdstrategy);
	_constraint = rw::pathplanning::QConstraint::make(_collisionDetector, _device, _workcell->getDefaultState());
}

RRTMTPlanner::~RRTMTPlanner() {

}

void RRTMTPlanner::initTrees()
{
	_trees.push_back(new RRT(qInit));

	for(int i = 1 ; i < _numberOfTrees -1 ; i++ )
		_trees.push_back(new RRT(cFree->sample()));


	if(_numberOfTrees != 1)
		_trees.push_back(new RRT(qGoal));
}

rw::trajectory::QPath RRTMTPlanner::plan(rw::math::Q qInit, rw::math::Q qGoal)
{
	int K = 1000;

	for(int i = 0;i < K; i++ )
	{
		rw::common::Ptr<RRT> currentTree = chooseTree(NULL);
		rw::common::Ptr<RRTNode> newNode = extendTreeInRandomDirection(currentTree);
		rw::common::Ptr<RRTNode> closestNode = cloestNodeInAnyOtherTree(currentTree,newNode);

		if( ((rw::math::Q)(newNode->getValue() - closestNode->getValue())).norm2() < _d )
			//Connect somehow

		connect(currentTree,newNode,closestNode);
	}

}

rw::common::Ptr<RRTNode> RRTMTPlanner::extendTreeInRandomDirection(rw::common::Ptr<RRTNode> nodeClose)
{

}

bool RRTMTPlanner::edgeCollisionDetection(rw::common::Ptr<RRTNode> nodeClose, rw::common::Ptr<RRTNode> nodeNew)
{

}

rw::common::Ptr<RRTNode> RRTMTPlanner::cloestNodeInAnyOtherTree(rw::common::Ptr<RRT> currentTree)
{

}

void RRTMTPlanner::connect(rw::common::Ptr<RRT> currentTree, rw::common::Ptr<RRTNode> newNode, rw::common::Ptr<RRTNode> closestNode)
{

}

rw::common::Ptr<RRT> RRTMTPlanner::swap(rw::common::Ptr<RRT> currentTree)
{

}

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

	_cFree = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device), _constraint);

}

RRTMTPlanner::~RRTMTPlanner() {

}

void RRTMTPlanner::initTrees(rw::math::Q qInit, rw::math::Q qGoal)
{
	_trees.push_back(new RRT(qInit));

	for(int i = 1 ; i < _numberOfTrees -1 ; i++ )
		_trees.push_back(new RRT(_cFree->sample()));


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

rw::common::Ptr<RRTNode> RRTMTPlanner::extendTreeInRandomDirection(rw::common::Ptr<RRT> currentTree)
{

}

bool RRTMTPlanner::edgeCollisionDetection(rw::common::Ptr<RRTNode> nodeClose, rw::common::Ptr<RRTNode> nodeNew)
{

}

rw::common::Ptr<RRTNode> RRTMTPlanner::cloestNodeInAnyOtherTree(rw::common::Ptr<RRT> currentTree, rw::common::Ptr<RRTNode> currentNode)
{
	RRTNode * inewNode = NULL;

	_trees.size()

	for()

}



bool RRTMTPlanner::connect(rw::common::Ptr<RRT> currentTree, rw::common::Ptr<RRTNode> newNode, rw::common::Ptr<RRTNode> closestNode)
{
	assert(_epsilon);

	rw::math::Q dirQ = newNode->getValue() - closestNode->getValue();
	rw::math::Q newQ = closestNode->getValue() + dirQ*_epsilon;
	rw::math::Q stpQ = _epsilon*dirQ/dirQ.norm2();
	rw::math::Q tmpQ = newQ;

	bool reached = false;

	do {
		if (((rw::math::Q) (closestNode->getValue() - tmpQ)).norm2()< _epsilon)
			reached = true;
		tmpQ += stpQ;
	} while (!_constraint->inCollision(tmpQ) && !reached);

	tmpQ -= stpQ;

	RRTNode* inewNode;
	inewNode = new RRTNode();
	inewNode->setValue(tmpQ);
	inewNode->setParrent(newNode.get());
	currentTree->addNodeToTree(inewNode);

	return reached;
}

rw::common::Ptr<RRT> RRTMTPlanner::chooseTree(rw::common::Ptr<RRT> currentTree)
{

}

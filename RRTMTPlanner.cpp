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
	assert(_cFree);
}

void RRTMTPlanner::initTrees(rw::common::Ptr<RRTNode> initNode, rw::common::Ptr<RRTNode> goalNode)
{
	_trees.push_back(new RRT(initNode));

	for(int i = 1 ; i < _numberOfTrees -1 ; i++ )
		_trees.push_back(new RRT(new RRTNode(_cFree->sample(),NULL)));


	if(_numberOfTrees != 1)
		_trees.push_back(new RRT(goalNode));
}

rw::trajectory::QPath RRTMTPlanner::plan(rw::math::Q qInit, rw::math::Q qGoal)
{
	rw::common::Ptr<RRTNode> initNode = new RRTNode(qInit,NULL);
	rw::common::Ptr<RRTNode> goalNode = new RRTNode(qGoal,NULL);

	initTrees(initNode, goalNode);

	int K = 10000;

	rw::common::Ptr<RRTNode> newNode;

	for(int i = 0;i < K; i++ )
	{
		rw::common::Ptr<RRT> currentTree = chooseTree(NULL);

		do
		{
			newNode = extendTreeInRandomDirection(currentTree);
		} while(!newNode);

		rw::common::Ptr<RRT> secondTree;
		rw::common::Ptr<RRTNode> closestNode = closestNodeInAnyOtherTree(secondTree,currentTree,newNode);

		if( ((rw::math::Q)(newNode->getValue() - closestNode->getValue())).norm2() < _d )
		{
			if(!edgeCollisionDetection(closestNode,newNode))
			{
				mergeTree(currentTree, secondTree, newNode, closestNode);
				if(closestNode->getTree() == newNode->getTree())
					return getPath(closestNode,newNode);
			}
		}
		if(connect(currentTree,newNode,closestNode))
			mergeTree(currentTree, secondTree, newNode, closestNode);

		if(closestNode->getTree() == newNode->getTree())
			return getPath(initNode,goalNode);
	}

	return NULL;

}

void RRTMTPlanner::mergeTree(rw::common::Ptr<RRT> firstTree,
		rw::common::Ptr<RRT> secondTree,
		rw::common::Ptr<RRTNode> firstNode,
		rw::common::Ptr<RRTNode> secondNode )
{
	rw::common::Ptr<RRTNode> tempNode;
	rw::common::Ptr<RRTNode> tempParrent = firstNode;

	while(secondNode != NULL)
	{
		tempNode = secondNode->getParrent();
		secondNode->setParrent(tempParrent);
		tempParrent = secondNode;
		secondNode = tempNode;
	}

	std::list<rw::common::Ptr<RRTNode> > nodeList = secondTree->getListOfNodes();

	std::list<rw::common::Ptr<RRTNode> >::iterator it;

	for(it = nodeList.begin(); it!=nodeList.end(); ++it)
		firstTree->addNodeToTree(*it);

	_trees.remove(secondTree);

}

rw::common::Ptr<RRTNode> RRTMTPlanner::extendTreeInRandomDirection(rw::common::Ptr<RRT> currentTree)
{
	rw::math::Q rndQ = _cFree->sample();

	rw::common::Ptr<RRTNode> tmpNode = new RRTNode();
	rw::common::Ptr<RRTNode> cloNode = new RRTNode();
	rw::common::Ptr<RRTNode> newNode = new RRTNode();

	tmpNode->setValue(rndQ);

	cloNode = currentTree->getClosestNode(rndQ);

	rw::math::Q dirQ = tmpNode->getValue()- cloNode->getValue();
	rw::math::Q newQ = cloNode->getValue() + dirQ*_epsilon;
	rw::math::Q stpQ = _epsilon*dirQ/dirQ.norm2();

	newNode->setValue(cloNode->getValue()+stpQ);
	if(!_constraint->inCollision(newNode->getValue())){

		newNode->setParrent(cloNode);
		currentTree->addNodeToTree(newNode);

		return newNode;
	}
	return NULL;
}

bool RRTMTPlanner::edgeCollisionDetection(rw::common::Ptr<RRTNode> nodeClose, rw::common::Ptr<RRTNode> nodeNew)
{
	const double eps = 0.01;
	rw::math::Q qStart = nodeNew->getValue();
	rw::math::Q qEnd = nodeClose->getValue();

	const rw::math::Q qDelta = qEnd - qStart;
	const double normDeltaQ = qDelta.norm2();

	const int n = ceil(normDeltaQ/eps);

	const int levels = Math::ceilLog2(n);

	const double extendedLength = pow(2,levels)*eps;

	const Q qExtended = (qDelta/qDelta.norm2())*extendedLength;

	Q qStep,qTemp;
	int steps;

	for(int i = 1;i <= levels;i++)
	{
		steps = pow(2,i-1);
		qStep = qExtended/steps;
		for(int j = 1 ; j<= steps;j++)
		{
			qTemp = qStart + (j - 1/2)*qStep;
			if( ((j - 1/2)*qStep).norm2() <= normDeltaQ )
			{
				if(_constraint->inCollision(qTemp))
					return false;
			}
		}
	}
	return true;
}

rw::common::Ptr<RRTNode> RRTMTPlanner::closestNodeInAnyOtherTree(rw::common::Ptr<RRT> &secondTree, rw::common::Ptr<RRT> currentTree, rw::common::Ptr<RRTNode> currentNode)
{
	secondTree = NULL;

	double distanceToNode = std::numeric_limits<double>::max();
	rw::common::Ptr<RRTNode> bstNode = NULL;
	rw::common::Ptr<RRTNode> tmpNode = NULL;

	std::list<rw::common::Ptr<RRT> >::iterator it;

	for(it = _trees.begin(); it!=_trees.end(); ++it){
		tmpNode = ((*it)->getClosestNode(currentNode->getValue()));

		if(bstNode == NULL){
			bstNode = tmpNode;
		}else if(((rw::math::Q)bstNode->getValue() - tmpNode->getValue()).norm2() < distanceToNode){
			distanceToNode = ((rw::math::Q)bstNode->getValue() - tmpNode->getValue()).norm2();
			bstNode = tmpNode;
		}
	}

	if(bstNode != NULL)
		secondTree = bstNode->getTree();

	return bstNode;
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

	rw::common::Ptr<RRTNode> inewNode;
	inewNode = new RRTNode();
	inewNode->setValue(tmpQ);

	inewNode->setParrent(newNode);
	currentTree->addNodeToTree(inewNode);

	newNode = inewNode;

	return reached;
}

rw::common::Ptr<RRT> RRTMTPlanner::chooseTree(rw::common::Ptr<RRT> currentTree)
{
	_trees.push_back(_trees.front());
	_trees.pop_front();

	return _trees.front();
}

rw::trajectory::QPath RRTMTPlanner::getPath(rw::common::Ptr<RRTNode> initNode,rw::common::Ptr<RRTNode> goalNode )
{
	rw::trajectory::QPath path;

	if(initNode->getParrent() == NULL)
	{

		while(goalNode->getParrent() != initNode)
		{
			path.push_back(goalNode->getValue());
			goalNode = goalNode->getParrent();
		}
	}
	else
	{
		while(initNode->getParrent() != goalNode)
		{
			path.push_back(initNode->getValue());
			initNode = initNode->getParrent();
		}
	}

	return path;
}

void RRTMTPlanner::setConnectN(int n)
{
    _connectN = n;
}
void RRTMTPlanner::setEpsilon(double e)
{
    _epsilon = e;
}
void RRTMTPlanner::setMinDis(double minDis)
{
    _d = minDis;
}
void RRTMTPlanner::setNumberOfTree(int nrOfTree)
{
    _numberOfTrees = nrOfTree;
}
void RRTMTPlanner::setSwapStrategy(int ss)
{
    _swapStrategy = ss;
}


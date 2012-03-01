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

	std::cout << "RRT 1" << std::endl;
	_workcell = robWorkStudio->getWorkcell();
	std::cout << "RRT 2" << std::endl;
	_device = _workcell->findDevice("PA10");
	std::cout << "RRT 3" << std::endl;
	_cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
	std::cout << "RRT 4" << std::endl;
	_collisionDetector = new rw::proximity::CollisionDetector(_workcell, _cdstrategy);
	std::cout << "RRT 5" << std::endl;
	_constraint = rw::pathplanning::QConstraint::make(_collisionDetector, _device, _workcell->getDefaultState());
	std::cout << "RRT 6" << std::endl;
	_cFree = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(_device), _constraint);
	std::cout << "RRT 7" << std::endl;
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

	std::cout << "plan 1" << std::endl;

	rw::common::Ptr<RRTNode> newNode;

	for(int i = 0;i < K; i++ )
	{
		std::cout << "plan 2 i " <<  i  << std::endl;

		rw::common::Ptr<RRT> currentTree = chooseTree(NULL);

		std::cout << "plan 3 i " <<  i  << std::endl;
		std::cout << "currentTree " <<  i  <<  " "  << currentTree << std::endl;

		do
		{
			newNode = extendTreeInRandomDirection(currentTree);
		} while(!newNode);


		std::cout << "plan 4 i " <<  i  << std::endl;

		rw::common::Ptr<RRT> secondTree;

		std::cout << "plan 5 i " <<  i  << std::endl;

		rw::common::Ptr<RRTNode> closestNode = closestNodeInAnyOtherTree(secondTree,currentTree,newNode);



		std::cout << "plan 6 secondTree " <<  secondTree  << std::endl;

		if( ((rw::math::Q)(newNode->getValue() - closestNode->getValue())).norm2() < _d )
		{

			std::cout << "plan 7 i " <<  i  << std::endl;

			if(!edgeCollisionDetection(closestNode,newNode))
			{

				std::cout << "plan 8 i " <<  i  << std::endl;

				mergeTree(currentTree, secondTree, newNode, closestNode);
				if(closestNode->getTree() == newNode->getTree())
					return getPath(closestNode,newNode);
			}
		}

		std::cout << "plan 9 i " <<  i  << std::endl;

		if(connect(currentTree,newNode,closestNode))
		{
			std::cout << "plan 9 currentTree " << currentTree << std::endl;
			std::cout << "plan 9 newNode " << newNode << std::endl;
			std::cout << "plan 9 closestNode " << closestNode << std::endl;
			std::cout << "plan 9 secondTree " << secondTree << std::endl;
			std::cout << "plan 9 connect " <<  i  << std::endl;
			mergeTree(currentTree, secondTree, newNode, closestNode);
		}


		std::cout << "plan 10 i " <<  i  << std::endl;

		if(closestNode->getTree() == newNode->getTree())
			return getPath(initNode,goalNode);

		std::cout << "plan 11 i " <<  i  << std::endl;
	}

	return NULL;

}

void RRTMTPlanner::mergeTree(rw::common::Ptr<RRT> firstTree,
		rw::common::Ptr<RRT> secondTree,
		rw::common::Ptr<RRTNode> firstNode,
		rw::common::Ptr<RRTNode> secondNode )
{
	std::cout << "merge 1" << std::endl;

//	firstNode->setParrent(secondNode);

	std::cout << "merge 2" << std::endl;

	rw::common::Ptr<RRTNode> tempNode;
	rw::common::Ptr<RRTNode> tempParrent = firstNode;

	std::cout << "merge 3" << std::endl;

	int testCounter = 0;

	while(secondNode != NULL)
	{
		std::cout << "merge loop " << testCounter++ << std::endl;

		tempNode = secondNode->getParrent();
		secondNode->setParrent(tempParrent);
		tempParrent = secondNode;
		secondNode = tempNode;
	}



	std::cout << "merge 6.5" << std::endl;

	std::list<rw::common::Ptr<RRTNode> > nodeList = secondTree->getListOfNodes();

	std::cout << "merge 7" << std::endl;

	std::list<rw::common::Ptr<RRTNode> >::iterator it;

	std::cout << "merge 8 nodeList size " << nodeList.size()  << std::endl;


	for(it = nodeList.begin(); it!=nodeList.end(); ++it)
		firstTree->addNodeToTree(*it);

	std::cout << "merge 9" << std::endl;

	_trees.remove(secondTree);

}

rw::common::Ptr<RRTNode> RRTMTPlanner::extendTreeInRandomDirection(rw::common::Ptr<RRT> currentTree)
{
	std::cout << "ext 1" << std::endl;

	rw::math::Q rndQ = _cFree->sample();

	std::cout << "ext 2" << std::endl;

	rw::common::Ptr<RRTNode> tmpNode = new RRTNode();
	rw::common::Ptr<RRTNode> cloNode = new RRTNode();
	rw::common::Ptr<RRTNode> newNode = new RRTNode();

	std::cout << "ext 3" << std::endl;

	tmpNode->setValue(rndQ);

	std::cout << "ext 4 rndQ " << rndQ << std::endl;

	cloNode = currentTree->getClosestNode(rndQ);

	std::cout << "ext 5" << std::endl;

	rw::math::Q dirQ = tmpNode->getValue()- cloNode->getValue();
	rw::math::Q newQ = cloNode->getValue() + dirQ*_epsilon;
	rw::math::Q stpQ = _epsilon*dirQ/dirQ.norm2();

	std::cout << "ext 6" << std::endl;

	newNode->setValue(cloNode->getValue()+stpQ);
	if(!_constraint->inCollision(newNode->getValue())){

		std::cout << "ext 7" << std::endl;

		newNode->setParrent(cloNode);
		currentTree->addNodeToTree(newNode);

		std::cout << "ext 8" << std::endl;

		return newNode;
	}

	std::cout << "ext 9" << std::endl;
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
	std::cout << "any 1" << std::endl;

	secondTree = NULL;

	double distanceToNode = std::numeric_limits<double>::max();
	rw::common::Ptr<RRTNode> bstNode = NULL;
	rw::common::Ptr<RRTNode> tmpNode = NULL;

	std::cout << "any 2" << std::endl;

	std::list<rw::common::Ptr<RRT> >::iterator it;

	std::cout << "any 3" << std::endl;

	for(it = _trees.begin(); it!=_trees.end(); ++it){

		std::cout << "any 4 it " << *it << std::endl;

		std::cout << "ant currentNode " << currentNode << std::endl;
		std::cout << "ant currentNode " << currentNode->getValue() << std::endl;

		tmpNode = ((*it)->getClosestNode(currentNode->getValue()));

		std::cout << "any 5" << std::endl;

		if(bstNode == NULL){
			bstNode = tmpNode;
		}else if(((rw::math::Q)bstNode->getValue() - tmpNode->getValue()).norm2() < distanceToNode){
			distanceToNode = ((rw::math::Q)bstNode->getValue() - tmpNode->getValue()).norm2();
			std::cout << "any 6" << std::endl;
			bstNode = tmpNode;
		}

		std::cout << "any 7" << std::endl;
	}

	std::cout << "any 8" << std::endl;

	if(bstNode != NULL)
		secondTree = bstNode->getTree();

	std::cout << "any 9 secondTree " <<  bstNode->getTree() << std::endl;

	return bstNode;
}



bool RRTMTPlanner::connect(rw::common::Ptr<RRT> currentTree, rw::common::Ptr<RRTNode> newNode, rw::common::Ptr<RRTNode> closestNode)
{

	std::cout << "con 1 epsilon " << _epsilon << std::endl;

	assert(_epsilon);

	std::cout << "con 2" << std::endl;

	rw::math::Q dirQ = newNode->getValue() - closestNode->getValue();
	rw::math::Q newQ = closestNode->getValue() + dirQ*_epsilon;
	rw::math::Q stpQ = _epsilon*dirQ/dirQ.norm2();
	rw::math::Q tmpQ = newQ;

	std::cout << "con 3" << std::endl;

	bool reached = false;

	std::cout << "con 4" << std::endl;


	int testCounter = 0;
	do {
//		std::cout << "con 5 " << testCounter++ << std::endl;

		if (((rw::math::Q) (closestNode->getValue() - tmpQ)).norm2()< _epsilon)
			reached = true;
		tmpQ += stpQ;
	} while (!_constraint->inCollision(tmpQ) && !reached);

	std::cout << "con 6" << std::endl;

	tmpQ -= stpQ;

	rw::common::Ptr<RRTNode> inewNode;
	inewNode = new RRTNode();
	inewNode->setValue(tmpQ);

	std::cout << "con 7" << std::endl;

	inewNode->setParrent(newNode);
	currentTree->addNodeToTree(inewNode);

	std::cout << "con 8" << std::endl;

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


	std::cout << "getpath 1" << std::endl;

	std::cout << initNode << std::endl;
	std::cout << goalNode << std::endl;

	if(initNode->getParrent() == NULL)
	{

		while(goalNode->getParrent() != initNode)
		{
			std::cout << "goal 2 " << goalNode->getValue() << std::endl;

			path.push_back(goalNode->getValue());
			goalNode = goalNode->getParrent();
		}
	}
	else
	{
		while(initNode->getParrent() != goalNode)
		{
			std::cout << "init 2 " << initNode->getValue() << std::endl;

			path.push_back(initNode->getValue());
			initNode = initNode->getParrent();
		}
	}

	std::cout << "Den nåede til regne en path, men kunne ikke," << std::endl;

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


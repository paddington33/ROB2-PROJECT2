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

}

RRTMTPlanner::~RRTMTPlanner() {

}

rw::trajectory::QPath RRTMTPlanner::RRTMTPlanner::plan(rw::math::Q qInit, rw::math::Q qGoal)
{

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

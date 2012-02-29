/*
 * RRTMTPlanner.h
 *
 *  Created on: Feb 29, 2012
 *      Author: Daniel
 */

#ifndef RRTMTPLANNER_H_
#define RRTMTPLANNER_H_

#include "RRTPlanner.h"
#include "RRTNode.h"
#include "RRT.h"
#include <rws/RobWorkStudio.hpp>
#include <rw/trajectory.hpp>
#include <rw/common.hpp>


class RRTMTPlanner: public RRTPlanner {
private:
	std::list<rw::common::Ptr<RRT> > _trees;
	int _connectN;
public:
	RRTMTPlanner(rws::RobWorkStudio* robWorkStudio, int connectN = -1);
	rw::trajectory::QPath plan(rw::math::Q qInit, rw::math::Q qGoal);
	rw::common::Ptr<RRTNode> extendTreeInRandomDirection(rw::common::Ptr<RRTNode> nodeClose);
	bool edgeCollisionDetection(rw::common::Ptr<RRTNode> nodeClose, rw::common::Ptr<RRTNode> nodeNew);
	rw::common::Ptr<RRTNode> cloestNodeInAnyOtherTree(rw::common::Ptr<RRT> currentTree);
	void connect(rw::common::Ptr<RRT> currentTree, rw::common::Ptr<RRTNode> newNode, rw::common::Ptr<RRTNode> closestNode);
	rw::common::Ptr<RRT> swap(rw::common::Ptr<RRT> currentTree);
	virtual ~RRTMTPlanner();
};

#endif /* RRTMTPLANNER_H_ */
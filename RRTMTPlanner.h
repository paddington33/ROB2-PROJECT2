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

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/trajectory.hpp>
#include <rw/common.hpp>
#include <rw/pathplanning/QConstraint.hpp>




class RRTMTPlanner: public RRTPlanner {
private:
	std::list<rw::common::Ptr<RRT> > _trees;

	rw::common::Ptr<rw::models::WorkCell> _workcell;
	rw::common::Ptr<rw::models::Device> _device;
	rw::common::Ptr<rw::proximity::CollisionStrategy> _cdstrategy;
	rw::common::Ptr<rw::proximity::CollisionDetector> _collisionDetector;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraint;

	int _connectN;
	double _epsilon;

	double _d; //Min distance to test connection

	double _d; //Min distance to test connection

public:
	RRTMTPlanner(rws::RobWorkStudio* robWorkStudio, int connectN = -1);
	void initTrees();
	rw::trajectory::QPath plan(rw::math::Q qInit, rw::math::Q qGoal);
	rw::common::Ptr<RRTNode> extendTreeInRandomDirection(rw::common::Ptr<RRT> currentTree);
	bool edgeCollisionDetection(rw::common::Ptr<RRTNode> nodeClose, rw::common::Ptr<RRTNode> nodeNew);
	rw::common::Ptr<RRTNode> cloestNodeInAnyOtherTree(rw::common::Ptr<RRT> currentTree);
	void connect(rw::common::Ptr<RRT> currentTree, rw::common::Ptr<RRTNode> newNode, rw::common::Ptr<RRTNode> closestNode);
	rw::common::Ptr<RRT> chooseTree(rw::common::Ptr<RRT> currentTree);
	virtual ~RRTMTPlanner();
};

#endif /* RRTMTPLANNER_H_ */

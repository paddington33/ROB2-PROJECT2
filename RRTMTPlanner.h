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
#include <rw/pathplanning/QSampler.hpp>


USE_ROBWORK_NAMESPACE

class RRTMTPlanner: public RRTPlanner {
private:
	std::list<rw::common::Ptr<RRT> > _trees;

	rw::common::Ptr<rw::models::WorkCell> _workcell;
	rw::common::Ptr<rw::models::Device> _device;
	rw::common::Ptr<rw::proximity::CollisionStrategy> _cdstrategy;
	rw::common::Ptr<rw::proximity::CollisionDetector> _collisionDetector;
	rw::common::Ptr<rw::pathplanning::QConstraint> _constraint;
	rw::common::Ptr<rw::pathplanning::QSampler> _cFree;

	int _swapStrategy;
	int _connectN;
	double _epsilon;

	double _d; //Min distance to test connection
	int _numberOfTrees;

	bool _edgeDetection;
	bool _pearls;

public:
	RRTMTPlanner(rws::RobWorkStudio* robWorkStudio, int connectN = -1);
	void setWorkCell(std::string deviceName);
	void initTrees(rw::common::Ptr<RRTNode> initNode, rw::common::Ptr<RRTNode> goalNode);
	rw::trajectory::QPath plan(rw::math::Q qInit, rw::math::Q qGoal);
	rw::trajectory::QPath plan();
	rw::common::Ptr<RRTNode> extendTreeInRandomDirection(rw::common::Ptr<RRT> currentTree);
	bool edgeCollisionDetection(rw::common::Ptr<RRTNode> nodeClose, rw::common::Ptr<RRTNode> nodeNew);
	rw::common::Ptr<RRTNode> closestNodeInAnyOtherTree(rw::common::Ptr<RRT> &secondTree,rw::common::Ptr<RRT> currentTree,rw::common::Ptr<RRTNode> currentNode);
	bool connect(rw::common::Ptr<RRT> currentTree, rw::common::Ptr<RRTNode> newNode, rw::common::Ptr<RRTNode> closestNode);
	rw::common::Ptr<RRT> chooseTree(rw::common::Ptr<RRT> currentTree);
	void mergeTree(rw::common::Ptr<RRT> firstTree,rw::common::Ptr<RRT> secondTree,rw::common::Ptr<RRTNode> firstNode,rw::common::Ptr<RRTNode> secondNode );
	rw::trajectory::QPath getPath(rw::common::Ptr<RRTNode> initNode,rw::common::Ptr<RRTNode> goalNode );
    void setConnectN(int n);
    void setEpsilon(double e);
    void setMinDis(double minDis);
    void setNumberOfTree(int nrOfTree);
    void setSwapStrategy(int ss);
    void setedgeDetection(bool edgeDetection);
    void setPearls(bool pearls);
	virtual ~RRTMTPlanner();
};

#endif /* RRTMTPLANNER_H_ */

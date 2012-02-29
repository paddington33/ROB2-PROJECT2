/*
 * RRTPlanner.h
 *
 *  Created on: Feb 28, 2012
 *      Author: daniel
 */

#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "rw/math.hpp"
#include "list"
#include "rws/RobWorkStudio.hpp"

class RRTPlanner {
private:
	rws::RobWorkStudio* _robWorkStudio;
public:
	RRTPlanner();
	RRTPlanner(rws::RobWorkStudio* robWorkStudio);
	virtual ~RRTPlanner();
	std::list<rw::math::Q> plan(rw::math::Q qInit, rw::math::Q qGoal);
};

#endif /* RRTPLANNER_H_ */

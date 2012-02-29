/*
 * RRTMTPlanner.h
 *
 *  Created on: Feb 29, 2012
 *      Author: daniel
 */

#ifndef RRTMTPLANNER_H_
#define RRTMTPLANNER_H_

#include "RRTPlanner.h"
#include <rws/RobWorkStudio.hpp>

class RRTMTPlanner: public RRTPlanner {
public:
	RRTMTPlanner(rws::RobWorkStudio* robWorkStudio);
	std::list<rw::math::Q> plan(rw::math::Q qInit, rw::math::Q qGoal);
	virtual ~RRTMTPlanner();
};

#endif /* RRTMTPLANNER_H_ */

/*
 * RRTPlanner.h
 *
 *  Created on: Feb 28, 2012
 *      Author: daniel
 */

#ifndef RRTPLANNER_H_
#define RRTPLANNER_H_

#include "rw/math.hpp"

class RRTPlanner {
public:
	RRTPlanner();
	virtual ~RRTPlanner();
	void plan(rw::math::Q qInit, rw::math::Q qGoal);
};

#endif /* RRTPLANNER_H_ */

/*
 * RRTMTPlanner.cpp
 *
 *  Created on: Feb 29, 2012
 *      Author: Daniel
 */

#include "RRTMTPlanner.h"
#include "RRTPlanner.h"

RRTMTPlanner::RRTMTPlanner(rws::RobWorkStudio* robWorkStudio) : RRTPlanner(robWorkStudio) {
	std::cout << "I am a RRTMT Planner" << std::endl;
}

RRTMTPlanner::~RRTMTPlanner() {

}

std::list<rw::math::Q> RRTMTPlanner::plan(rw::math::Q qInit, rw::math::Q qGoal)
{
	std::cout << "Planning with RRTMTPlanner" << std::endl;
	std::list<rw::math::Q>* path = new std::list<rw::math::Q>();





	return *path;


}

/*
 * RRT.cpp
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel
 */

#include <rw/math/Q.hpp>

#include "RRT.h"


RRT::RRT(){
}

RRT::RRT(rw::math::Q q){
	_tree.push_back(q);
}

RRT::~RRT() {

}

RRTNode * RRT::getClosestNode(rw::math::Q node) {

	RRTNode * closestNode = NULL;
	rw::math::Q tempQ;
	double distanceToNode = std::numeric_limits<double>::max();

	std::list<RRTNode*>::iterator itt;
	for (itt = _tree.begin(); itt != _tree.end(); itt++) {
		tempQ = (*itt)->getValue();
		if((tempQ - node).norm2() < distanceToNode){
			distanceToNode = (tempQ - node).norm2();
			closestNode = *itt;
		}
	}

	return closestNode;
}


bool RRT::addNodeToTree(RRTNode * nodeNew) {
	_tree.push_back(nodeNew);
	return !(_tree.empty());
}


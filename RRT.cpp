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

RRT::RRT(rw::common::Ptr<RRTNode> node){

	_tree.push_back(node);
}

RRT::~RRT() {

}

rw::common::Ptr<RRTNode> RRT::getClosestNode(rw::math::Q node) {

	std::cout << "close 1" << std::endl;

	rw::common::Ptr<RRTNode> closestNode = NULL;

	std::cout << "close 2" << std::endl;

	rw::math::Q tempQ;

	std::cout << "close 3" << std::endl;

	double distanceToNode = std::numeric_limits<double>::max();

	std::cout << "close 4" << std::endl;

	std::list<rw::common::Ptr<RRTNode> >::iterator itt;

	std::cout << "close 5 tree size " << _tree.size()  << std::endl;
	for (itt = _tree.begin(); itt != _tree.end(); itt++) {

//		std::cout << "close loop " << (*itt)->getValue() << std::endl;

		tempQ = (*itt)->getValue();

//		std::cout << "close 6" << std::endl;

		if((tempQ - node).norm2() < distanceToNode){
			distanceToNode = (tempQ - node).norm2();

//			std::cout << "close 7" << std::endl;

			closestNode = *itt;
		}
	}

	std::cout << "close 8 " << closestNode << std::endl;
	return closestNode;
}

std::list<rw::common::Ptr<RRTNode> > RRT::getListOfNodes(){
	return _tree;
}

bool RRT::addNodeToTree(rw::common::Ptr<RRTNode> newNode) {
	_tree.push_back(newNode);

	newNode->addTreeToNode(this);

	return !(_tree.empty());
}


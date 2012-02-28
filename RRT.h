/*
 * RRT.h
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel
 */
#include "RRTNode.h"

#ifndef RRT_H_
#define RRT_H_

class RRT {
private:
	RRTNode _root;
public:
	RRT();
	virtual ~RRT();
	RRTNode getClosestNode(rw::math::Q node);
};

#endif /* RRT_H_ */

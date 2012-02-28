/*
 * RRT.h
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel test
 */
#include "RRTNode.h"
#include <limits>



#include <list>

#ifndef RRT_H_
#define RRT_H_

class RRT {
private:
	std::list<RRTNode *> _tree;
public:
	RRT();
	virtual ~RRT();

	bool addNodeToTree(RRTNode * nodeNew);

	RRTNode * getClosestNode(rw::math::Q node);
};

#endif /* RRT_H_ */

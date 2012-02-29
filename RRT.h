/*
 * RRT.h
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel test
 */
#include "RRTNode.h"
#include <rw/math.hpp>
#include <rw/common.hpp>
#include <limits>

#include <list>

#ifndef RRT_H_
#define RRT_H_

class RRT {
private:
	std::list<rw::common::Ptr<RRTNode> > _tree;
public:
	RRT();
	RRT(rw::math::Q q);
	virtual ~RRT();

	bool addNodeToTree(rw::common::Ptr<RRTNode>);
	rw::common::Ptr<RRTNode> getClosestNode(rw::math::Q node);
};

#endif /* RRT_H_ */

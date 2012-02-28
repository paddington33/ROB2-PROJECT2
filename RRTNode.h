/*
 * RRTNode.h
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel
 */

#include <rw/math.hpp>
#include <rw/math/Q.hpp>

#ifndef RRTNODE_H_
#define RRTNODE_H_

class RRTNode {
private:
	RRTNode* _parrent;
	rw::math::Q _value;
public:
	RRTNode();
	virtual ~RRTNode();

    RRTNode * getParrent() const;
    rw::math::Q getValue() const;

    void setParrent(RRTNode* parrent);
    void setValue(rw::math::Q value);

};

#endif /* RRTNODE_H_ */

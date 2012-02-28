/*
 * RRTNode.cpp
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel
 */

#include "RRTNode.h"

RRTNode::RRTNode() {

}

RRTNode::RRTNode(math::Q value, RRTNode parrent) :
	_value(value),
	_parrent(parrent)
{

}

RRTNode::~RRTNode() {}

RRTNode RRTNode::getParrent() const
{
    return parrent;
}

math::Q RRTNode::getValue() const
{
    return value;
}

void RRTNode::setParrent(RRTNode parrent)
{
    this->parrent = parrent;
}

void RRTNode::setValue(math::Q value)
{
    this->value = value;
}

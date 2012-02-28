/*
 * RRTNode.cpp
 *
 *  Created on: Feb 23, 2012
 *      Author: daniel
 */

#include "RRTNode.h"

RRTNode::RRTNode() :
	_parrent(NULL)
{

}

RRTNode::RRTNode(rw::math::Q value, RRTNode* parrent) :
	_value(value),
	_parrent(parrent)
{

}

RRTNode::~RRTNode() {}

RRTNode* RRTNode::getParrent() const
{
    return _parrent;
}

rw::math::Q RRTNode::getValue() const
{
    return _value;
}

void RRTNode::setParrent(RRTNode * parrent)
{
    this->_parrent = parrent;
}

void RRTNode::setValue(rw::math::Q value)
{
    this->_value = value;
}

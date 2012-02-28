#include "Rob2Project2Plugin.hpp"

#include <QPushButton>
#include <qspinbox.h>
#include <qprogressbar.h>
#include <RobWorkStudio.hpp>

#include <rw/rw.hpp>


#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>

#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/math/Math.hpp>
#include <rw/math/Q.hpp>

#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>

#include <cmath>

#include <iostream>
#include <fstream>
#include <time.h>

#include "RRT.h"


USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;
using namespace rw;

#define Edge std::pair<Q,Q>



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginName", QIcon(":/pa_icon.png"))
{
    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    int row = 0;

    _btn0 = new QPushButton("Run Scene");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn4 = new QPushButton("Run All");
    pLayout->addWidget(_btn4, row++, 0);
    connect(_btn4, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _bar0 = new QProgressBar();
 	_bar0 -> setMaximum(edges);
     _bar0 -> setValue(0);
     pLayout->addWidget(_bar0,row++,0);

    _box0 = new QSpinBox();
    _box0 -> setRange(0,10000000);
    _box0 -> setValue(edges);
    pLayout->addWidget(_box0, row++, 0);
    connect(_box0, SIGNAL(valueChanged(int)), this, SLOT(clickEvent()));

    _btn1 = new QPushButton("Load Kuka scene");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn2 = new QPushButton("Load SceneSimple");
    pLayout->addWidget(_btn2, row++, 0);
    connect(_btn2, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn3 = new QPushButton("Load PA10 scene");
    pLayout->addWidget(_btn3, row++, 0);
    connect(_btn3, SIGNAL(clicked()), this, SLOT(clickEvent()));


    pLayout->setRowStretch(row,1);
}

SamplePlugin::~SamplePlugin(){ /* deallocate used memory */ }
void SamplePlugin::open(WorkCell* workcell){ /* do something when workcell is openned */}
void SamplePlugin::close() { /* do something when the workcell is closed */}

void SamplePlugin::initialize() {
    /* do something when plugin is initialized */
    getRobWorkStudio()->stateChangedEvent().add(
            boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);
}

void SamplePlugin::stateChangedListener(const State& state) {
    log().info() << "State changed!";
}


void SamplePlugin::clickEvent() {

	QObject *obj = sender();

	RRT* tree = new RRT();

}

Q_EXPORT_PLUGIN(SamplePlugin);


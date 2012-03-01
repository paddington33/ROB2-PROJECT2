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
#include <rw/models/Models.hpp>
#include <rw/trajectory/TimedUtil.hpp>

#include <cmath>

#include <iostream>
#include <fstream>
#include <time.h>

#include "RRT.h"
#include "RRTPlanner.h"


USE_ROBWORK_NAMESPACE
using namespace robwork;
using namespace rws;
using namespace rw;

#define Edge std::pair<Q,Q>



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginName", QIcon(":/pa_icon.png"))
{

	std::cout << "1" << std::endl;

	//create planner
	_robWorkStudio = getRobWorkStudio();

	std::cout << "2" << std::endl;

	std::cout << "3" << std::endl;

    QWidget* base = new QWidget(this);
    QGridLayout* pLayout = new QGridLayout(base);
    base->setLayout(pLayout);
    this->setWidget(base);

    int row = 0;

    _btn0 = new QPushButton("Run Planner");
    pLayout->addWidget(_btn0, row++, 0);
    connect(_btn0, SIGNAL(clicked()), this, SLOT(clickEvent()));

    _btn4 = new QPushButton("Run All planner");
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

    _checkbox0 = new QCheckBox();
    pLayout->addWidget(_checkbox0,row,1);
    connect(_checkbox0, SIGNAL(clicked()), this, SLOT(clickEvent()));
    _label0 = new QLabel("Edge Detection");
    pLayout->addWidget(_label0,row++,0);


    _checkbox2 = new QCheckBox();
    pLayout->addWidget(_checkbox2,row,1);
    connect(_checkbox2, SIGNAL(clicked()), this, SLOT(clickEvent()));
    _label2 = new QLabel("Weighted joints");
    pLayout->addWidget(_label2,row++,0);

    _label7 = new QLabel("Connect");
    pLayout->addWidget(_label7,row++,0);

    _box3 = new QSpinBox();
    _box3 -> setRange(-1,10000);
    _box3 -> setValue(10);
    pLayout->addWidget(_box3, row++, 0);
    connect(_box3, SIGNAL(valueChanged(int)), this, SLOT(clickEvent()));

    _checkbox1 = new QCheckBox();
    pLayout->addWidget(_checkbox1,row,1);
    connect(_checkbox1, SIGNAL(clicked()), this, SLOT(clickEvent()));
    _label1 = new QLabel("Connect with pearls");
    pLayout->addWidget(_label1,row++,0);

    _label6 = new QLabel("Swap strategies");
    pLayout->addWidget(_label6,row++,0);

    _combobox0 = new QComboBox();
    pLayout->addWidget(_combobox0,row++,0);


    _combobox0 -> addItem("Loop");
    _combobox0 -> addItem("Random");
    _combobox0 -> addItem("Back'n'Forth");

    _label5 = new QLabel("Number Of trees");
    pLayout->addWidget(_label5,row++,0);
    _box2 = new QSpinBox();
    _box2 -> setRange(0,100);
    _box2 -> setValue(2);
    pLayout->addWidget(_box2, row++, 0);
    connect(_box2, SIGNAL(valueChanged(int)), this, SLOT(clickEvent()));


    _label4 = new QLabel("Epsilon");
    pLayout->addWidget(_label4,row++,0);
    _box1 = new QDoubleSpinBox();
    _box1 -> setDecimals(4);
    _box1 -> setSingleStep(0.0001);
    _box1 -> setRange(0.0001,1);
    _box1 -> setValue(0.01);
    pLayout->addWidget(_box1, row++, 0);
    connect(_box1, SIGNAL(valueChanged(double)), this, SLOT(clickEvent()));


    _btn1 = new QPushButton("Load Kuka scene");
    pLayout->addWidget(_btn1, row++, 0);
    connect(_btn1, SIGNAL(clicked()), this, SLOT(clickEvent()));


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

}


void SamplePlugin::loadScene(std::string scene)
{
	WorkCell::Ptr workCell = WorkCellLoader::load(scene);
	std::cout	<< "Workcell " << *workCell << "successfully loaded." << std::endl;
	RobWorkStudio* robWorkStudio = getRobWorkStudio();
	robWorkStudio->setWorkcell(workCell);
}


void SamplePlugin::clickEvent() {

	QObject *obj = sender();
	if(obj == _btn3){
		loadScene("PA10InGantry/Scene.wc.xml");
	} else if(obj == _btn1){
		loadScene("KukaKr16/Scene.wc.xml");
		_planner = new RRTMTPlanner(_robWorkStudio);
	} else if(obj == _btn0){
		clickEventRRT();
	} else if(obj == _btn4){
		//run many planner
	} else if(obj == _box0){
		//nr of runs spinbox
	} else if(obj == _checkbox0){
		//edge CD
	} else if(obj == _checkbox2){
		//weighted joint
	} else if(obj == _box3){
		//connectN
	} else if(obj == _checkbox1){
		//connect pearl mode
	} else if(obj == _combobox0){
		//swap strategi
	} else if(obj == _box2){
		//nr of tree
	} else if(obj == _box1){
		//epsilon
	}


}



void SamplePlugin::clickEventRRT() {

	using namespace proximity;

	rws::RobWorkStudio* robWorkStudio = getRobWorkStudio();

	rw::common::Ptr<rw::models::WorkCell> workcell = robWorkStudio->getWorkcell();
	rw::models::Device::Ptr device = workcell->findDevice("PA10");

	rw::proximity::CollisionStrategy::Ptr cdstrategy = rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
	CollisionDetector::Ptr collisionDetector = new CollisionDetector(workcell, cdstrategy);

	rw::pathplanning::QConstraint::Ptr constraint = rw::pathplanning::QConstraint::make(
			collisionDetector, device, workcell->getDefaultState());

	QSampler::Ptr cFree = QSampler::makeConstrained(QSampler::makeUniform(device),constraint);

//	RRTPlanner* planner = new RRTPlanner(robWorkStudio);

	RRTPlanner* planner = new RRTMTPlanner(robWorkStudio);
	((RRTMTPlanner*)planner)->setEpsilon(.1);
	((RRTMTPlanner*)planner)->setMinDis(.05);
	((RRTMTPlanner*)planner)->setNumberOfTree(2);


	std::cout << "D 1" << std::endl;

	rw::math::Q qInit = cFree->sample();
	rw::math::Q qGoal = cFree->sample();

	std::cout << "D 2" << std::endl;

	rw::trajectory::QPath path = ((RRTMTPlanner*)planner)->plan(qInit,qGoal);

	std::cout << "D 3" << std::endl;

	rw::kinematics::State state = workcell->getDefaultState();

	std::cout << "D 4" << std::endl;

	std::cout << "D pathLength " << path.size() << std::endl;

	robWorkStudio->setTimedStatePath(
	        TimedUtil::makeTimedStatePath(
	            *workcell,
	            rw::models::Models::getStatePath(*device, path, state)));

	std::cout << "D 5" << std::endl;
}

Q_EXPORT_PLUGIN(SamplePlugin);

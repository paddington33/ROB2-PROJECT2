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

//	std::cout << "1" << std::endl;

	//create planner
	_robWorkStudio = getRobWorkStudio();

//	std::cout << "2" << std::endl;

//	std::cout << "3" << std::endl;

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
 	_bar0 -> setMaximum(100);
     _bar0 -> setValue(0);
     pLayout->addWidget(_bar0,row++,0);

    _box0 = new QSpinBox();
    _box0 -> setRange(0,10000000);
    _box0 -> setValue(100);
    pLayout->addWidget(_box0, row++, 0);
    connect(_box0, SIGNAL(valueChanged(int)), this, SLOT(clickEvent()));

    _checkbox0 = new QCheckBox();
    pLayout->addWidget(_checkbox0,row,1);
    connect(_checkbox0, SIGNAL(clicked()), this, SLOT(clickEvent()));
    _label0 = new QLabel("Edge Detection");
    pLayout->addWidget(_label0,row++,0);

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
    connect(_combobox0, SIGNAL(activated(int)), this, SLOT(clickEvent()));

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


    _label8 = new QLabel("Min. Distance");
    pLayout->addWidget(_label8,row++,0);
    _box4 = new QDoubleSpinBox();
    _box4 -> setDecimals(4);
    _box4 -> setSingleStep(0.0001);
    _box4 -> setRange(0.0001,10);
    _box4 -> setValue(0.05);
    pLayout->addWidget(_box4, row++, 0);
    connect(_box4, SIGNAL(valueChanged(double)), this, SLOT(clickEvent()));


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
	_robWorkStudio = getRobWorkStudio();
	_robWorkStudio->stateChangedEvent().add(
            boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	_planner = new RRTMTPlanner(_robWorkStudio);

    ((RRTMTPlanner*)_planner)->setConnectN(_box3->value());
    ((RRTMTPlanner*)_planner)->setSwapStrategy(_combobox0->currentIndex());
    ((RRTMTPlanner*)_planner)->setNumberOfTree(_box2->value());
    ((RRTMTPlanner*)_planner)->setEpsilon(_box1->value());
	((RRTMTPlanner*)_planner)->setMinDis(_box4->value());
	((RRTMTPlanner*)_planner)->setPearls(_checkbox1->isChecked());
	((RRTMTPlanner*)_planner)->setedgeDetection(_checkbox0->isChecked());

	_numberOfRuns = 100;

}

void SamplePlugin::stateChangedListener(const State& state) {

}


void SamplePlugin::loadScene(std::string scene,std::string deviceName)
{
	WorkCell::Ptr workCell = WorkCellLoader::load(scene);
	std::cout	<< "Workcell " << *workCell << "successfully loaded." << std::endl;
	_robWorkStudio->setWorkcell(workCell);
	_planner->setWorkCell(deviceName);
}


void SamplePlugin::clickEvent() {
	std::cout << "click Event" << std::endl;

	QObject *obj = sender();
	if(obj == _btn3){
		loadScene("PA10InGantry/Scene.wc.xml","PA10");
	} else if(obj == _btn1){
		loadScene("KukaKr16/Scene.wc.xml","KukaKr16");
	} else if(obj == _btn0){	//Run scene
		clickEventRRT();
	} else if(obj == _btn4){
		//run many planner
		runNplanners();
	} else if(obj == _box0){
		//nr of runs spinbox
		_numberOfRuns = (_box0 -> value());
		_bar0->setMaximum(_numberOfRuns);
//		std::cout << _numberOfRuns << std::endl;
	} else if(obj == _checkbox0){
		//edge CD
		((RRTMTPlanner*)_planner)->setedgeDetection(_checkbox0->isChecked());
//		std::cout<< _checkbox0->isChecked() << std::endl;
	} else if(obj == _box3){
		//connectN
		((RRTMTPlanner*)_planner)->setConnectN(_box3->value());
	} else if(obj == _checkbox1){
		//connect pearl mode
		((RRTMTPlanner*)_planner)->setPearls(_checkbox1->isChecked());
//		std::cout<< _checkbox1->isChecked() << std::endl;
	} else if(obj == _combobox0){
		//swap strategi
		((RRTMTPlanner*)_planner)->setSwapStrategy(_combobox0->currentIndex());
//		std::cout << _combobox0->currentIndex() << std::endl;
	} else if(obj == _box2){
		//nr of tree
		((RRTMTPlanner*)_planner)->setNumberOfTree(_box2->value());
	} else if(obj == _box1){
		//epsilon
		((RRTMTPlanner*)_planner)->setEpsilon(_box1->value());
	} else if(obj == _box4){
		//min dis
		((RRTMTPlanner*)_planner)->setMinDis(_box4->value());
	}


}

void SamplePlugin::runNplanners(){

	common::Timer timerPlanner;


	for(int i = 0 ; i < _numberOfRuns ; i++ ) {

		_planner = new RRTMTPlanner(_robWorkStudio);

		_planner->setWorkCell(_robWorkStudio->getWorkcell()->getDevices().at(0)->getName());

		((RRTMTPlanner*)_planner)->setConnectN(_box3->value());
		((RRTMTPlanner*)_planner)->setSwapStrategy(_combobox0->currentIndex());
		((RRTMTPlanner*)_planner)->setNumberOfTree(_box2->value());
		((RRTMTPlanner*)_planner)->setEpsilon(_box1->value());
		((RRTMTPlanner*)_planner)->setMinDis(_box4->value());
		((RRTMTPlanner*)_planner)->setPearls(_checkbox1->isChecked());
		((RRTMTPlanner*)_planner)->setedgeDetection(_checkbox0->isChecked());

		timerPlanner.resume();
		rw::trajectory::QPath path = ((RRTMTPlanner*)_planner)->plan();
		timerPlanner.pause();
		std::cout << "pathLength " << i << " : " << path.size() << " " << std::endl; // path.size() << std::endl;

		//increase progressbar
		_bar0 -> setValue(i+1);

		delete _planner;

	}

	std::cout << "total time: " << timerPlanner.getTime() << "   average time " << timerPlanner.getTime()/_numberOfRuns <<  std::endl;
}



void SamplePlugin::clickEventRRT() {

	using namespace proximity;

	rw::models::Device::Ptr device;
	rw::kinematics::State state;
	rw::trajectory::QPath path;

		_planner = new RRTMTPlanner(_robWorkStudio);

		((RRTMTPlanner*)_planner)->setConnectN(_box3->value());
		((RRTMTPlanner*)_planner)->setSwapStrategy(_combobox0->currentIndex());
		((RRTMTPlanner*)_planner)->setNumberOfTree(_box2->value());
		((RRTMTPlanner*)_planner)->setEpsilon(_box1->value());
		((RRTMTPlanner*)_planner)->setMinDis(_box4->value());
		((RRTMTPlanner*)_planner)->setPearls(_checkbox1->isChecked());
		((RRTMTPlanner*)_planner)->setedgeDetection(_checkbox0->isChecked());


		_planner->setWorkCell(_robWorkStudio->getWorkcell()->getDevices().at(0)->getName());

		path = ((RRTMTPlanner*)_planner)->plan();

		state = _robWorkStudio->getWorkcell()->getDefaultState();

		device = _robWorkStudio->getWorkcell()->getDevices().at(0);

		std::cout << "pathLength " << path.size() << std::endl;

	_robWorkStudio->setTimedStatePath(TimedUtil::makeTimedStatePath(*_robWorkStudio->getWorkcell(),rw::models::Models::getStatePath(*device, path, state)));

}

Q_EXPORT_PLUGIN(SamplePlugin);

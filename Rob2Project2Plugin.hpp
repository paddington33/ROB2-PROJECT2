#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rws/RobWorkStudio.hpp>
#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/math/Q.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/models/WorkCell.hpp>
#include "RRTMTPlanner.h"


class SamplePlugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
int edges;
public:
    SamplePlugin();
	virtual ~SamplePlugin();

	// functions inherited from RobworkStudioPlugin, are typically used but can be optional
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

private slots:
    void clickEvent();
    void clickEventRRT();
    void runNplanners();


    void loadScene(std::string scene, std::string deviceName);

    void stateChangedListener(const rw::kinematics::State& state);

private:
    int _numberOfRuns;
    rws::RobWorkStudio* _robWorkStudio;
    RRTMTPlanner* _planner;
    QComboBox* _combobox0;
    QPushButton* _btn0,*_btn1,*_btn3,*_btn4;
    QSpinBox* _box0,* _box2,* _box3;
    QDoubleSpinBox* _box1,* _box4;
    QProgressBar* _bar0;
    QLabel* _label0,* _label1,* _label2,* _label3,* _label4,* _label5,* _label6,* _label7,* _label8;
    QCheckBox* _checkbox0,* _checkbox1,* _checkbox3;
};

#endif /*SAMPLEPLUGIN_HPP*/

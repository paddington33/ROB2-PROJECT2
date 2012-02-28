#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include <rw/rw.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/math/Q.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/models/WorkCell.hpp>

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

    void stateChangedListener(const rw::kinematics::State& state);

private:
    QPushButton* _btn0,*_btn1,*_btn2,*_btn3,*_btn4;
    QSpinBox* _box0;
    QProgressBar* _bar0;
};

#endif /*SAMPLEPLUGIN_HPP*/

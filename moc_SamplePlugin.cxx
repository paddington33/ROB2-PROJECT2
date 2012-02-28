/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created: Thu Feb 16 14:04:45 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SamplePlugin.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SamplePlugin[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x08,
      33,   27,   13,   13, 0x08,
      90,   77,   13,   13, 0x08,
     142,  136,   13,   13, 0x08,
     165,   13,   13,   13, 0x08,
     192,   13,   13,   13, 0x08,
     247,  216,  211,   13, 0x08,
     328,  216,  211,   13, 0x08,
     409,  216,  211,   13, 0x08,
     487,  216,  211,   13, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SamplePlugin[] = {
    "SamplePlugin\0\0clickEvent()\0state\0"
    "stateChangedListener(rw::kinematics::State)\0"
    "testWorkCell\0edgeCollisionCheck(rw::models::WorkCell::Ptr)\0"
    "scene\0loadScene(std::string)\0"
    "runAlledgeCollisionCheck()\0"
    "updateEdgeNumber()\0bool\0"
    "qStart,qEnd,constraint,counter\0"
    "straightForward(rw::math::Q,rw::math::Q,rw::pathplanning::QConstraint:"
    ":Ptr,int&)\0"
    "uniformStepSize(rw::math::Q,rw::math::Q,rw::pathplanning::QConstraint:"
    ":Ptr,int&)\0"
    "binarySearch(rw::math::Q,rw::math::Q,rw::pathplanning::QConstraint::Pt"
    "r,int&)\0"
    "expandedBinarySearch(rw::math::Q,rw::math::Q,rw::pathplanning::QConstr"
    "aint::Ptr,int&)\0"
};

const QMetaObject SamplePlugin::staticMetaObject = {
    { &rws::RobWorkStudioPlugin::staticMetaObject, qt_meta_stringdata_SamplePlugin,
      qt_meta_data_SamplePlugin, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SamplePlugin::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin))
        return static_cast<void*>(const_cast< SamplePlugin*>(this));
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(const_cast< SamplePlugin*>(this));
    typedef rws::RobWorkStudioPlugin QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef rws::RobWorkStudioPlugin QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: clickEvent(); break;
        case 1: stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        case 2: edgeCollisionCheck((*reinterpret_cast< rw::models::WorkCell::Ptr(*)>(_a[1]))); break;
        case 3: loadScene((*reinterpret_cast< std::string(*)>(_a[1]))); break;
        case 4: runAlledgeCollisionCheck(); break;
        case 5: updateEdgeNumber(); break;
        case 6: { bool _r = straightForward((*reinterpret_cast< rw::math::Q(*)>(_a[1])),(*reinterpret_cast< rw::math::Q(*)>(_a[2])),(*reinterpret_cast< rw::pathplanning::QConstraint::Ptr(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 7: { bool _r = uniformStepSize((*reinterpret_cast< rw::math::Q(*)>(_a[1])),(*reinterpret_cast< rw::math::Q(*)>(_a[2])),(*reinterpret_cast< rw::pathplanning::QConstraint::Ptr(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 8: { bool _r = binarySearch((*reinterpret_cast< rw::math::Q(*)>(_a[1])),(*reinterpret_cast< rw::math::Q(*)>(_a[2])),(*reinterpret_cast< rw::pathplanning::QConstraint::Ptr(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 9: { bool _r = expandedBinarySearch((*reinterpret_cast< rw::math::Q(*)>(_a[1])),(*reinterpret_cast< rw::math::Q(*)>(_a[2])),(*reinterpret_cast< rw::pathplanning::QConstraint::Ptr(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        default: ;
        }
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

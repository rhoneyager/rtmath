/****************************************************************************
** Meta object code from reading C++ file 'frmmain.h'
**
** Created: Fri Jul 20 01:50:34 2012
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "StdAfx.h"
#include "../../frmmain.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'frmmain.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_frmMain[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x08,
      28,   26,    8,    8, 0x08,
      63,    8,    8,    8, 0x08,
      83,    8,    8,    8, 0x08,
     100,    8,    8,    8, 0x08,
     122,    8,    8,    8, 0x08,
     137,    8,    8,    8, 0x08,
     147,    8,    8,    8, 0x08,
     157,    8,    8,    8, 0x08,
     166,    8,    8,    8, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_frmMain[] = {
    "frmMain\0\0allowExport(int)\0,\0"
    "editTreeItem(QTreeWidgetItem*,int)\0"
    "menuGlobals(QPoint)\0menuRots(QPoint)\0"
    "menuScaAngles(QPoint)\0generateRuns()\0"
    "loadSet()\0saveSet()\0newSet()\0import()\0"
};

void frmMain::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        frmMain *_t = static_cast<frmMain *>(_o);
        switch (_id) {
        case 0: _t->allowExport((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 1: _t->editTreeItem((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: _t->menuGlobals((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 3: _t->menuRots((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 4: _t->menuScaAngles((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 5: _t->generateRuns(); break;
        case 6: _t->loadSet(); break;
        case 7: _t->saveSet(); break;
        case 8: _t->newSet(); break;
        case 9: _t->import(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData frmMain::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject frmMain::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_frmMain,
      qt_meta_data_frmMain, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &frmMain::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *frmMain::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *frmMain::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_frmMain))
        return static_cast<void*>(const_cast< frmMain*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int frmMain::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

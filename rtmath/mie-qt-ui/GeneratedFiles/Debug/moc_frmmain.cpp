/****************************************************************************
** Meta object code from reading C++ file 'frmmain.h'
**
** Created: Wed May 18 16:28:52 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "StdAfx.h"
#include "..\..\frmmain.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'frmmain.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_frmMain[] = {

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
       9,    8,    8,    8, 0x0a,
      18,    8,    8,    8, 0x0a,
      31,    8,    8,    8, 0x0a,
      47,    8,    8,    8, 0x0a,
      63,    8,    8,    8, 0x0a,
      72,    8,    8,    8, 0x0a,
      82,    8,    8,    8, 0x0a,
      92,    8,    8,    8, 0x0a,
     108,    8,    8,    8, 0x0a,
     125,    8,    8,    8, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_frmMain[] = {
    "frmMain\0\0jobRun()\0jobItemAdd()\0"
    "jobItemDelete()\0jobItemModify()\0"
    "jobNew()\0jobOpen()\0jobSave()\0"
    "jobSaveOutput()\0jobExportImage()\0"
    "imageProperties()\0"
};

const QMetaObject frmMain::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_frmMain,
      qt_meta_data_frmMain, 0 }
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
        switch (_id) {
        case 0: jobRun(); break;
        case 1: jobItemAdd(); break;
        case 2: jobItemDelete(); break;
        case 3: jobItemModify(); break;
        case 4: jobNew(); break;
        case 5: jobOpen(); break;
        case 6: jobSave(); break;
        case 7: jobSaveOutput(); break;
        case 8: jobExportImage(); break;
        case 9: imageProperties(); break;
        default: ;
        }
        _id -= 10;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

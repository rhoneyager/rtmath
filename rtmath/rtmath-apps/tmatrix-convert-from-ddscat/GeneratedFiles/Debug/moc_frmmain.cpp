/****************************************************************************
** Meta object code from reading C++ file 'frmmain.h'
**
** Created: Thu Aug 16 01:46:41 2012
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../frmmain.h"
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
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
       9,    8,    8,    8, 0x0a,
      22,    8,    8,    8, 0x0a,
      39,    8,    8,    8, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_frmMain[] = {
    "frmMain\0\0doGenerate()\0findDefaultPar()\0"
    "findBaseDir()\0"
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
        case 0: doGenerate(); break;
        case 1: findDefaultPar(); break;
        case 2: findBaseDir(); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

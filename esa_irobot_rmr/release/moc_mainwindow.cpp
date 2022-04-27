/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[22];
    char stringdata0[311];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 15), // "uiValuesChanged"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 14), // "newInstruction"
QT_MOC_LITERAL(4, 43, 16), // "uiValuesChangedR"
QT_MOC_LITERAL(5, 60, 9), // "newrobotX"
QT_MOC_LITERAL(6, 70, 9), // "newrobotY"
QT_MOC_LITERAL(7, 80, 24), // "on_pushButton_12_clicked"
QT_MOC_LITERAL(8, 105, 26), // "navigate_to_selected_point"
QT_MOC_LITERAL(9, 132, 6), // "Xbunka"
QT_MOC_LITERAL(10, 139, 6), // "Ybunka"
QT_MOC_LITERAL(11, 146, 19), // "on_checkBox_clicked"
QT_MOC_LITERAL(12, 166, 7), // "checked"
QT_MOC_LITERAL(13, 174, 28), // "on_checkBox_skeleton_clicked"
QT_MOC_LITERAL(14, 203, 18), // "on_Mapping_clicked"
QT_MOC_LITERAL(15, 222, 18), // "on_showMap_clicked"
QT_MOC_LITERAL(16, 241, 18), // "on_showCam_clicked"
QT_MOC_LITERAL(17, 260, 11), // "setUiValues"
QT_MOC_LITERAL(18, 272, 11), // "instruction"
QT_MOC_LITERAL(19, 284, 12), // "setUiValuesR"
QT_MOC_LITERAL(20, 297, 6), // "robotX"
QT_MOC_LITERAL(21, 304, 6) // "robotY"

    },
    "MainWindow\0uiValuesChanged\0\0newInstruction\0"
    "uiValuesChangedR\0newrobotX\0newrobotY\0"
    "on_pushButton_12_clicked\0"
    "navigate_to_selected_point\0Xbunka\0"
    "Ybunka\0on_checkBox_clicked\0checked\0"
    "on_checkBox_skeleton_clicked\0"
    "on_Mapping_clicked\0on_showMap_clicked\0"
    "on_showCam_clicked\0setUiValues\0"
    "instruction\0setUiValuesR\0robotX\0robotY"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   69,    2, 0x06 /* Public */,
       4,    2,   72,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    0,   77,    2, 0x08 /* Private */,
       8,    2,   78,    2, 0x08 /* Private */,
      11,    1,   83,    2, 0x08 /* Private */,
      13,    1,   86,    2, 0x08 /* Private */,
      14,    1,   89,    2, 0x08 /* Private */,
      15,    1,   92,    2, 0x08 /* Private */,
      16,    1,   95,    2, 0x08 /* Private */,
      17,    1,   98,    2, 0x0a /* Public */,
      19,    2,  101,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,    5,    6,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Bool, QMetaType::Int, QMetaType::Int,    9,   10,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::QString,   18,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   20,   21,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->uiValuesChanged((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->uiValuesChangedR((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 2: _t->on_pushButton_12_clicked(); break;
        case 3: { bool _r = _t->navigate_to_selected_point((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 4: _t->on_checkBox_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->on_checkBox_skeleton_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->on_Mapping_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->on_showMap_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->on_showCam_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->setUiValues((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->setUiValuesR((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MainWindow::*)(QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::uiValuesChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (MainWindow::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::uiValuesChangedR)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::uiValuesChanged(QString _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MainWindow::uiValuesChangedR(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

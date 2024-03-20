QT += core gui widgets

CONFIG += c++11

SOURCES += \
    main.cpp \
    mainwindow.cpp \
    gripper/zimmergripper.cpp \
    customsettings.cpp \
	tcpsocket.cpp \

HEADERS += \
    mainwindow.h \
    gripper/zimmergripper.h \
    customsettings.h \
	tcpsocket.h \
	robot/libcustom/Signal.hpp \
	robot/libcustom/tcpclient.h \
	robot/libcustom/timer.h \
	robot/robotapi/robot.h \
	robot/robotapi/m1013_v3.h \
	robot/robotconf.h \
	robot/sdkv2.h \

FORMS += mainwindow.ui

LIBS += -lmodbus
LIBS += -L$$PWD/robot/ -lrobotsdkv3

INCLUDEPATH += $$PWD/robot
DEPENDPATH += $$PWD/robot

LIBS += -L$$PWD/robot/robotapi/doosanapi/library/ -lDRFL
LIBS += -L$$PWD/robot/robotapi/doosanapi/library/ -lPocoNet
LIBS += -L$$PWD/robot/robotapi/doosanapi/library/ -lPocoFoundation

INCLUDEPATH += $$PWD/robot/robotapi/doosanapi/library
DEPENDPATH += $$PWD/robot/robotapi/doosanapi/library

PRE_TARGETDEPS += $$PWD/robot/robotapi/doosanapi/library/libDRFL.a

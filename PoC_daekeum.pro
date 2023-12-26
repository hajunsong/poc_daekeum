QT += core gui widgets

CONFIG += c++11

SOURCES += \
	doosan.cpp \
    main.cpp \
    mainwindow.cpp \
    gripper/zimmergripper.cpp \
    customsettings.cpp \
	tcpsocket.cpp \

HEADERS += \
	doosan.h \
	doosanapi/include/DRFC.h \
	doosanapi/include/DRFL.h \
	doosanapi/include/DRFLEx.h \
	doosanapi/include/DRFS.h \
    mainwindow.h \
    gripper/zimmergripper.h \
    customsettings.h \
	tcpsocket.h \

FORMS += mainwindow.ui

LIBS += -lmodbus

INCLUDEPATH += $$PWD/doosanapi/include
DEPENDPATH += $$PWD/doosanapi/include

LIBS += -L$$PWD/doosanapi/library/Linux/64bits/20.04/ -lPocoNet
LIBS += -L$$PWD/doosanapi/library/Linux/64bits/20.04/ -lPocoFoundation
LIBS += -L$$PWD/doosanapi/library/Linux/64bits/20.04/ -lDRFL

PRE_TARGETDEPS += $$PWD/doosanapi/library/Linux/64bits/20.04/libDRFL.a

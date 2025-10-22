#-------------------------------------------------
#
# Project created by QtCreator 2013-10-14T14:34:02
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = RoomLearningTool
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    mypanel.cpp \
    myqlabel.cpp \
    pugixml.cpp

HEADERS  += mainwindow.h \
    mypanel.h \
    myqlabel.h \
    pugixml.hpp \
    pugiconfig.hpp

FORMS    += mainwindow.ui

RESOURCES += \
    resource.qrc
    

QT += widgets

CONFIG += c++11

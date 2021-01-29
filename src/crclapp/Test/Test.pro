QT += core
QT -= gui

CONFIG += c++11

TARGET = CrclTest
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app


QMAKE_CXXFLAGS +=-std=c++11
QMAKE_CXXFLAGS +=-Wno-unused-variable
QMAKE_CXXFLAGS +=-Wno-sign-compare
QMAKE_CXXFLAGS +=-Wno-unused-parameter
QMAKE_CXXFLAGS +=-Wno-reorder
QMAKE_CXXFLAGS +=-Wno-format-extra-args
QMAKE_CXXFLAGS +=-Wno-unused-local-typedefs
QMAKE_CXXFLAGS +=-Wno-ignored-qualifiers
QMAKE_CXXFLAGS +=-Wno-deprecated-declarations
QMAKE_CXXFLAGS +=-Wno-unused-function
QMAKE_CXXFLAGS +=-Wno-unused-but-set-variable
QMAKE_CXXFLAGS +=-Wno-write-strings
QMAKE_CXXFLAGS +=-Wno-missing-field-initializers
QMAKE_CXXFLAGS +=-DDEBUG
DEFINES += QT_NO_VERSION_TAGGING

INCLUDEPATH += "$$(HOME)/src/robot-agility/gz/installs/ros_pkgs/include"

INCLUDEPATH += "/usr/include/eigen3"
INCLUDEPATH += "/usr/include/gazebo-7"
INCLUDEPATH += "/usr/include/sdformat-4.0"
INCLUDEPATH += "/usr/include/ignition/math2"

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"


# Local ROS libs
LIBS += -L$$(HOME)/src/robot-agility/gz/installs/ros_pkgs/lib
LIBS +=  -lcrcllib


# Boost
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lboost_system
LIBS += -lboost_thread


# Ros libs
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib -lpthread -ltf2_ros -ltf_conversions

SOURCES += main.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


config_features.path     = "$$OUT_PWD/config"
config_features.files     = $$PWD/config/FanucLRMate200iD.urdf \
 $$PWD/config/Config.ini

message("mkspecs $$config_features.path")
message("mkspecs files $$config_features.files")
INSTALLS                  += config_features

DISTFILES += \
    Notes

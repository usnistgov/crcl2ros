
QT += core
QT -= gui

gzversion=7

TARGET = crclapp
CONFIG += console
CONFIG -= app_bundle
TEMPLATE = app
message("Compiling crclapp application")


CONFIG +=  c++11
release: DESTDIR = release
debug:   DESTDIR = debug

OBJECTS_DIR = $$DESTDIR/.obj
MOC_DIR = $$DESTDIR/.moc
RCC_DIR = $$DESTDIR/.qrc
#UI_DIR = $$DESTDIR/.ui

DEFINES+=QT_NO_VERSION_TAGGING
DEFINES+=DEBUG
DEFINES+=GAZEBO
#DEFINES+=ROSMSG



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
QMAKE_CXXFLAGS +=-std=c++11
QMAKE_CXXFLAGS +=-Wno-format-security


QMAKE_LFLAGS += -g

INCLUDEPATH += "./include/crclapp"


INCLUDEPATH += "./include/crclapp/CrclXsd"
INCLUDEPATH += "./include"
INCLUDEPATH += "./src"
INCLUDEPATH += "./src/CRCL"
# Eigen - header only
INCLUDEPATH += "/usr/include/eigen3"

#ROS
INCLUDEPATH += "/opt/ros/kinetic/include"

# Local  libs
LIBS += -L../../../lib
#LIBS +=  -lgotraj


# Boost - many could be replace by C11 std
LIBS += -L/usr/lib/x86_64-linux-gnu
LIBS += -lboost_system
LIBS += -lboost_chrono
LIBS += -lboost_thread
LIBS += -lboost_filesystem
LIBS += -lboost_date_time
LIBS += -lboost_regex
LIBS += -lboost_log_setup
LIBS += -lboost_log
LIBS += -lboost_locale

# GNU readline
LIBS += -lreadline

# Ros libs
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lcpp_common -lroslib -lrosconsole -lrostime -lrospack
LIBS += -lurdf -lurdfdom_sensor -lurdfdom_model_state -lurdfdom_model -lurdfdom_world
LIBS +=  -ltf -ltf2 -lclass_loader
LIBS +=   -lxmlrpcpp -lroscpp_serialization
LIBS +=  -lrosconsole_bridge  -lrosconsole_log4cxx  -lrosconsole_backend_interface
LIBS += -lactionlib -lpthread -ltf2_ros -ltf_conversions

# Code Synthesis
# xerces code synthesis dependency - no XML parsing
# Static lib
#LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"

# xerces code synthesis dependency
#LIBS += "/usr/lib/x86_64-linux-gnu/libxerces-c.a"
LIBS +=  -lxerces-c

# MOVEIT
LIBS += -lactionlib
LIBS += -lapr-1
LIBS += -laprutil-1
LIBS += -lassimp
LIBS += -lc
LIBS += -lccd
LIBS += -lclass_loader
LIBS += -lcom_err
LIBS += -lconsole_bridge
LIBS += -lcpp_common
LIBS += -lcrypt
LIBS += -lcrypto
LIBS += -lcurl
LIBS += -ldl
LIBS += -ldrm
LIBS += -ldynamic_reconfigure_config_init_mutex
LIBS += -leigen_conversions
LIBS += -lexpat
LIBS += -lfcl
LIBS += -lfreeimage
LIBS += -lfreetype
LIBS += -lgcc_s
LIBS += -lgeometric_shapes
LIBS += -lGL
LIBS += -lglapi
LIBS += -lglib-2.0
LIBS += -lgmp
LIBS += -lgobject-2.0
LIBS += -lgomp
LIBS += -lharfbuzz
LIBS += -lICE
LIBS += -licudata
LIBS += -licui18n
LIBS += -licuuc
LIBS += -limage_transport
LIBS += -linteractive_markers
LIBS += -ljasper
LIBS += -ljbig
LIBS += -ljpeg
LIBS += -lkdl_conversions
LIBS += -llaser_geometry
LIBS += -llog4cxx
LIBS += -llzma
LIBS += -lm
LIBS += -lmessage_filters
LIBS += -lmoveit_collision_detection
LIBS += -lmoveit_collision_detection_fcl
LIBS += -lmoveit_collision_plugin_loader
LIBS += -lmoveit_common_planning_interface_objects
LIBS += -lmoveit_exceptions
LIBS += -lmoveit_kinematic_constraints
LIBS += -lmoveit_kinematics_base
LIBS += -lmoveit_kinematics_plugin_loader
LIBS += -lmoveit_move_group_interface
LIBS += -lmoveit_occupancy_map_monitor
LIBS += -lmoveit_planning_interface
LIBS += -lmoveit_planning_scene
LIBS += -lmoveit_planning_scene_interface
LIBS += -lmoveit_planning_scene_monitor
LIBS += -lmoveit_profiler
LIBS += -lmoveit_profiler
LIBS += -lmoveit_rdf_loader
LIBS += -lmoveit_robot_model
LIBS += -lmoveit_robot_model_loader
LIBS += -lmoveit_robot_state
LIBS += -lmoveit_robot_trajectory
LIBS += -lmoveit_trajectory_execution_manager
LIBS += -lmoveit_transforms
LIBS += -lmoveit_visual_tools
LIBS += -lmoveit_warehouse
LIBS += -loctomap
LIBS += -loctomath
LIBS += -lOgreMain
LIBS += -lOgreOverlay
LIBS += -lorocos-kdl
LIBS += -lpcre
LIBS += -lpcre16
LIBS += -lpng12
LIBS += -lPocoFoundation
LIBS += -lpthread
LIBS += -lpython2.7
LIBS += -lqhull
LIBS += -lQt5Core
LIBS += -lQt5Gui
LIBS += -lQt5Widgets
LIBS += -lrandom_numbers
LIBS += -lresolv
LIBS += -lresource_retriever
LIBS += -lrosconsole
LIBS += -lrosconsole_backend_interface
LIBS += -lrosconsole_bridge
LIBS += -lrosconsole_log4cxx
LIBS += -lroscpp
LIBS += -lroscpp
LIBS += -lroscpp_serialization
LIBS += -lroslib
LIBS += -lrospack
LIBS += -lrostime
LIBS += -lrt
LIBS += -lrviz
LIBS += -lrviz_default_plugin
LIBS += -lrviz_visual_tools
LIBS += -lrviz_visual_tools_gui
LIBS += -lrviz_visual_tools_remote_control
LIBS += -lSM
LIBS += -lsqlite3
LIBS += -lsrdfdom
LIBS += -lssl
LIBS += -lstdc++
LIBS += -ltf
LIBS += -ltf_conversions
LIBS += -ltf2
LIBS += -ltf2_ros
LIBS += -ltiff
LIBS += -ltinyxml
LIBS += -ltinyxml2
LIBS += -lurdf
LIBS += -lurdfdom_model
LIBS += -lurdfdom_model_state
LIBS += -lurdfdom_sensor
LIBS += -lurdfdom_world
LIBS += -lutil
LIBS += -luuid
LIBS += -lwarehouse_ros
LIBS += -lwebp
LIBS += -lwebpmux
LIBS += -lX11
LIBS += -lX11-xcb
LIBS += -lXau
LIBS += -lxcb
LIBS += -lxcb-dri2
LIBS += -lxcb-dri3
LIBS += -lxcb-glx
LIBS += -lxcb-present
LIBS += -lxcb-sync
LIBS += -lXdamage
LIBS += -lXdmcp
LIBS += -lXext
LIBS += -lXfixes
LIBS += -lxmlrpcpp
LIBS += -lXmu
LIBS += -lXpm
LIBS += -lxshmfence
LIBS += -lXt
LIBS += -lXxf86vm
LIBS += -lyaml-cpp
LIBS += -lz

SOURCES += \
    src/CRCL/CRCLCommandInstance.cxx \
    src/CRCL/CRCLCommands.cxx \
    src/CRCL/CRCLProgramInstance.cxx \
    src/CRCL/CRCLStatus.cxx \
    src/CRCL/DataPrimitives.cxx \
    src/CommandLineInterface.cpp \
    src/Globals.cpp \
    src/main.cpp \
    src/CrclWm.cpp \
    src/Demo.cpp \
    src/Shape.cpp \
    src/CrclPublisherInterface.cpp \
    src/CrclSubscriberInterface.cpp \
    src/MTCSOCKET/client.cpp \
    src/MTCSOCKET/socketserver.cpp \
    src/Ros.cpp \
    src/CrclSocketServer.cpp \
    src/Crcl2RosMsgApi.cpp \
    src/RosMsgHandler.cpp \
    src/CrclRobotImpl.cpp \
    src/CrclPrimitives.cpp

HEADERS += \
    include/crclapp/CrclXsd/CRCLCommandInstance.hxx \
    include/crclapp/CrclXsd/CRCLCommands.hxx \
    include/crclapp/CrclXsd/CRCLProgramInstance.hxx \
    include/crclapp/CrclXsd/CRCLStatus.hxx \
    include/crclapp/CrclXsd/DataPrimitives.hxx \
    include/crclapp/CommandLineInterface.h \
    include/crclapp/Globals.h \
    include/crclapp/CrclWm.h \
    include/crclapp/Demo.h \
    include/crclapp/Shape.h \
    include/crclapp/CrclSubscriberInterface.h \
    include/crclapp/CrclPublisherInterface.h \
    include/crclapp/MTCSOCKET/client.hpp \
    include/crclapp/MTCSOCKET/internal.hpp \
    include/crclapp/MTCSOCKET/socketserver.h \
    include/crclapp/Ros.h \
    include/crclapp/gripper.h \
    include/crclapp/rcs/math.h \
    include/crclapp/CrclSocketServer.h \
    include/crclapp/Crcl2RosMsgApi.h \
    include/crclapp/RosMsgHandler.h \
    include/crclapp/CrclRobotImpl.h \
    include/crclapp/CrclPrimitives.h \
    include/crclapp/rcs/RCSMsgQueue.h \
    include/crclapp/rcs/RCSMsgQueueThread.h \
    include/crclapp/rcs/RCSPriorityQueue.h \
    include/crclapp/rcs/RCSThreadTemplate.h \
    include/crclapp/rcs/RCSTimer.h

DISTFILES += \
    include/nistcrcl/CrclXsd/CRCLCommandInstance.xsd \
    include/nistcrcl/CrclXsd/CRCLCommands.xsd \
    include/nistcrcl/CrclXsd/CRCLProgramInstance.xsd \
    include/nistcrcl/CrclXsd/CRCLStatus.xsd \
    include/nistcrcl/CrclXsd/DataPrimitives.xsd \
    include/nistcrcl/CrclXsd/CreateTree.bash \
    include/nistcrcl/NIST/RCSTimer.txt \
    Notes \
    notes

# Hard to make distinction between general build and command line qmake build
build_features.path     = "../../../install/lib/crclapp/config"
#build_features.path     = "$$OUT_PWD/$$DESTDIR/config"
build_features.files     =    $$PWD/config/Config.ini \
   $$PWD/config/MotomanSia20d.urdf\
   $$PWD/config/FanucLRMate200iD.urdf\
   $$PWD/config/lrmate200id.urdf\
   $$PWD/config/motoman_sia20d.ini\
   $$PWD/config/fanuc-lrmate-200id.ini

INSTALLS  += build_features

# THis moves not copies executable from build to install
target.path = ../../../install/lib/crclapp
INSTALLS += target



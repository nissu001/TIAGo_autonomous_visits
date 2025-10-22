cmake_minimum_required(VERSION 2.8.3)
project(places_learning_gui)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pugixml
  roscpp
  qt_build
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS pugixml roscpp qt_build
#  DEPENDS system_lib
)

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##############################################################################
## Qt Environment
##############################################################################

## Find Qt5
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

## Inclusion de directorios
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Se establecen las variables de fuentes y encabezados
set(qt_srcs
  src/mainwindow.cpp
  src/myqlabel.cpp
  src/mypanel.cpp
  src/qnode.cpp
  src/mydialog.cpp
)
set(qt_hdrs
  include/PlacesLearningGui/mainwindow.h
  include/PlacesLearningGui/myqlabel.h
  include/PlacesLearningGui/mypanel.h
  include/PlacesLearningGui/qnode.hpp
  include/PlacesLearningGui/ui_mainwindow.h
  include/PlacesLearningGui/mydialog.h
)

## Generar archivos MOC
qt5_wrap_cpp(qt_moc_srcs ${qt_hdrs})

## Generar archivos UI
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})

## Añadir el ejecutable de tu proyecto
add_executable(places_learning_gui
  src/PlacesLearningGui.cpp
  ${qt_srcs}
  ${qt_moc_srcs}
  ${QT_FORMS_HPP}
)

## Enlazar con las bibliotecas de Qt5 y las bibliotecas de ROS
target_link_libraries(places_learning_gui
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5OpenGL
  GL
  glut
)





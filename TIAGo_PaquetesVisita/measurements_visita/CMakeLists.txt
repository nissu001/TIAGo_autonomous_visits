cmake_minimum_required(VERSION 3.0.2)
project(measurements_visita)

##activar esta linea al usar en tiago y desactivarla en ordenador personal, sirve para desactivar una bandera de tiago que crea errores de compilacion en el codigo
##disable_pal_flags()


# DEPENDENCIAS :  (mismas que en package.xml)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  geometry_msgs
  
  tf
  actionlib
)



catkin_package(
  CATKIN_DEPENDS roscpp
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)



# measurements_visita node
add_executable(measurements_visita src/cmeasurements_visita.cpp src/measurements_visita.cpp)
add_dependencies(measurements_visita ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(measurements_visita ${catkin_LIBRARIES})


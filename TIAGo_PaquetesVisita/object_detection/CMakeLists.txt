cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

##activar esta linea al usar en tiago y desactivarla en ordenador personal, sirve para desactivar una bandera de tiago que crea errores de compilacion en el codigo
##disable_pal_flags()


# DEPENDENCIAS :  (mismas que en package.xml)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  pal_interaction_msgs
  actionlib_msgs
  yolov8_ros_msgs  
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



# object_detection node
add_executable(object_detection src/c_object_detection.cpp src/object_detection.cpp)
add_dependencies(object_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_detection ${catkin_LIBRARIES})


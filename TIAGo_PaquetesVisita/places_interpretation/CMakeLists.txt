cmake_minimum_required(VERSION 2.8.3)
project(places_interpretation)

find_package(catkin REQUIRED COMPONENTS
  mensajes
  pugixml
  roscpp
  actionlib
  move_base_msgs
  tf
)


##add_message_files(
  ##FILES
  ##Place.msg
  ##Point2D.msg
  ##PlacesVector.msg
  ##Room.msg
  ##RoomsVector.msg
  ##Pose2DStamped.msg
##)

generate_messages(
  DEPENDENCIES
  mensajes
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS mensajes pugixml roscpp 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(places_interpretation src/cPlacesInterpretation.cpp src/PlacesInterpretation.cpp)
add_dependencies(places_interpretation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(places_interpretation ${catkin_LIBRARIES})

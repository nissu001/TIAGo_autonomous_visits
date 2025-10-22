cmake_minimum_required(VERSION 3.0.2)
project(mensajes)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Place.msg
  Point2D.msg
  PlacesVector.msg
  Room.msg
  RoomsVector.msg
  Pose2DStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

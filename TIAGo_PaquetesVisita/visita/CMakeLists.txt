cmake_minimum_required(VERSION 2.8.3)
project(visita)

##activar esta linea al usar en tiago y desactivarla en ordenador personal, sirve para desactivar una bandera de tiago que crea errores de compilacion en el codigo
##disable_pal_flags()

find_package(catkin REQUIRED COMPONENTS
  mensajes
  pugixml
  roscpp
  actionlib
  move_base_msgs
  tf
  actionlib_msgs
  pal_interaction_msgs
  roslib
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS mensajes pugixml roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



# places_interpretation node
add_executable(places_interpretation_visita src/cPlacesInterpretation.cpp src/PlacesInterpretation.cpp)
add_dependencies(places_interpretation_visita ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(places_interpretation_visita ${catkin_LIBRARIES})

# sending_goals node
add_executable(sending_goals_visita src/sendingGoals2.cpp)
add_dependencies(sending_goals_visita ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sending_goals_visita ${catkin_LIBRARIES})

add_executable(visita src/cvisita.cpp src/visita.cpp)
add_dependencies(visita ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(visita ${catkin_LIBRARIES})



include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(inf3480_move_robot src/inf3480_move_robot.cpp)

target_link_libraries(inf3480_move_robot
  ${catkin_LIBRARIES}
)


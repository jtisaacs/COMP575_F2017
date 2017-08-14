cmake_minimum_required(VERSION 2.8.3)
project(mobility)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  random_numbers
  message_generation
  shared_messages
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs random_numbers message_runtime shared_messages
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(
	mobility  
	src/mobility.cpp
  	src/PIDController.cpp
	src/PIDError.cpp
	src/RotationalError.cpp
	src/TranslationalError.cpp
	src/Saturation.cpp
	src/RotationalController.cpp
	src/TranslationalController.cpp
	src/SearchController.cpp
	src/TargetState.cpp
)

add_dependencies(mobility ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  mobility
  ${catkin_LIBRARIES}
)


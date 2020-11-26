cmake_minimum_required(VERSION 3.0.2)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(simple_trajectory)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
include_directories(${PROJECT_SOURCE_DIR}/include/simple_trajectory)

find_package(catkin REQUIRED COMPONENTS
  jointspace
  actionlib
  control_msgs
  geometry_msgs
  roscpp

)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES simple_trajectory
  CATKIN_DEPENDS actionlib control_msgs geometry_msgs roscpp jointspace
# DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

#set(${PROJECT_NAME}_node simple_trajectory.cpp)

add_executable(${PROJECT_NAME}_node src/simple_trajectory.cpp)
add_dependencies(${PROJECT_NAME}_node jointspace_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES})


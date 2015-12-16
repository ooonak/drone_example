cmake_minimum_required(VERSION 2.8.3)
project(drone_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs 
)

catkin_package(
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(fly src/fly.cpp)
add_executable(kbd src/kbd.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(fly ${catkin_LIBRARIES})
target_link_libraries(kbd ${catkin_LIBRARIES})

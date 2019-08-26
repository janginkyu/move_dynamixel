cmake_minimum_required(VERSION 2.8.3)
project(move_dynamixel)
find_package(catkin REQUIRED cmake_modules)
find_package(catkin REQUIRED COMPONENTS
    roscpp std_msgs dynamixel_sdk
)

catkin_package()
#MESSAGE(FATAL_ERROR ${dynamixel_sdk_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} ${dynamixel_sdk_INCLUDE_DIRS})
add_executable(move_dynamixel src/main.cpp src/move_dynamixel.cpp)
target_link_libraries(move_dynamixel ${dynamixel_sdk_LIBRARIES})
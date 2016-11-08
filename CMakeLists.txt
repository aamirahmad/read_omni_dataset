cmake_minimum_required(VERSION 2.8.3)
project(read_omni_dataset)


find_package(cmake_modules REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs 
  tf 
  geometry_msgs     
  nav_msgs
  message_generation
)

INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BallData.msg
  LRMLandmarksData.msg
  LRMGTData.msg
  RobotState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs     
  nav_msgs
)

catkin_package(
INCLUDE_DIRS include
#  CATKIN_DEPENDS roscpp rospy
CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# elseif(COMPILER_SUPPORTS_CXX0X)
# 	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
# else()
#         message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${Eigen_INCLUDE_DIRS})

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(include)

add_library(read_omni_dataset src/read_omni_dataset.cpp)
target_link_libraries(read_omni_dataset ${catkin_LIBRARIES} ${rosbag_LIBRARIES})

#add_executable(read_omni_dataset_node src/read_omni_dataset.cpp)
#add_dependencies(read_omni_dataset read_omni_dataset_generate_messages_cpp)
#target_link_libraries(read_omni_dataset_node ${catkin_LIBRARIES} ${rosbag_LIBRARIES} read_omni_dataset)

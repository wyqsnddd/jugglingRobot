cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
#if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
#  add_definitions("-fno-strict-aliasing -lstdc++ -O4") 
#endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/FindGUROBI.cmake)
find_package(GUROBI REQUIRED)
include_directories(${GUROBI_INCLUDE_DIRS})
link_directories(${GUROBI_LIBRARIES})
include_directories(${GUROBI_INCLUDE_DIRS})
message(GUROBI_LIBRARIES)
message(${GUROBI_LIBRARIES})
message(GUROBI_LIBRARY)
message(${GUROBI_LIBRARY})

set(gurobi_libraries pthread m gurobi60 gurobi_c++ )

# For Eigen
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIRS})

set(BOOST_LIBS_ME system thread date_time filesystem iostreams python )
rosbuild_add_boost_directories()

include_directories(${PROJECT_SOURCE_DIR}/common/include)
link_directories(${PROJECT_SOURCE_DIR}/ros/lib)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


rosbuild_add_library(robotPaddlingController
  common/src/robotPaddlingController.cpp)
rosbuild_link_boost(robotPaddlingController ${BOOST_LIBS_ME}) 
target_link_libraries(robotPaddlingController ${GUROBI_LIBRARIES})

rosbuild_add_library(rostopicCommunication
  common/src/rostopicCommunication.cpp)
rosbuild_link_boost(rostopicCommunication ${BOOST_LIBS_ME}) 


rosbuild_add_executable(rosPaddlingControlNode
  common/src/robotPaddlingControlNode.cpp
  common/src/robotPaddlingController.cpp
  common/src/rostopicCommunication.cpp)
rosbuild_link_boost(rosPaddlingControlNode ${BOOST_LIBS_ME}) 


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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/ros/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
#  add_definitions(" -fno-strict-aliasing -lstdc++ -O4") # This is used for final excution 
   add_definitions(" -fPIC -fno-strict-aliasing -Wall -lstdc++ ") # This is used for fast developing/prototyping 
endif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)

include_directories(${PROJECT_SOURCE_DIR}/common/include)
link_directories(${PROJECT_SOURCE_DIR}/ros/lib)


#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/FindGUROBI.cmake)
find_package(GUROBI REQUIRED)

include_directories(${GUROBI_INCLUDE_DIRS})
link_directories(${GUROBI_LIBRARIES})
include_directories(${GUROBI_INCLUDE_DIRS})
include_directories(${GUROBI_INCLUDE_DIRS})

# For Eigen
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/FindEigen.cmake)
include_directories(${EIGEN_INCLUDE_DIRS})

set(BOOST_LIBS_ME system thread date_time filesystem iostreams python )
rosbuild_add_boost_directories()

rosbuild_add_library(singleArmQuaternionConstraint
                                       common/src/singleArmQuaternionConstraint.cpp)
rosbuild_link_boost(singleArmQuaternionConstraint ${BOOST_LIBS_ME})
target_link_libraries(singleArmQuaternionConstraint ${GUROBI_LIBRARIES})


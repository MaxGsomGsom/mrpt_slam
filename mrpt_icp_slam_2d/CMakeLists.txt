# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)
project(mrpt_icp_slam_2d)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.

#find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  mrpt_bridge
  dynamic_reconfigure
  roslaunch
  geometry_msgs
  message_generation)

#find mrpt packages
find_package(MRPT 1.5 REQUIRED)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
if (MRPT_VERSION VERSION_LESS 1.9.9)
	find_package(MRPT REQUIRED base gui obs slam)
else()
	find_package(MRPT REQUIRED gui obs slam)
endif()


if (CMAKE_COMPILER_IS_GNUCXX)
	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
ENDIF()

#Include directories
INCLUDE_DIRECTORIES(
   ${CMAKE_SOURCE_DIR}/src
)

add_message_files(
  FILES
  ObservationWithTransform.msg
)

generate_messages(DEPENDENCIES 
  mrpt_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS
	nav_msgs sensor_msgs std_msgs geometry_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)



add_executable(mrpt_icp_slam_2d
    src/mrpt_icp_slam_2d_wrapper.cpp
    src/mrpt_icp_slam_2d_app.cpp
)

TARGET_LINK_LIBRARIES(mrpt_icp_slam_2d ${MRPT_LIBS}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS mrpt_icp_slam_2d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  tutorial
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
# Add testing for *.launch files
roslaunch_add_file_check(launch)

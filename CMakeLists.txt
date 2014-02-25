#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola, Karsten Knese
#  Copyright 2010 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

project(task_velocity_damping)

find_package(catkin REQUIRED COMPONENTS tf2)
find_package(PkgConfig REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES task_velocity_damping
	DEPENDS jrl-mathtools jrl-mal dynamic-graph dynamic-graph-python sot-core tf2
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(task_velocity_damping
	src/TaskVelocityDamping.cpp
	include/dynamic-graph/TaskVelocityDamping/TaskVelocityDamping.hh
        include/dynamic-graph/TaskVelocityDamping/SignalHelper.h
)
target_link_libraries(task_velocity_damping ${catkin_LIBRARIES})

set(PKG_DEP dynamic-graph dynamic-graph-python jrl-mathtools jrl-mal sot-core)
foreach(DEP ${PKG_DEP})
	message("invoking dependency check for: ${DEP}")
	pkg_check_modules(${DEP} REQUIRED ${DEP})
	message("use dependency for:  ${${DEP}_LDFLAGS}")
	target_link_libraries(task_velocity_damping ${${DEP}_LDFLAGS})
endforeach(DEP)

# ADD SUBDIRECTORY FOR PYTHON WRAPPER EXPORT
add_subdirectory(src)


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



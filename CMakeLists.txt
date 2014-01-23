#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola, Karsten Knese
#  Copyright 2010 CNRS
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#catkin part
project(task_velocity_damping)
find_package(catkin REQUIRED COMPONENTS)

find_package(PkgConfig REQUIRED)
#pkg_check_modules(dynamic-graph REQUIRED dynamic-graph)
#pkg_check_modules(dynamic-graph REQUIRED dynamic-graph)


 
  # loop over a, b,c with the variable f 
#foreach(f ${VAR}) 
#    message(${f}) 
#endforeach(f) 

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES task_velocity_damping
	DEPENDS jrl-mathtools jrl-mal dynamic-graph dynamic-graph-python sot-core
)

include_directories(include ${catkin_INCLUDE_DIRS})



# LAAS cmake submodule
#INCLUDE(cmake/base.cmake)
#INCLUDE(cmake/python.cmake)
#INCLUDE(cmake/boost.cmake)

#SET(PROJECT_NAME task_velocity_damping)
#SET(PROJECT_VERSION 1.0)
#SET(PROJECT_DESCRIPTION "Task Velocity Damping")
#SET(PROJECT_URL "http://github.com/karsten1987/task_velocity_damping")

#SET(${PROJECT_NAME}_HEADERS
#    include/dynamic-graph/TaskVelocityDamping/TaskVelocityDamping.hh
#)

#SETUP_PROJECT()

# Search for dependencies.
#ADD_REQUIRED_DEPENDENCY("jrl-mathtools")
#ADD_REQUIRED_DEPENDENCY("jrl-mal")
#ADD_REQUIRED_DEPENDENCY("dynamic-graph")
#ADD_REQUIRED_DEPENDENCY("dynamic-graph-python")
#ADD_REQUIRED_DEPENDENCY("sot-core")


add_library(task_velocity_damping
	src/TaskVelocityDamping.cpp
	include/dynamic-graph/TaskVelocityDamping/TaskVelocityDamping.hh
)
target_link_libraries(task_velocity_damping ${catkin_LIBRARIES})
#pkg_config_use_dependency(task_velocity_damping dynamic-graph)

set(PKG_DEP dynamic-graph dynamic-graph-python jrl-mathtools jrl-mal sot-core)
foreach(DEP ${PKG_DEP})
	message("invoking dependency check for: ${DEP}")
	pkg_check_modules(${DEP} REQUIRED ${DEP})
	message("use dependency for:  ${${DEP}_LIBRARIES}")
	target_link_libraries(task_velocity_damping ${${DEP}_LDFLAGS})
	#link_directories(${${DEP}_LIBRARY_DIRS})	
#	pkg_config_use_dependency(task_velocity_damping ${DEP})
endforeach(DEP)


#add_library(task_velocity_damping
#        src/TaskVelocityDamping.cpp
#        include/dynamic-graph/TaskVelocityDamping/TaskVelocityDamping.hh
#)


add_subdirectory(src)

#SETUP_PROJECT_FINALIZE()

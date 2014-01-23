# Copyright 2010, Florent Lamiraux, CNRS
#
# This file is part of sot-core.
# sot-core is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-core is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# dynamic-graph-tutorial. If not, see <http://www.gnu.org/licenses/>.

# Libraries DEFINITIONS and INCLUDES

#dynamic_graph_python_module("sot/${PROJECT_NAME}"
#    ${PROJECT_NAME}
#    ros/${PROJECT_NAME}/wrap
#)

include(../cmake/python.cmake)
FINDPYTHON()
set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")


#CMake Error: File /home/student/sot_ws/src/cmake/dynamic_graph/submodule/__init__.py.cmake does not exist.
#CMake Error at task_velocity_damping/cmake/python.cmake:95 (CONFIGURE_FILE):
#  configure_file Problem configuring file
#Call Stack (most recent call first):
#  task_velocity_damping/CMakeLists.txt:53 (dynamic_graph_python_module)
#DYNAMIC_GRAPH_PYTHON_MODULE("/sot/core/task_velocity_damping" ${LIBRARY_NAME} wrap)
set(WRAP_DIR ${LIBRARY_OUTPUT_PATH}/dynamic_graph/sot/core/${PROJECT_NAME})
message("creating wrapper dir for ${PROJECT_NAME}: ${WRAP_DIR}")
message("python sitedir : ${PYTHON_SITELIB}")
file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/sot/core/${PROJECT_NAME}")
message("TASKVELOCITY !!!!! project name ${PROJECT_NAME}")
message("TASKVELOCITY !!!!!  brary output path ${LIBRARY_OUTPUT_PATH}")

dynamic_graph_python_module("sot/core/${PROJECT_NAME}"
    ${PROJECT_NAME}
    sot/core/${PROJECT_NAME}/wrap
)

#check that!
#SET(NEW_ENTITY_CLASS
#  TaskVelocityDamping
#  )

PYTHON_INSTALL("dynamic_graph/sot/core/${PROJECT_NAME}" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/core" "meta_task_velocity_damping.py")
PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/core" "meta_task_opmod_velocity_damping.py")

#DYNAMIC_GRAPH_PYTHON_MODULE("/sot/core/task_velocity_damping" ${LIBRARY_NAME} wrap)

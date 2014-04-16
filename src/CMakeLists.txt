#
#  Copyright (c) 2013, PAL Robotics, S.L. Author: Gennaro Raiola, Karsten Knese
#  Copyright 2010 CNRS
#

# LAAS cmake include for python 
include(${SOT_CMAKE_DIR}/python.cmake)
FINDPYTHON()
set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib)

set(WRAP_DIR ${LIBRARY_OUTPUT_PATH}/dynamic_graph/sot/core/${PROJECT_NAME})
message("creating wrapper dir for ${PROJECT_NAME}: ${WRAP_DIR}")
message("python sitedir : ${PYTHON_SITELIB}")
file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/sot/core/${PROJECT_NAME}")

dynamic_graph_python_module("sot/core/${PROJECT_NAME}"
    ${PROJECT_NAME}
    sot/core/${PROJECT_NAME}/wrap
)

SET(NEW_ENTITY_CLASS
TaskVelocityDamping
  )


#PYTHON_INSTALL("dynamic_graph/sot/core/${PROJECT_NAME}" "__init__.py" "${PYTHON_SITELIB}")
PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/core" "meta_task_velocity_damping.py")
PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/core" "meta_task_dynamic_velocity_damping.py")
PYTHON_INSTALL_ON_SITE("dynamic_graph/sot/core" "meta_task_opmod_velocity_damping.py")

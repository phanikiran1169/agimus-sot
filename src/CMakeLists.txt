#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of agimus_sot.
# agimus_sot is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# agimus_sot is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# agimus_sot  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  contact-admittance.cc
  gain-adaptive.cc
  holonomic-constraint.cc
  object-localization.cc
  delay.cc
  time.cc
  )

target_link_libraries (${LIBRARY_NAME} PUBLIC
    dynamic-graph-python::dynamic-graph-python
    dynamic_graph_bridge::ros_bridge
    sot-core::sot-core
    )
INSTALL(TARGETS ${LIBRARY_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  )

#AGIMUS_SOT_PYTHON_MODULE("sot" ${LIBRARY_NAME} wrap)
add_library(wrap MODULE python-module-py.cc)
file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src/agimus_sot/sot/${SUBMODULENAME})
set_target_properties(wrap
  PROPERTIES PREFIX ""
  OUTPUT_NAME agimus_sot/sot/wrap)
target_compile_options(wrap PRIVATE "-Wl,--no-as-needed")
target_link_libraries(wrap PUBLIC
  dynamic-graph-python::dynamic-graph-python
  ${PYTHON_LIBRARY} ${LIBRARY_NAME})
target_include_directories(wrap PRIVATE ${PYTHON_INCLUDE_PATH})
install(TARGETS wrap
  DESTINATION ${PYTHON_SITELIB}/agimus_sot/sot)
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/agimus_sot/sot/__init__.py
  DESTINATION ${PYTHON_SITELIB}/agimus_sot/sot)

SET(FILES
  supervisor.py
  tools.py
  solver.py
  events.py
  ros_interface.py
  factory.py
  srdf_parser.py
  __init__.py)

FOREACH(F ${FILES})
    PYTHON_INSTALL_ON_SITE("agimus_sot" ${F})
ENDFOREACH()

SET(FILES
  controllers.py
  switch.py
  gripper.py
  __init__.py)

FOREACH(F ${FILES})
    PYTHON_INSTALL_ON_SITE("agimus_sot/control" ${F})
ENDFOREACH()

SET(FILES
  __init__.py
  com.py
  end_effector.py
  foot.py
  grasp.py
  op_frame.py
  posture.py
  pre_grasp.py
  pre_grasp_post_action.py
  task.py)

FOREACH(F ${FILES})
  PYTHON_INSTALL_ON_SITE("agimus_sot/task" ${F})
ENDFOREACH()

SET(FILES
  __init__.py
  task_factory.py
  pre_grasp.py)

FOREACH(F ${FILES})
  PYTHON_INSTALL_ON_SITE("agimus_sot/react" ${F})
ENDFOREACH()

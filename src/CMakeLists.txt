#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp-core
# hpp-core is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-core is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-core  If not, see
# <http://www.gnu.org/licenses/>.

MACRO(DYNAMIC_GRAPH_HPP_PYTHON_MODULE SUBMODULENAME LIBRARYNAME TARGETNAME)
  FINDPYTHON()

  SET(PYTHON_MODULE ${TARGETNAME})

  ADD_LIBRARY(${PYTHON_MODULE}
    MODULE
    ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/python-module-py.cc)

  FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/src/dynamic_graph_hpp/${SUBMODULENAME})
  SET_TARGET_PROPERTIES(${PYTHON_MODULE}
    PROPERTIES PREFIX ""
    OUTPUT_NAME dynamic_graph_hpp/${SUBMODULENAME}/wrap
   )

  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} "-Wl,--no-as-needed")
  TARGET_LINK_LIBRARIES(${PYTHON_MODULE} ${LIBRARYNAME} ${PYTHON_LIBRARY})

  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

  #
  # Installation
  #
  SET(PYTHON_INSTALL_DIR ${PYTHON_SITELIB}/dynamic_graph_hpp/${SUBMODULENAME})

  INSTALL(TARGETS ${PYTHON_MODULE}
    DESTINATION
    ${PYTHON_INSTALL_DIR})

  SET(ENTITY_CLASS_LIST "")
  FOREACH (ENTITY ${NEW_ENTITY_CLASS})
    SET(ENTITY_CLASS_LIST "${ENTITY_CLASS_LIST}${ENTITY}('')\n")
  ENDFOREACH(ENTITY ${NEW_ENTITY_CLASS})

  CONFIGURE_FILE(
    ${PROJECT_SOURCE_DIR}/cmake/dynamic_graph/submodule/__init__.py.cmake
    ${PROJECT_BINARY_DIR}/src/dynamic_graph_hpp/${SUBMODULENAME}/__init__.py
    )

  INSTALL(
    FILES ${PROJECT_BINARY_DIR}/src/dynamic_graph_hpp/${SUBMODULENAME}/__init__.py
    DESTINATION ${PYTHON_INSTALL_DIR}
    )

ENDMACRO(DYNAMIC_GRAPH_PYTHON_MODULE SUBMODULENAME)

SET(LIBRARY_NAME ${PROJECT_NAME})

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  path-sampler.cc
  event.cc
  boolean-reduction.cc
  ros_subscribe.cpp
  sot_to_ros.cpp
  )

PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} dynamic-graph-python)
PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} dynamic_graph_bridge)
PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} sot-core)
PKG_CONFIG_USE_DEPENDENCY (${LIBRARY_NAME} hpp-corbaserver)
INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  )

SET(NEW_ENTITY_CLASS "PathSampler" "Event" "CompareDouble" "RosQueuedSubscribe")

DYNAMIC_GRAPH_HPP_PYTHON_MODULE("sot" ${LIBRARY_NAME} wrap)

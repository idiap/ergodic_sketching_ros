# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
#
# SPDX-License-Identifier: GPL-3.0-only

add_library(sackmesser)
add_library(sackmesser::sackmesser ALIAS sackmesser)

target_sources(sackmesser
	PRIVATE
	    ConfigurationServer.cpp
	    Logger.cpp
	    utility.cpp
	    Visualiser.cpp
	    Interface.cpp
	    Logger.cpp
)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${${PROJECT_NAME}_VERSION}
  SOVERSION 1
  INTERFACE_${PROJECT_NAME}_MAJOR_VERSION 1
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		$<INSTALL_INTERFACE:${${PROJECT_NAME}_PREFIX_INCLUDE}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>/..
)

configure_file(${CMAKE_CURRENT_LIST_DIR}/sackmesser_package_config.h.in "${CMAKE_CURRENT_BINARY_DIR}/sackmesser/sackmesser_package_config.hpp")

install(
	FILES
		"${CMAKE_CURRENT_BINARY_DIR}/sackmesser/sackmesser_package_config.hpp"
		ConfigurationServer.hpp
		Logger.hpp
	    utility.hpp
	    Visualiser.hpp
	    Interface.hpp
	    CallbackHandler.hpp
	    ClassFactory.hpp
	    Logger.hpp
	DESTINATION
		${${PROJECT_NAME}_PREFIX_INCLUDE}/${PROJECT_NAME}
	COMPONENT
		Devel
)

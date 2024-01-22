# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
#
# SPDX-License-Identifier: GPL-3.0-only

find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_runtime)
add_library(${PROJECT_NAME}::${PROJECT_NAME}_runtime ALIAS ${PROJECT_NAME}_runtime)

target_sources(${PROJECT_NAME}_runtime
	PRIVATE
	    ConfigurationServerYAML.cpp
	    Interface.cpp
)

target_include_directories(${PROJECT_NAME}_runtime
	PUBLIC
		$<INSTALL_INTERFACE:${${PROJECT_NAME}_PREFIX_INCLUDE}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
		$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>/..
		${YAML_CPP_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_runtime
	PUBLIC
		${PROJECT_NAME}
	PRIVATE
		${YAML_CPP_LIBRARIES}
)

install(
	FILES
		ConfigurationServerYAML.hpp
	    Interface.hpp
	    CallbackHandler.hpp
	DESTINATION
		${${PROJECT_NAME}_PREFIX_INCLUDE}/${PROJECT_NAME}_runtime
	COMPONENT
		Devel
)

add_subdirectory(impl)

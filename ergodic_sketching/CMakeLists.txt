# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
# SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
#
# SPDX-License-Identifier: GPL-3.0-only

cmake_minimum_required(VERSION 3.16)
project(ergodic_sketching)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake") # Not needed for this example
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_compile_options(
    "-Wall"
    "-Wextra"
    "-Werror=vla"
    "-Wno-unused-function"
    "-Wno-missing-braces"
    "-Wno-unknown-pragmas"
    "-Wno-parentheses"
    "-pedantic"
    "-Wconversion"
    "-Werror=pedantic"
    "-O3"
)

add_subdirectory(dependencies/sackmesser)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

set(LIB_SRCS
    src/AgentInitialization.cpp
    src/Camera.cpp
    src/Display.cpp
    src/ErgodicAgent.cpp
    src/ErgodicControl.cpp
    src/ErgodicSketching.cpp
    src/HeatEquationCoverage.cpp
    src/ImageProcessing.cpp
    src/RobotDrawing.cpp
    src/Serialization.cpp
    src/SketchPipeline.cpp
    src/SpectralMultiscaleCoverage.cpp
    src/TrajectoryUtils.cpp
)

set(LIB_LIBRARIES
	sackmesser
	${OpenCV_LIBRARIES}
	Eigen3::Eigen
)

set(LIB_HEADERS
    include/ergodic_sketching/AgentConfiguration.hpp
    include/ergodic_sketching/AgentInitialization.hpp
    include/ergodic_sketching/Camera.hpp
    include/ergodic_sketching/CameraConfiguration.hpp
    include/ergodic_sketching/Configuration.hpp
    include/ergodic_sketching/Display.hpp
    include/ergodic_sketching/ErgodicAgent.hpp
    include/ergodic_sketching/ErgodicControl.hpp
    include/ergodic_sketching/ErgodicSketching.hpp
    include/ergodic_sketching/HeatEquationCoverage.hpp
    include/ergodic_sketching/ImageProcessing.hpp
    include/ergodic_sketching/RobotDrawing.hpp
    include/ergodic_sketching/Serialization.hpp
    include/ergodic_sketching/SketchPipeline.hpp
    include/ergodic_sketching/SpectralMultiscaleCoverage.hpp
    include/ergodic_sketching/TrajectoryUtils.hpp
    include/ergodic_sketching/Visualisation.hpp
)

add_library( ${PROJECT_NAME} SHARED ${LIB_SRCS} ${LIB_HEADERS} ) # Create the SHARED library

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # Make headers available to the compilers
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} # Link used libraries
    PUBLIC
      ${LIB_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} # Install library
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers) # Create this shit to be friendly with cmake guidelines
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1
    COMPATIBILITY AnyNewerVersion
)

install( # Install config file (needed to do findPackage(...))
    FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

install(EXPORT ${PROJECT_NAME}Targets # Install Targets file
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
    FILES
        ${LIB_HEADERS}
    DESTINATION
        include/${PROJECT_NAME}
)

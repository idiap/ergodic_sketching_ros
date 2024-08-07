# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
# SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
#
# SPDX-License-Identifier: GPL-3.0-only

include(CMakeFindDependencyMacro)
# Capturing values from configure (optional)
# set(my-config-var @my-config-var@)
# Same syntax as find_package
find_dependency(Eigen3 REQUIRED)
find_dependency(sackmesser REQUIRED)
find_dependency(OpenCV REQUIRED)
# Any extra setup
# Add the targets file
include("${CMAKE_CURRENT_LIST_DIR}/ergodic_sketchingTargets.cmake")

set(ergodic_sketching_LIBRARIES ergodic_sketching::ergodic_sketching)
get_target_property(ergodic_sketching_INCLUDE_DIRS ergodic_sketching::ergodic_sketching INTERFACE_INCLUDE_DIRECTORIES)

# SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
#
# SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
#
# SPDX-License-Identifier: GPL-3.0-only

target_sources(${PROJECT_NAME}_runtime
	PRIVATE
	    ${CMAKE_CURRENT_LIST_DIR}/ConfigurationServerYAML.cpp
)

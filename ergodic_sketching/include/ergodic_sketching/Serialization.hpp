// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Core>
#include <map>

namespace sketching {
namespace serial {
std::map<std::string, std::vector<std::vector<Eigen::Vector2d>>> loadTexts(const std::string& directory);

bool loadText(const std::string& filename, std::pair<std::string, std::vector<std::vector<Eigen::Vector2d>>>& writing);

bool loadTrajectories(const std::string& filename, std::vector<std::vector<Eigen::Vector2d>>& trajectories);
bool loadTrajectories(const std::string& filename, std::vector<Eigen::Matrix<double, 7, 1>>& trajectories);

void save(const std::string& filename, const std::vector<std::vector<Eigen::Vector2d>>& trajectories);

void save(const std::string& filename, const std::vector<Eigen::Vector3d>& path);

void save(const std::string& filename, const std::vector<Eigen::VectorXd>& states);

void save(const std::string& filename, const std::vector<Eigen::Matrix<double, 7, 1>>& states);

void save(const std::string& filename, const std::vector<std::vector<Eigen::Vector3d>>& paths);
}  // namespace serial
}  // namespace sketching

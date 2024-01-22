// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/Serialization.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sackmesser/utility.hpp>

namespace sketching {
namespace serial {
std::map<std::string, std::vector<std::vector<Eigen::Vector2d>>> loadTexts(const std::string& directory) {
    std::filesystem::directory_iterator dir(directory);

    std::map<std::string, std::vector<std::vector<Eigen::Vector2d>>> writing;

    if (!dir->exists()) {
        return writing;
    }

    for (const std::filesystem::directory_entry& entry : dir) {
        std::pair<std::string, std::vector<std::vector<Eigen::Vector2d>>> text;

        if (!loadText(entry.path(), text)) {
            continue;
        }

        std::cout << text.first << std::endl;

        writing.emplace(text);
    }

    return writing;
}

bool loadTrajectories(std::ifstream& data, std::vector<std::vector<Eigen::Vector2d>>& trajectories) {
    std::vector<Eigen::Vector2d> trajectory;

    std::string line;

    while (!data.eof()) {
        std::getline(data, line);

        if (line.find(";") < line.length()) {
            if (!trajectory.empty()) {
                trajectories.push_back(trajectory);
                trajectory.clear();
            }

            continue;
        }

        std::vector<std::string> values = sackmesser::split_string(line, ' ');

        if (values.size() != 2) {
            continue;
        }

        trajectory.push_back(Eigen::Vector2d(std::stod(values[0]), std::stod(values[1])));
    }

    return true;
}

bool loadTrajectories(std::ifstream& data, std::vector<Eigen::Matrix<double, 7, 1>>& trajectory) {
    std::string line;

    while (!data.eof()) {
        std::getline(data, line);

        if (line.find(";") < line.length()) {
            break;
        }

        std::vector<std::string> values = sackmesser::split_string(line, ',');

        if (values.size() != 7) {
            continue;
        }
        Eigen::Matrix<double, 7, 1> states_t;
        states_t << std::stod(values[0]), std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);
        trajectory.push_back(states_t);
    }

    return true;
}

bool loadText(const std::string& filename, std::pair<std::string, std::vector<std::vector<Eigen::Vector2d>>>& writing) {
    std::ifstream data(filename);

    if (!data.is_open()) {
        return false;
    }

    std::string text = "";
    std::string line = "";

    bool read = true;

    while (read) {
        std::getline(data, line);

        if (line.find(";") < line.length()) {
            read = false;
            continue;
        }

        text.append(line);
    }

    std::vector<std::vector<Eigen::Vector2d>> trajectories;

    loadTrajectories(data, trajectories);

    data.close();

    writing = std::make_pair(text, trajectories);

    return true;
}

bool loadTrajectories(const std::string& filename, std::vector<std::vector<Eigen::Vector2d>>& trajectories) {
    std::ifstream data(filename);

    if (!data.is_open()) {
        return false;
    }

    loadTrajectories(data, trajectories);

    data.close();

    return true;
}

bool loadTrajectories(const std::string& filename, std::vector<Eigen::Matrix<double, 7, 1>>& trajectories) {
    std::ifstream data(filename);

    if (!data.is_open()) {
        return false;
    }

    loadTrajectories(data, trajectories);

    data.close();

    return true;
}

void save(const std::string& filename, const std::vector<std::vector<Eigen::Vector2d>>& trajectories) {
    std::ofstream data(filename);

    for (const std::vector<Eigen::Vector2d>& path : trajectories) {
        for (const Eigen::Vector2d& position : path) {
            data << position.x() << " " << position.y() << std::endl;
        }

        data << ";" << std::endl;
    }

    data.close();
}

void save(const std::string& filename, const std::vector<Eigen::Vector3d>& path) {
    std::ofstream csv_export;
    csv_export.open(filename);
    for (auto p : path) {
        csv_export << p.x() << " " << p.y() << " " << p.z() << "\n";
    }
    csv_export.close();
}

void save(const std::string& filename, const std::vector<Eigen::VectorXd>& states) {
    Eigen::IOFormat saveFormat(4, 0, "", ",", "", "", "", "");
    std::ofstream f;
    f.open(filename);

    for (auto st : states) {
        f << st.format(saveFormat) << "\n";
    }
    f.close();
}

void save(const std::string& filename, const std::vector<Eigen::Matrix<double, 7, 1>>& states) {
    Eigen::IOFormat saveFormat(4, 0, "", ",", "", "", "", "");
    std::ofstream f;
    f.open(filename);

    for (auto st : states) {
        f << st.format(saveFormat) << "\n";
    }
    f.close();
}

void save(const std::string& filename, const std::vector<std::vector<Eigen::Vector3d>>& paths) {
    std::ofstream csv_export;
    csv_export.open(filename);
    for (auto path : paths) {
        for (auto p : path) {
            csv_export << p.x() << " " << p.y() << " " << p.z() << "\n";
        }
        csv_export << "\n";
    }

    csv_export.close();
}

}  // namespace serial
}  // namespace sketching

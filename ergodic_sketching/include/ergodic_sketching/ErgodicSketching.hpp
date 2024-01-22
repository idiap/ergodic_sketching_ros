// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Core>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

namespace cv {
class Mat;
}

namespace sackmesser {
class Interface;
}  // namespace sackmesser

namespace sketching {
class Planner;
class ErgodicControl;
class ImageProcessing;
class FaceLandmarks;
class SketchPipeline;

class ErgodicSketching {
public:
    explicit ErgodicSketching(const std::shared_ptr<sackmesser::Interface>& interface);

    virtual ~ErgodicSketching();

    std::vector<std::vector<Eigen::Vector2d>> sketch(const cv::Mat& image) const;

    std::vector<std::vector<Eigen::Vector2d>> sketch(const std::string& pipeline, const cv::Mat& image) const;

    std::vector<std::vector<Eigen::Vector2d>> sketch(const std::vector<std::pair<std::string, cv::Mat>>& input) const;

    class Pipeline;

    const Pipeline* pipeline(const std::string& name) const;

protected:
private:
    void saveSketch(const std::vector<std::vector<Eigen::Vector2d>>& paths) const;

private:
    std::shared_ptr<sackmesser::Interface> interface_;

    std::map<std::string, std::unique_ptr<Pipeline>> pipelines_;

    class Configuration;

    std::unique_ptr<Configuration> config_;
};

}  // namespace sketching

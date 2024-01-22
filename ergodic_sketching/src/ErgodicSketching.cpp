// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/ErgodicControl.hpp>
#include <ergodic_sketching/ErgodicSketching.hpp>
#include <ergodic_sketching/HeatEquationCoverage.hpp>
#include <ergodic_sketching/ImageProcessing.hpp>
#include <ergodic_sketching/Serialization.hpp>
#include <ergodic_sketching/SketchPipeline.hpp>
#include <fstream>
#include <sackmesser/CallbackHandler.hpp>
#include <sackmesser/Interface.hpp>
#include <sackmesser/Visualiser.hpp>

namespace sketching {
class ErgodicSketching::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) {
        return config_server->loadParameter(ns_ + "render_image", &render_image, true) &&            //
               config_server->loadParameter(ns_ + "save_trajectories", &save_trajectories, true) &&  //
               config_server->loadParameter(ns_ + "pipelines", &pipelines) &&                        //
               config_server->loadParameter(ns_ + "show_points", &show_points, true);
    }

    std::vector<std::string> pipelines;
    bool save_trajectories = false;
    bool render_image = false;
    bool show_points = false;
};

ErgodicSketching::ErgodicSketching(const std::shared_ptr<sackmesser::Interface>& interface) : interface_(interface), config_(std::make_unique<Configuration>("ergodic_sketching/")) {
    config_->load(interface->configServer());

    for (const std::string& pipeline : config_->pipelines) {
        pipelines_.emplace(std::make_pair(pipeline, std::make_unique<Pipeline>(interface, "ergodic_sketching/" + pipeline + "/")));
    }

    interface->callbacks()->add<std::vector<std::vector<Eigen::Vector2d>>(const cv::Mat&, const int&)>("patch", [this](const cv::Mat& image, const int& /*timesteps*/) { return sketch(image); });
}

std::vector<std::vector<Eigen::Vector2d>> ErgodicSketching::sketch(const cv::Mat& image) const {
    std::vector<ErgodicControl::Agent::Path> all_paths;

    for (const std::pair<const std::string, std::unique_ptr<Pipeline>>& pipeline : pipelines_) {
        std::vector<ErgodicControl::Agent::Path> paths = pipeline.second->compute(image);

        all_paths.insert(all_paths.end(), std::make_move_iterator(paths.begin()), std::make_move_iterator(paths.end()));
    }

    saveSketch(all_paths);

    return all_paths;
}

std::vector<std::vector<Eigen::Vector2d>> ErgodicSketching::sketch(const std::string& pipeline, const cv::Mat& image) const {
    if (pipelines_.find(pipeline) == pipelines_.end()) {
        interface_->log()->msg("no pipeline named " + pipeline, sackmesser::LOG::ERROR);

        return std::vector<ErgodicControl::Agent::Path>();
    }

    std::vector<ErgodicControl::Agent::Path> paths = pipelines_.at(pipeline)->compute(image);

    interface_->log()->msg("pipeline " + pipeline + " finished sketching");

    saveSketch(paths);

    return paths;
}
std::vector<std::vector<Eigen::Vector2d>> ErgodicSketching::sketch(const std::vector<std::pair<std::string, cv::Mat>>& input) const {
    std::vector<ErgodicControl::Agent::Path> all_paths;

    for (const std::pair<std::string, cv::Mat>& it : input) {
        std::vector<ErgodicControl::Agent::Path> paths = sketch(it.first, it.second);

        all_paths.insert(all_paths.end(), std::make_move_iterator(paths.begin()), std::make_move_iterator(paths.end()));
    }

    saveSketch(all_paths);

    return all_paths;
}

ErgodicSketching::~ErgodicSketching() {}

void ErgodicSketching::saveSketch(const std::vector<std::vector<Eigen::Vector2d>>& paths) const {
    if (!config_->save_trajectories) {
        return;
    }

    serial::save(interface_->outputPath() + interface_->timeString() + "_sketch.csv", paths);

    interface_->log()->msg("saved " + interface_->outputPath() + interface_->timeString() + "_sketch.csv");
}

const ErgodicSketching::Pipeline* ErgodicSketching::pipeline(const std::string& name) const {
    if (pipelines_.find(name) == pipelines_.end()) {
        return nullptr;
    }

    return pipelines_.at(name).get();
}

}  // namespace sketching

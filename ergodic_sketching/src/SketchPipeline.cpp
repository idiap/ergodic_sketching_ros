// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/Configuration.hpp>
#include <ergodic_sketching/HeatEquationCoverage.hpp>
#include <ergodic_sketching/ImageProcessing.hpp>
#include <ergodic_sketching/SketchPipeline.hpp>
#include <sackmesser/Interface.hpp>
#include <sackmesser/Visualiser.hpp>

namespace sketching {
ErgodicSketching::Pipeline::Pipeline(const std::shared_ptr<sackmesser::Interface>& interface, const std::string& name) : interface_(interface), config_(std::make_unique<Configuration>(name)) {
    ergodic_control_ = std::make_unique<HeatEquationCoverage>(interface, name);
    image_processing_ = std::make_unique<ImageProcessing>(interface->configServer(), name);

    config_->load(interface->configServer());
}

ErgodicSketching::Pipeline::~Pipeline() {
    if (ergodic_control_->isComputing()) {
        ergodic_control_->stop();
    }
}

std::vector<ErgodicControl::Agent::Path> ErgodicSketching::Pipeline::compute(const cv::Mat& image) {
    cv::Mat input = image;

    image_processing_->process(input);

    interface_->visualiser()->show(input, "processed");

    std::vector<ErgodicControl::Agent::Path> all_paths;

    Eigen::MatrixXd distribution;
    cv::cv2eigen(input, distribution);

    for (int i = 0; i < config_->num_strokes; ++i) {
        if (distribution.sum() < 1000.0) {
            interface_->log()->msg("CONVERGED");

            return all_paths;
        }

        interface_->log()->msg("STROKE " + std::to_string(i + 1) + "/" + std::to_string(config_->num_strokes));

        std::vector<ErgodicControl::Agent::Path> paths = ergodic_control_->compute(distribution);

        for (ErgodicControl::Agent::Path& path : paths) {
            all_paths.push_back(path);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    return all_paths;
}

const ImageProcessing* ErgodicSketching::Pipeline::imageProcessing() const {
    return image_processing_.get();
}

}  // namespace sketching

// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <ergodic_sketching/ErgodicControl.hpp>
#include <ergodic_sketching/ErgodicSketching.hpp>
#include <memory>
#include <string>
#include <vector>

namespace sackmesser {
class Interface;
}

namespace cv {
class Mat;
}

namespace sketching {
class ImageProcessing;

class ErgodicSketching::Pipeline {
public:
    explicit Pipeline(const std::shared_ptr<sackmesser::Interface>& interface, const std::string& name = "ergodic_control/");

    virtual ~Pipeline();

    std::vector<ErgodicControl::Agent::Path> compute(const cv::Mat& image);

    const ImageProcessing* imageProcessing() const;

protected:
private:
    std::shared_ptr<sackmesser::Interface> interface_;

    std::unique_ptr<ImageProcessing> image_processing_;

    std::unique_ptr<ErgodicControl> ergodic_control_;

    class Configuration;

    std::unique_ptr<Configuration> config_;
};

}  // namespace sketching

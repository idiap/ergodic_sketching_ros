// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Core>
#include <atomic>
#include <list>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>
namespace sackmesser {
class Interface;
}

namespace sketching {
namespace acquisition {
class Camera {
public:
    explicit Camera(const std::shared_ptr<sackmesser::Interface>& interface);

    virtual ~Camera();

    virtual cv::Mat getFrame() = 0;

    bool isFaceDetected();

    void drawFaceOval(cv::Mat& img);

    Eigen::Vector2d imageSize();

    void applySignatureBox(cv::Mat& image);

    virtual void start() = 0;

    virtual void stop() = 0;

protected:
    void updateFrame(const cv::Mat& image);

    bool isRunning();

protected:
    cv::Mat last_frame_;

    std::atomic<bool> running_;
    std::atomic<bool> face_detected_;

    class Configuration;

    std::unique_ptr<Configuration> config_;

public:
    const Configuration* config() const;
};

class UsbCamera : public Camera {
public:
    explicit UsbCamera(const std::shared_ptr<sackmesser::Interface>& interface);

    void start() override;

    void stop() override;

    cv::Mat getFrame() override;

    ~UsbCamera();

private:
    cv::VideoCapture camera_;
};

}  // namespace acquisition
}  // namespace sketching

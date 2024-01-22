// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/Camera.hpp>

#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Interface.hpp>

#include <ergodic_sketching/CameraConfiguration.hpp>

namespace sketching {
namespace acquisition {
Camera::Camera(const std::shared_ptr<sackmesser::Interface>& interface) : config_(std::make_unique<Configuration>("camera/")) {
    config_->load(interface->configServer());
    face_detected_ = false;
}

Camera::~Camera() {}

bool Camera::isRunning() {
    return running_;
}

bool Camera::isFaceDetected() {
    return face_detected_.load();
}

void Camera::drawFaceOval(cv::Mat& img) {
    int face_oval_height = (int)(0.9 * config_->image_height / 2);
    int face_oval_width = (int)(face_oval_height / 1.62);

    cv::ellipse(img, cv::Point(config_->image_width / 2, config_->image_height / 2), cv::Size(face_oval_width, face_oval_height), 0, 0, 360, cv::Scalar(208, 224, 64), 2, cv::LINE_AA);
}

Eigen::Vector2d Camera::imageSize() {
    return Eigen::Vector2d(config_->image_height, config_->image_width);
}

void Camera::applySignatureBox(cv::Mat& image) {
    cv::rectangle(image, cv::Point(0, config_->image_height), cv::Point(config_->rectangle_width, config_->image_height - config_->rectangle_height), cv::Scalar(255, 255, 255), cv::FILLED,
                  cv::LINE_8);
}

const Camera::Configuration* Camera::config() const {
    return config_.get();
}

UsbCamera::UsbCamera(const std::shared_ptr<sackmesser::Interface>& interface) : Camera(interface) {}

void UsbCamera::start() {
    camera_.open("/dev/video0");
    if (!camera_.isOpened()) {
        throw std::runtime_error("Could not open USB camera");
    }
    camera_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera_.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
    camera_.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    camera_.set(cv::CAP_PROP_FPS, 30);
    running_ = true;
}

void UsbCamera::stop() {
    camera_.release();
    running_ = false;
}

cv::Mat UsbCamera::getFrame() {
    cv::Mat raw_image;
    camera_ >> raw_image;

    cv::Rect crop_area(raw_image.size().width / 2 - config_->image_width / 2, raw_image.size().height / 2 - config_->image_height / 2, config_->image_width, config_->image_height);
    last_frame_ = raw_image(crop_area);

    if (config_->cv_rotate > -1) {
        cv::rotate(last_frame_, last_frame_, config_->cv_rotate);
    }

    return last_frame_;
}

UsbCamera::~UsbCamera() {}

}  // namespace acquisition
}  // namespace sketching

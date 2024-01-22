// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/Display.hpp>

namespace sketching {
Display::Display(const std::shared_ptr<sackmesser::Interface>& interface) : config_(std::make_unique<Configuration>("display/")) {
    config_->load(interface->configServer());

    raw_display_ = cv::Mat::ones(config_->display_height, config_->display_width, CV_8UC3);
    raw_display_.setTo(cv::Scalar(0, 0, 0));

    font_ = cv::freetype::createFreeType2();
    font_->loadFontData(interface->inputPath() + "fonts/" + config_->font_file, 0);
}

void Display::changeBackgroundColor(const cv::Scalar& color) {
    raw_display_.setTo(color);
}

void Display::resetBackgroundColor() {
    raw_display_.setTo(cv::Scalar(0, 0, 0));
}

cv::Mat Display::displayWithText(const std::string& text) {
    cv::Mat display = raw_display_.clone();  // Desired copy

    int x_cord = config_->display_width / 2;
    int y_cord = config_->display_height / 2;

    cv::Size text_size = font_->getTextSize(text, 35, -1, 0);

    x_cord -= text_size.width / 2;
    y_cord -= text_size.height / 2;  // Approximation of string height

    cv::Point text_origin(x_cord, y_cord);
    font_->putText(display, text, text_origin, 35, cv::Scalar(255, 255, 255), -1, cv::LINE_AA, true);
    return display;
}

cv::Mat Display::displayImageWithLabel(const std::string& text, const cv::Mat& image) {
    cv::Mat display = raw_display_.clone();  // Desired copy

    int x_cord_img = config_->area_width / 2 + config_->areas.at(0).x;
    int y_cord_img = config_->area_height / 2 + config_->areas.at(0).y;

    x_cord_img -= image.size().width / 2;
    y_cord_img -= image.size().height / 2;

    image.copyTo(display(cv::Rect(x_cord_img, y_cord_img, image.cols, image.rows)));

    int x_cord_txt = config_->area_width / 2 + config_->areas.at(1).x;
    int y_cord_txt = config_->area_height / 2 + config_->areas.at(1).y;
    cv::Size text_size = font_->getTextSize(text, 35, -1, 0);

    x_cord_txt -= text_size.width / 2;
    y_cord_txt -= text_size.height / 2;

    cv::Point text_origin(x_cord_txt, y_cord_txt);
    font_->putText(display, text, text_origin, 35, cv::Scalar(255, 255, 255), -1, cv::LINE_AA, true);

    return display;
}

cv::Mat Display::displayWithMultilinesText(const std::vector<std::string>& lines) {
    cv::Mat display = raw_display_.clone();  // Desired copy

    cv::Size text_size(0, 0);
    std::vector<cv::Size> line_sizes;
    for (auto line : lines) {
        cv::Size size_i = font_->getTextSize(line, 35, -1, 0);
        size_i.height += 20;

        if (size_i.width > text_size.width)
            text_size.width = size_i.width;

        text_size.height += size_i.height;

        line_sizes.push_back(size_i);
    }

    int y_cord = config_->display_height / 2;
    y_cord -= text_size.height / 2;

    for (int i = 0; i < lines.size(); i++) {
        int x_cord = (config_->display_width - line_sizes.at(i).width) / 2;

        cv::Point text_origin(x_cord, y_cord);
        font_->putText(display, lines.at(i), text_origin, 35, cv::Scalar(255, 255, 255), -1, cv::LINE_AA, true);

        y_cord += line_sizes.at(i).height;
    }

    return display;
}

cv::Mat Display::displayImageWithMultilinesText(const std::vector<std::string>& lines, const cv::Mat& image) {
    cv::Mat display = raw_display_.clone();  // Desired copy

    cv::Size text_size(0, 0);
    std::vector<cv::Size> line_sizes;
    for (auto line : lines) {
        cv::Size size_i = font_->getTextSize(line, 35, -1, 0);
        size_i.height += 20;

        if (size_i.width > text_size.width)
            text_size.width = size_i.width;

        text_size.height += size_i.height;

        line_sizes.push_back(size_i);
    }

    int x_cord_img = config_->area_width / 2 + config_->areas.at(0).x;
    int y_cord_img = config_->area_height / 2 + config_->areas.at(0).y;

    x_cord_img -= image.size().width / 2;
    y_cord_img -= image.size().height / 2;
    image.copyTo(display(cv::Rect(x_cord_img, y_cord_img, image.cols, image.rows)));

    int x_cord_txt = config_->area_width / 2 + config_->areas.at(1).x;
    int y_cord_txt = config_->area_height / 2 + config_->areas.at(1).y;

    y_cord_txt -= text_size.height / 2;

    for (int i = 0; i < lines.size(); i++) {
        int x_cord_txt_tmp = x_cord_txt - line_sizes.at(i).width / 2;

        cv::Point text_origin(x_cord_txt_tmp, y_cord_txt);
        font_->putText(display, lines.at(i), text_origin, 35, cv::Scalar(255, 255, 255), -1, cv::LINE_AA, true);

        y_cord_txt += line_sizes.at(i).height;
    }

    return display;
}

const Display::Configuration* Display::config() const {
    return config_.get();
}
}  // namespace sketching

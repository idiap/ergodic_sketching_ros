// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <memory>
#include <opencv2/freetype.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Interface.hpp>

namespace sketching {
class Display {
    struct Configuration : public sackmesser::ConfigurationBase {
    public:
        using sackmesser::ConfigurationBase::ConfigurationBase;
        bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) {
            bool loaded = config_server->loadParameter(ns_ + "width", &display_width, false) &&    //
                          config_server->loadParameter(ns_ + "height", &display_height, false) &&  //
                          config_server->loadParameter(ns_ + "font_file", &font_file);

            area_width = display_width / 2;
            area_height = display_height;

            cv::Point area1_origin(0, 0);
            cv::Point area2_origin(area_width - 10, 0);

            areas.push_back(area1_origin);
            areas.push_back(area2_origin);

            return loaded;
        }

        std::vector<cv::Point> areas;
        int display_width;
        std::string font_file;
        int display_height;
        int area_width;
        int area_height;
    };

public:
    explicit Display(const std::shared_ptr<sackmesser::Interface>& interface);
    virtual ~Display() {}

    void changeBackgroundColor(const cv::Scalar& color);
    void resetBackgroundColor();
    cv::Mat displayWithText(const std::string& text);
    cv::Mat displayWithMultilinesText(const std::vector<std::string>& lines);
    cv::Mat displayImageWithLabel(const std::string& text, const cv::Mat& image);
    cv::Mat displayImageWithMultilinesText(const std::vector<std::string>& lines, const cv::Mat& image);

    const Configuration* config() const;

private:
    cv::Mat raw_display_;
    cv::Ptr<cv::freetype::FreeType2> font_;
    std::unique_ptr<Configuration> config_;
};
}  // namespace sketching

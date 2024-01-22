// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <Eigen/Dense>
#include <ergodic_sketching/ImageProcessing.hpp>
#include <sackmesser/ConfigurationServer.hpp>

namespace sketching {
sackmesser::Factory<ImageProcessing::Processor> ImageProcessing::Processor::factory_ = sackmesser::Factory<ImageProcessing::Processor>();

ImageProcessing::ImageProcessing(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) {
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "preprocessor",
        [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr { return std::make_unique<Preprocessor>(config_server, name); });
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "geometric_warping",
        [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr { return std::make_unique<GeometricWarping>(config_server, name); });
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "edge_enhancement",
        [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr { return std::make_unique<EdgeEnhancement>(config_server, name); });
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "pencil_sketch",
        [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr { return std::make_unique<PencilSketch>(config_server, name); });
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "histogram_normalization",
        [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr { return std::make_unique<HistogramNormalization>(config_server, name); });
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "contrast_brightness_enhancement", [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr {
            return std::make_unique<ContrastBrightnessEnhancement>(config_server, name);
        });
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "binarization",
        [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr { return std::make_unique<Binarization>(config_server, name); });
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "quantization",
        [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr { return std::make_unique<Quantization>(config_server, name); });
    Processor::Factory().add<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(
        "contours", [](const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) -> Processor::Ptr { return std::make_unique<Contours>(config_server, name); });

    std::vector<std::string> processors;
    config_server->loadParameter(name + "image_processing/processors", &processors);

    for (const std::string& processor : processors) {
        processors_.push_back(Processor::Factory().create<const std::shared_ptr<sackmesser::ConfigurationServer>&, const std::string&>(processor, config_server, name + "image_processing/"));
    }
}

ImageProcessing::~ImageProcessing() {}

void ImageProcessing::process(cv::Mat& image) const {
    for (unsigned i = 0; i < processors_.size(); ++i) {
        processors_[i]->apply(image);
    }
}

ImageProcessing::Processor::Processor() {}

ImageProcessing::Processor::~Processor() {}

sackmesser::Factory<ImageProcessing::Processor>& ImageProcessing::Processor::Factory() {
    return factory_;
}

class ImageProcessing::Preprocessor::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) {
        return config_server->loadParameter(ns_ + "invert_image", &invert_image, false) &&  //
               config_server->loadParameter(ns_ + "threshold", &threshold, true, 0, 255);
    }

public:
    bool invert_image = true;
    int threshold = 0;
};

ImageProcessing::Preprocessor::Preprocessor(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name)
    : config_(std::make_unique<Configuration>(name + "preprocessor/")) {
    config_->load(config_server);
}

ImageProcessing::Preprocessor::~Preprocessor() {}

void ImageProcessing::Preprocessor::apply(cv::Mat& image) {
    cv::cvtColor(image, image, cv::COLOR_BGRA2GRAY);

    if (config_->invert_image) {
        cv::bitwise_not(image, image);
    }

    cv::threshold(image, image, config_->threshold, 255, cv::THRESH_TOZERO);

    cv::rotate(image, image, cv::ROTATE_180);

    if (image.cols > image.rows) {
        cv::rotate(image, image, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
}

ImageProcessing::GeometricWarping::Mask::Mask() {}

ImageProcessing::GeometricWarping::Mask::~Mask() {}

void ImageProcessing::GeometricWarping::Mask::apply(cv::Mat& image) {
    cv::Rect crop_lower(0, image.rows / 2 - 1, image.cols, image.rows / 2);
    cv::Rect crop_upper(0, 0, image.cols, image.rows / 2);

    cv::Mat image_lower = image(crop_lower);
    cv::Mat image_upper = image(crop_upper);

    double h = static_cast<double>(image.rows / 2) + 1.0;
    double w = static_cast<double>(image.cols);

    std::vector<cv::Point2d> source_nodes(4);
    source_nodes[0] = cv::Point2d(0.0, 0.0);
    source_nodes[1] = cv::Point2d(w, 0.0);
    source_nodes[2] = cv::Point2d(w, h);
    source_nodes[3] = cv::Point2d(0.0, h);

    cv::Mat warp_mat_lower = cv::findHomography(source_nodes, getNodesLower(w, h), cv::RANSAC);
    cv::Mat warp_mat_upper = cv::findHomography(source_nodes, getNodesUpper(w, h), cv::RANSAC);

    cv::warpPerspective(image_lower, image_lower, warp_mat_lower, image_lower.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, 0);
    cv::warpPerspective(image_upper, image_upper, warp_mat_upper, image_upper.size(), cv::INTER_CUBIC, cv::BORDER_CONSTANT, 0);

    image(crop_lower) = image_lower;
    image(crop_upper) = image_upper;
}

class ImageProcessing::GeometricWarping::Hourglass::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) {
        return config_server->loadParameter(ns_ + "left_scale", &left_scale, true) &&    //
               config_server->loadParameter(ns_ + "right_scale", &right_scale, true) &&  //
               config_server->loadParameter(ns_ + "stretch", &stretch, true, 0.5, 2.0);
    }

public:
    double left_scale = 0.15;
    double right_scale = 0.45;
    double stretch = 1.5;
};

class ImageProcessing::GeometricWarping::Diamond::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) { return config_server->loadParameter(ns_ + "indent", &indent, true); }

public:
    double indent;
};

ImageProcessing::GeometricWarping::Hourglass::Hourglass(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name)
    : config_(std::make_unique<Configuration>(name + "geometric_warping/hourglass/")) {
    config_->load(config_server);
}

ImageProcessing::GeometricWarping::Hourglass::~Hourglass() {}

std::vector<cv::Point2d> ImageProcessing::GeometricWarping::Hourglass::getNodesLower(const double& w, const double& h) {
    std::vector<cv::Point2d> target_nodes_lower(4);
    target_nodes_lower[0] = cv::Point2d(config_->left_scale * w, 0.0);
    target_nodes_lower[1] = cv::Point2d(config_->right_scale * w, 0.0);
    target_nodes_lower[2] = cv::Point2d(w, config_->stretch * h);
    target_nodes_lower[3] = cv::Point2d(0.0, config_->stretch * h);

    return target_nodes_lower;
}

std::vector<cv::Point2d> ImageProcessing::GeometricWarping::Hourglass::getNodesUpper(const double& w, const double& h) {
    std::vector<cv::Point2d> target_nodes_upper(4);
    target_nodes_upper[0] = cv::Point2d(0.f, -(config_->stretch - 1.0) * h);
    target_nodes_upper[1] = cv::Point2d(w, -(config_->stretch - 1.0) * h);
    target_nodes_upper[2] = cv::Point2d(config_->right_scale * w, h);
    target_nodes_upper[3] = cv::Point2d(config_->left_scale * w, h);

    return target_nodes_upper;
}

ImageProcessing::GeometricWarping::Diamond::Diamond(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name)
    : config_(std::make_unique<Configuration>(name + "geometric_warping/diamond/")) {
    config_->load(config_server);
}

ImageProcessing::GeometricWarping::Diamond::~Diamond() {}

std::vector<cv::Point2d> ImageProcessing::GeometricWarping::Diamond::getNodesLower(const double& w, const double& h) {
    std::vector<cv::Point2d> target_nodes_lower(4);
    target_nodes_lower[0] = cv::Point2d(0.0, -5.0);
    target_nodes_lower[1] = cv::Point2d(w, -5.0);
    target_nodes_lower[2] = cv::Point2d((1.0 - config_->indent) * w, h);
    target_nodes_lower[3] = cv::Point2d(config_->indent * w, h);

    return target_nodes_lower;
}

std::vector<cv::Point2d> ImageProcessing::GeometricWarping::Diamond::getNodesUpper(const double& w, const double& h) {
    std::vector<cv::Point2d> target_nodes_upper(4);
    target_nodes_upper[0] = cv::Point2d(config_->indent * w, 0.0);
    target_nodes_upper[1] = cv::Point2d((1.0 - config_->indent) * w, 0.0);
    target_nodes_upper[2] = cv::Point2d(w, h + 5.0);
    target_nodes_upper[3] = cv::Point2d(0.0, h + 5.0);

    return target_nodes_upper;
}

ImageProcessing::GeometricWarping::GeometricWarping(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) {
    // masks_.push_back(std::make_shared<Hourglass>(config_server, name));
    masks_.push_back(std::make_shared<Diamond>(config_server, name));
}

ImageProcessing::GeometricWarping::~GeometricWarping() {}

void ImageProcessing::GeometricWarping::apply(cv::Mat& image) {
    unsigned idx = static_cast<unsigned>(std::rand() % static_cast<int>(masks_.size()));

    masks_[idx]->apply(image);
}

class ImageProcessing::EdgeEnhancement::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) {
        return config_server->loadParameter(ns_ + "canny_low", &canny_low, true, 0.0) &&    //
               config_server->loadParameter(ns_ + "canny_high", &canny_high, true, 0.0) &&  //
               config_server->loadParameter(ns_ + "threshold", &threshold, true, 0.0) &&    //
               config_server->loadParameter(ns_ + "maxval", &maxval, true, 0.0);
    }

public:
    double canny_low = 50;
    double canny_high = 200;
    double threshold = 200;
    double maxval = 255;
};

ImageProcessing::EdgeEnhancement::EdgeEnhancement(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name)
    : config_(std::make_unique<Configuration>(name + "edge_enhancement/")) {
    config_->load(config_server);
}

ImageProcessing::EdgeEnhancement::~EdgeEnhancement() {}

void ImageProcessing::EdgeEnhancement::apply(cv::Mat& image) {
    cv::GaussianBlur(image, image, cv::Size(3, 3), 0.0, 0.0);

    cv::Mat canny;
    cv::Canny(image, canny, config_->canny_low, config_->canny_high);

    cv::bitwise_not(image, image);
    cv::threshold(image, image, config_->threshold, config_->maxval, cv::THRESH_TOZERO);

    image += canny;
}

class ImageProcessing::Quantization::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) { return config_server->loadParameter(ns_ + "num_bins", &num_bins, true, 0); }

public:
    int num_bins = 4;
};

ImageProcessing::Quantization::Quantization(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name)
    : config_(std::make_unique<Configuration>(name + "quantization/")) {
    config_->load(config_server);
}

ImageProcessing::Quantization::~Quantization() {}

void ImageProcessing::Quantization::apply(cv::Mat& /*image*/) {}

class ImageProcessing::PencilSketch::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& /*config_server*/) {
        return true;
        // return config_server->loadParameter(ns_ + "num_bins", &num_bins, true, 0);
    }

public:
};

ImageProcessing::PencilSketch::PencilSketch(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name)
    : config_(std::make_unique<Configuration>(name + "pencil_sketch/")) {
    config_->load(config_server);
}

ImageProcessing::PencilSketch::~PencilSketch() {}

void ImageProcessing::PencilSketch::apply(cv::Mat& image) {
    cv::Mat img_invert, img_smoothing, img_dodged;
    cv::bitwise_not(image, img_invert);
    cv::GaussianBlur(img_invert, img_smoothing, cv::Size(21, 21), 0, 0);
    cv::divide(image, 255 - img_smoothing, img_dodged, 256);
    image = ~img_dodged;
}

ImageProcessing::HistogramNormalization::HistogramNormalization(const std::shared_ptr<sackmesser::ConfigurationServer>& /*config_server*/, const std::string& /*name*/) {}

ImageProcessing::HistogramNormalization::~HistogramNormalization() {}

void ImageProcessing::HistogramNormalization::apply(cv::Mat& image) {
    // CLAHE histogram normalization
    cv::Mat lab_image;
    cv::cvtColor(image, lab_image, cv::COLOR_BGR2Lab);
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);
    cv::Size s(8, 8);
    clahe->setTilesGridSize(s);

    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);
    cv::cvtColor(lab_image, image, cv::COLOR_Lab2BGR);
}

class ImageProcessing::ContrastBrightnessEnhancement::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) {
        return config_server->loadParameter(ns_ + "alpha", &alpha, true, 0) &&  //
               config_server->loadParameter(ns_ + "beta", &beta, true, 0);
    }

    double alpha = 1.5;
    double beta = 50;

public:
};

ImageProcessing::ContrastBrightnessEnhancement::ContrastBrightnessEnhancement(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name)
    : config_(std::make_unique<Configuration>(name + "contrast_brightness_enhancement/")) {
    config_->load(config_server);
}

ImageProcessing::ContrastBrightnessEnhancement::~ContrastBrightnessEnhancement() {}

void ImageProcessing::ContrastBrightnessEnhancement::apply(cv::Mat& image) {
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            for (int c = 0; c < image.channels(); c++) {
                image.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(config_->alpha * image.at<cv::Vec3b>(y, x)[c] + config_->beta);
            }
        }
    }
}

class ImageProcessing::Binarization::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) { return config_server->loadParameter(ns_ + "threshold", &threshold, true, 0); }

    int threshold = 0;

public:
};

ImageProcessing::Binarization::Binarization(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name)
    : config_(std::make_unique<Configuration>(name + "binarization/")) {
    config_->load(config_server);
}

ImageProcessing::Binarization::~Binarization() {}

void ImageProcessing::Binarization::apply(cv::Mat& image) {
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            if (image.at<uchar>(y, x) > config_->threshold) {
                image.at<uchar>(y, x) = 255;
            } else {
                image.at<uchar>(y, x) = 0;
            }
        }
    }
}

class ImageProcessing::Contours::Configuration : public sackmesser::ConfigurationBase {
public:
    using sackmesser::ConfigurationBase::ConfigurationBase;

    bool load(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server) {
        return config_server->loadParameter(ns_ + "canny_low", &canny_low, true, 0) &&            //
               config_server->loadParameter(ns_ + "canny_high", &canny_high, true, 0) &&          //
               config_server->loadParameter(ns_ + "line_thickness", &line_thickness, true, 0) &&  //
               config_server->loadParameter(ns_ + "threshold", &threshold, true, 0) &&            //
               config_server->loadParameter(ns_ + "kernel_size", &kernel_size, true, 0);
    }

public:
    int canny_low;
    int canny_high;
    int line_thickness;
    int threshold;
    int kernel_size;
};

ImageProcessing::Contours::Contours(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name) : config_(std::make_unique<Configuration>(name + "contours/")) {
    config_->load(config_server);
}

ImageProcessing::Contours::~Contours() {}

void ImageProcessing::Contours::apply(cv::Mat& image) {
    cv::threshold(image, image, config_->threshold, 255, cv::THRESH_TOZERO);

    cv::GaussianBlur(image, image, cv::Size(config_->kernel_size, config_->kernel_size), 0.0, 0.0);

    cv::Mat canny;
    cv::Canny(image, canny, config_->canny_low, config_->canny_high);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    findContours(canny, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat drawing = cv::Mat::zeros(canny.size(), CV_8UC1);
    for (unsigned i = 0; i < contours.size(); i++) {
        drawContours(drawing, contours, (int)i, 255, config_->line_thickness, cv::LINE_8, hierarchy, 0);
    }

    image = drawing;
}

}  // namespace sketching

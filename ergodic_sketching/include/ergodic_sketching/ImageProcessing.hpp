// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <Eigen/Core>
#include <memory>
#include <opencv2/opencv.hpp>
#include <sackmesser/ClassFactory.hpp>
#include <vector>

namespace sackmesser {
class ConfigurationServer;
}  // namespace sackmesser

namespace sketching {
class ImageProcessing {
public:
    explicit ImageProcessing(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name = "");

    virtual ~ImageProcessing();

    void process(cv::Mat& image) const;

protected:
private:
    class Processor {
    public:
        Processor();

        virtual ~Processor();

        virtual void apply(cv::Mat& image) = 0;

        using Ptr = std::unique_ptr<Processor>;

        static sackmesser::Factory<Processor>& Factory();

    private:
        static sackmesser::Factory<Processor> factory_;
    };

    class Preprocessor : public Processor {
    public:
        Preprocessor(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~Preprocessor();

        void apply(cv::Mat& image) override;

    private:
        class Configuration;

        std::unique_ptr<Configuration> config_;
    };

    class GeometricWarping : public Processor {
    public:
        GeometricWarping(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~GeometricWarping();

        void apply(cv::Mat& image) override;

    private:
        class Mask {
        public:
            Mask();

            virtual ~Mask();

            void apply(cv::Mat& image);

            using Ptr = std::shared_ptr<Mask>;

        private:
            virtual std::vector<cv::Point2d> getNodesLower(const double& w, const double& h) = 0;

            virtual std::vector<cv::Point2d> getNodesUpper(const double& w, const double& h) = 0;
        };

        class Hourglass : public Mask {
        public:
            Hourglass(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

            virtual ~Hourglass();

        private:
            std::vector<cv::Point2d> getNodesLower(const double& w, const double& h) override;

            std::vector<cv::Point2d> getNodesUpper(const double& w, const double& h) override;

            class Configuration;

            std::unique_ptr<Configuration> config_;
        };

        class Diamond : public Mask {
        public:
            Diamond(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

            virtual ~Diamond();

        private:
            std::vector<cv::Point2d> getNodesLower(const double& w, const double& h) override;

            std::vector<cv::Point2d> getNodesUpper(const double& w, const double& h) override;

            class Configuration;

            std::unique_ptr<Configuration> config_;
        };

        std::vector<Mask::Ptr> masks_;
    };

    class EdgeEnhancement : public Processor {
    public:
        EdgeEnhancement(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~EdgeEnhancement();

        void apply(cv::Mat& image) override;

    private:
        class Configuration;

        std::unique_ptr<Configuration> config_;
    };

    class Quantization : public Processor {
    public:
        Quantization(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~Quantization();

        void apply(cv::Mat& image) override;

    private:
        class Configuration;

        std::unique_ptr<Configuration> config_;
    };

    class PencilSketch : public Processor {
    public:
        PencilSketch(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~PencilSketch();

        void apply(cv::Mat& image) override;

    private:
        class Configuration;

        std::unique_ptr<Configuration> config_;
    };

    class HistogramNormalization : public Processor {
    public:
        HistogramNormalization(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~HistogramNormalization();

        void apply(cv::Mat& image) override;

    private:
    };

    class ContrastBrightnessEnhancement : public Processor {
    public:
        ContrastBrightnessEnhancement(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~ContrastBrightnessEnhancement();

        void apply(cv::Mat& image) override;

    private:
        class Configuration;

        std::unique_ptr<Configuration> config_;
    };

    class Binarization : public Processor {
    public:
        Binarization(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~Binarization();

        void apply(cv::Mat& image) override;

    private:
        class Configuration;

        std::unique_ptr<Configuration> config_;
    };

    class Contours : public Processor {
    public:
        Contours(const std::shared_ptr<sackmesser::ConfigurationServer>& config_server, const std::string& name);

        virtual ~Contours();

        void apply(cv::Mat& image) override;

    private:
        class Configuration;

        std::unique_ptr<Configuration> config_;
    };

    std::vector<Processor::Ptr> processors_;
};

}  // namespace sketching

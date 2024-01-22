// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Jeremy Maceiras  <jeremy.maceiras@idiap.ch>
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <ergodic_sketching/AgentConfiguration.hpp>
#include <ergodic_sketching/Configuration.hpp>
#include <ergodic_sketching/HeatEquationCoverage.hpp>
#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Interface.hpp>
#include <sackmesser/Visualiser.hpp>

#include <numeric>

namespace sketching {
double rbf(const Eigen::Vector2d& mean, const Eigen::Vector2d& x, const double& var) {
    return std::exp(-var * (x - mean).squaredNorm());
}

class CoverageDensity {
    const std::vector<HeatEquationCoverage::Agent::Ptr>* agents_;

public:
    CoverageDensity(const std::vector<HeatEquationCoverage::Agent::Ptr>* agents, const double& dx, const double& var) : agents_(agents), dx_(dx), var_(var) {}

    double operator()(Eigen::Index row, Eigen::Index col) const {
        Eigen::Vector2d mean = Eigen::Vector2d(static_cast<double>(col), static_cast<double>(row));

        /*double val = 0.0;

        for (const HeatEquationCoverage::Agent::Ptr& agent : *agents_) {
            val += rbf(mean, agent->getPosition() / dx_, var_);
        }*/

        double val = std::accumulate((*agents_).begin(), (*agents_).end(), 0.0,
                                     [&](double tmp_val, const HeatEquationCoverage::Agent::Ptr& agent) -> double { return tmp_val + rbf(mean, agent->getPosition() / dx_, var_); });

        return val;
    }

private:
    const double dx_;
    const double var_;
};

HeatEquationCoverage::HeatEquationCoverage(const std::shared_ptr<sackmesser::Interface>& interface, const std::string& name)
    : ErgodicControl(interface, name + "hedac/"), config_(std::make_unique<Configuration>(name + "hedac/")) {
    if (!config_->load(interface->configServer())) {
        throw "HeatEquationCoverage: failed to configure";
    }
}

HeatEquationCoverage::~HeatEquationCoverage() {}

class LocalCooling {
    const std::vector<HeatEquationCoverage::Agent::Ptr>* agents_;

public:
    LocalCooling(const std::vector<HeatEquationCoverage::Agent::Ptr>* agents, const double& dx, const double& var) : agents_(agents), dx_(dx), var_(var) {}

    double operator()(Eigen::Index row, Eigen::Index col) const {
        Eigen::Vector2d x = Eigen::Vector2d(static_cast<double>(col), static_cast<double>(row));
        /*
        double val = 0.0;

        for (const HeatEquationCoverage::Agent::Ptr& agent : *agents_) {
            val += rbf(agent->getPosition() / dx_, x, var_);
        }
        */
        double val = std::accumulate((*agents_).begin(), (*agents_).end(), 0.0,
                                     [&](double tmp_val, const HeatEquationCoverage::Agent::Ptr& agent) -> double { return tmp_val + rbf(agent->getPosition() / dx_, x, var_); });

        return val;
    }

private:
    const double dx_;
    const double var_;
};

double bilinearIneterpolate(const cv::Mat& img, Eigen::Vector2d pt) {
    int x = (int)pt.x();
    int y = (int)pt.y();

    int x0 = cv::borderInterpolate(x, img.cols, cv::BORDER_REFLECT_101);
    int x1 = cv::borderInterpolate(x + 1, img.cols, cv::BORDER_REFLECT_101);
    int y0 = cv::borderInterpolate(y, img.rows, cv::BORDER_REFLECT_101);
    int y1 = cv::borderInterpolate(y + 1, img.rows, cv::BORDER_REFLECT_101);

    float a = (float)pt.x() - (float)x;
    float c = (float)pt.y() - (float)y;

    return ((img.at<float>(y0, x0) * (1.f - a) + img.at<float>(y0, x1) * a) * (1.f - c) + (img.at<float>(y1, x0) * (1.f - a) + img.at<float>(y1, x1) * a) * c);
}

Eigen::MatrixXd HeatEquationCoverage::compute(const Eigen::MatrixXd& distribution, const std::vector<Agent::Ptr>& agents, const unsigned& timesteps) {
    int height = static_cast<int>(distribution.rows());
    int width = static_cast<int>(distribution.cols());

    const double& dx = stepSize();

    double area = dx * static_cast<double>(width) * dx * static_cast<double>(height);

    Eigen::MatrixXd goal_density = distribution.normalized();
    Eigen::MatrixXd coverage_density = Eigen::MatrixXd::Zero(height, width);
    Eigen::MatrixXd heat = goal_density;

    double dt = std::min(1.0, (dx * dx) / (4.0 * config_->diffusion));
    // double coverage_rbf_var = 1.0 / config_->agent_radius;
    // double cooling_rbf_var = 1.0 / config_->cooling_radius;

    int size = 31;

    Eigen::MatrixXd cover_kernel = Eigen::MatrixXd::NullaryExpr(size, size, [this](Eigen::Index row, Eigen::Index col) {
        return rbf(Eigen::Vector2d(static_cast<double>(col), static_cast<double>(row)), Eigen::Vector2d(15, 15), 1.0 / config_->agent_radius);
    });

    Eigen::MatrixXd cooling_kernel = Eigen::MatrixXd::NullaryExpr(size, size, [this](Eigen::Index row, Eigen::Index col) {
        return rbf(Eigen::Vector2d(static_cast<double>(col), static_cast<double>(row)), Eigen::Vector2d(15, 15), 1.0 / config_->cooling_radius);
    });

    for (unsigned t = 0; t < timesteps && isComputing(); ++t) {
        Eigen::MatrixXd local_cooling = Eigen::MatrixXd::Zero(height, width);

        // coverage_density += Eigen::MatrixXd::NullaryExpr(height, width, CoverageDensity(&agents, dx, coverage_rbf_var));
        // Eigen::MatrixXd local_cooling = Eigen::MatrixXd::NullaryExpr(height, width, LocalCooling(&agents, dx, cooling_rbf_var)).normalized();

        for (const Agent::Ptr& agent : agents) {
            Eigen::Vector2d adjusted_position = agent->getPosition() / dx;
            int row = static_cast<int>(adjusted_position.y());
            int col = static_cast<int>(adjusted_position.x());

            int row_start_local = 0;
            int num_rows = size;
            int row_start_global = row - num_rows / 2;

            int col_start_local = 0;
            int num_cols = size;
            int col_start_global = col - num_cols / 2;

            if (row_start_global < 0) {
                row_start_local = num_rows / 2 - row - 1;
                num_rows = size - row_start_local - 1;
                row_start_global = 0;
            } else if (row_start_global + num_rows >= height) {
                num_rows -= row - (height - num_rows / 2 - 1);
            }

            if (col_start_global < 0) {
                col_start_local = num_cols / 2 - col - 1;
                num_cols = size - col_start_local - 1;
                col_start_global = 0;
            } else if (col_start_global + num_cols >= width) {
                num_cols -= col - (width - num_cols / 2 - 1);
            }

            coverage_density.block(row_start_global, col_start_global, num_rows, num_cols) += cover_kernel.block(row_start_local, col_start_local, num_rows, num_cols);

            local_cooling.block(row_start_global, col_start_global, num_rows, num_cols) += cooling_kernel.block(row_start_local, col_start_local, num_rows, num_cols);
        }

        local_cooling.normalize();

        Eigen::MatrixXd coverage = coverage_density.normalized();

        Eigen::MatrixXd source = area * (goal_density - coverage).unaryExpr([](double x) { return std::pow(std::max(x, 0.0), 2.0); }).normalized();

        Eigen::MatrixXd current_heat = Eigen::MatrixXd::Zero(height, width);

        for (int i = 1; i < height - 1; ++i) {
            for (int j = 1; j < width - 1; ++j) {
                current_heat(i, j) = dt * (config_->diffusion * (heat(i + 1, j) + heat(i - 1, j) + heat(i, j + 1) + heat(i, j - 1) - 4.0 * heat(i, j)) / (dx * dx)  //
                                           + config_->source_strength * source(i, j)                                                                                //
                                           - config_->local_cooling * local_cooling(i, j))                                                                          //
                                     + heat(i, j);                                                                                                                  //
            }
        }

        heat = current_heat;

        cv::Mat matrix_image;
        cv::Mat matrix_gradient_x;
        cv::Mat matrix_gradient_y;
        cv::eigen2cv(heat, matrix_image);

        matrix_image.convertTo(matrix_image, CV_32F);

        cv::Scharr(matrix_image, matrix_gradient_x, CV_32F, 1, 0, 7);
        cv::Scharr(matrix_image, matrix_gradient_y, CV_32F, 0, 1, 7);

        for (const HeatEquationCoverage::Agent::Ptr& agent : agents) {
            Eigen::Vector2d gradient = calculateGradient(matrix_gradient_x, matrix_gradient_y, agent->getPosition(), width, height, dx);

            agent->updateState(gradient);
        }

        interface_->visualiser()->show(heat, "heat");
    }

    return coverage_density;
}

Eigen::Vector2d HeatEquationCoverage::calculateGradient(const cv::Mat& gradient_x, const cv::Mat& gradient_y, const Eigen::Vector2d& position, const int& width, const int& height, const double& dx) {
    Eigen::Vector2d adjusted_position = position / dx;

    int row = static_cast<int>(adjusted_position.y());
    int col = static_cast<int>(adjusted_position.x());

    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();

    if (row > 0 && row < height - 1 && col > 0 && col < width - 1) {
        gradient.x() = bilinearIneterpolate(gradient_x, adjusted_position);
        gradient.y() = bilinearIneterpolate(gradient_y, adjusted_position);
    }

    if (row <= 0) {
        gradient.y() = 0.1;
    } else if (row >= height - 1) {
        gradient.y() = -0.1;
    }

    if (col <= 0) {
        gradient.x() = 0.1;
    } else if (col >= width - 1) {
        gradient.x() = -0.1;
    }

    return gradient;
}

}  // namespace sketching

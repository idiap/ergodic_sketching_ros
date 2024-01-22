// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Logger.hpp>

namespace sackmesser {
ConfigurationServer::ConfigurationServer(const std::shared_ptr<Logger>& logger) : logger_(logger) {}

ConfigurationServer::~ConfigurationServer() {}

bool ConfigurationServer::loadParameter(const std::string& name, bool* param, bool is_dynamic) {
    if (!load(name, param)) {
        return false;
    }

    addParameter(name, param, is_dynamic, false, true);

    if (!checkParameter(name, *param)) {
        return false;
    }

    return true;
}

bool ConfigurationServer::loadParameter(const std::string& name, int* param, bool is_dynamic, const int& min_val, const int& max_val) {
    if (!load(name, param)) {
        return false;
    }

    addParameter(name, param, is_dynamic, min_val, max_val);

    if (!checkParameter(name, *param)) {
        return false;
    }

    return true;
}

bool ConfigurationServer::loadParameter(const std::string& name, unsigned* param, bool /*is_dynamic*/, const unsigned& /*max_val*/) {
    // addParameter(name, param, is_dynamic, static_cast<unsigned>(0), max_val);

    if (!load(name, param)) {
        return false;
    }

    // if (!checkParameter(name, *param))
    // {
    //     return false;
    // }

    return true;
}

bool ConfigurationServer::loadParameter(const std::string& name, double* param, bool is_dynamic, const double& min_val, const double& max_val) {
    if (!load(name, param)) {
        return false;
    }

    addParameter(name, param, is_dynamic, min_val, max_val);

    if (!checkParameter(name, *param)) {
        return false;
    }

    return true;
}

bool ConfigurationServer::loadParameter(const std::string& name, std::string* param) {
    return load(name, param);
}

bool ConfigurationServer::loadParameter(const std::string& name, std::map<std::string, double>* param) {
    return load(name, param);
}

bool ConfigurationServer::loadParameter(const std::string& name, std::vector<std::string>* param) {
    return load(name, param);
}

bool ConfigurationServer::loadParameter(const std::string& name, std::vector<double>* param) {
    return load(name, param);
}

std::map<std::string, boost::any>* ConfigurationServer::getDynamicParameters() {
    return &parameters_dynamic_;
}

const std::map<std::string, boost::any>& ConfigurationServer::getParametersMinimum() {
    return parameters_min_val_;
}

const std::map<std::string, boost::any>& ConfigurationServer::getParametersMaximum() {
    return parameters_max_val_;
}

const std::map<std::string, boost::any>& ConfigurationServer::getParametersDefault() {
    return parameters_default_;
}

}  // namespace sackmesser

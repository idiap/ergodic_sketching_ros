// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <yaml-cpp/yaml.h>

#include <sackmesser/utility.hpp>
#include <sackmesser_runtime/ConfigurationServerYAML.hpp>

namespace sackmesser::runtime {
class ConfigurationServerYAML::Impl {
public:
    Impl(const std::string& file, const std::shared_ptr<Logger>& logger);

    ~Impl();

    bool load(const std::string& n, bool* p);

    bool load(const std::string& n, double* p);

    bool load(const std::string& n, int* p);

    bool load(const std::string& n, unsigned* p);

    bool load(const std::string& n, std::string* p);

    bool load(const std::string& n, std::map<std::string, double>* p);

    bool load(const std::string& n, std::vector<std::string>* p);

    bool load(const std::string& n, std::vector<double>* p);

private:
    template <class Type>
    bool find(const std::string& name, Type* param) const;

private:
    YAML::Node file_;

    std::shared_ptr<Logger> logger_;
};

template <class Type>
bool ConfigurationServerYAML::Impl::find(const std::string& name, Type* param) const {
    std::vector<std::string> node_names = split_string(name, '/');

    YAML::Node node = YAML::Clone(file_);

    for (const std::string& s : node_names) {
        if (!node[s]) {
            logger_->msg("ConfigurationServer: no parameter named " + s, LOG::FATAL);
            return false;
        }

        node = node[s];
    }

    *param = node.as<Type>();

    logger_->msg("ConfigurationServer: loaded " + name, LOG::INFO_VERBOSE);

    return true;
}

}  // namespace sackmesser::runtime

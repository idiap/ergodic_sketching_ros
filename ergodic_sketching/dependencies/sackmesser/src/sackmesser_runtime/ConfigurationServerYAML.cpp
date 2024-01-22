// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <sackmesser_runtime/ConfigurationServerYAML.hpp>
#include <sackmesser_runtime/impl/ConfigurationServerYAML.hpp>

namespace sackmesser::runtime {
ConfigurationServerYAML::ConfigurationServerYAML(const std::string& file, const std::shared_ptr<Logger>& logger) : ConfigurationServer(logger) {
    impl_ = std::make_unique<Impl>(file, logger);
}

ConfigurationServerYAML::~ConfigurationServerYAML() {}

bool ConfigurationServerYAML::load(const std::string& n, bool* p) {
    return impl_->load(n, p);
}

bool ConfigurationServerYAML::load(const std::string& n, double* p) {
    return impl_->load(n, p);
}

bool ConfigurationServerYAML::load(const std::string& n, int* p) {
    return impl_->load(n, p);
}

bool ConfigurationServerYAML::load(const std::string& n, unsigned* p) {
    return impl_->load(n, p);
}

bool ConfigurationServerYAML::load(const std::string& n, std::string* p) {
    return impl_->load(n, p);
}

bool ConfigurationServerYAML::load(const std::string& n, std::map<std::string, double>* p) {
    return impl_->load(n, p);
}

bool ConfigurationServerYAML::load(const std::string& n, std::vector<std::string>* p) {
    return impl_->load(n, p);
}

bool ConfigurationServerYAML::load(const std::string& n, std::vector<double>* p) {
    return impl_->load(n, p);
}

}  // namespace sackmesser::runtime

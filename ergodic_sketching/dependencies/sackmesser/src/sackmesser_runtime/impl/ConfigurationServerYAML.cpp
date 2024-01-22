// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <filesystem>
#include <sackmesser_runtime/impl/ConfigurationServerYAML.hpp>

namespace sackmesser::runtime {
ConfigurationServerYAML::Impl::Impl(const std::string& file, const std::shared_ptr<Logger>& logger) : logger_(logger) {
    if (!std::filesystem::exists(std::filesystem::path(file))) {
        logger->msg("file '" + file + "' does not exist", LOG::FATAL);
    }

    file_ = YAML::LoadFile(file);
}

ConfigurationServerYAML::Impl::~Impl() {}

bool ConfigurationServerYAML::Impl::load(const std::string& n, bool* p) {
    return find<bool>(n, p);
}

bool ConfigurationServerYAML::Impl::load(const std::string& n, double* p) {
    return find<double>(n, p);
}

bool ConfigurationServerYAML::Impl::load(const std::string& n, int* p) {
    return find<int>(n, p);
}

bool ConfigurationServerYAML::Impl::load(const std::string& n, unsigned* p) {
    return find<unsigned>(n, p);
}

bool ConfigurationServerYAML::Impl::load(const std::string& n, std::string* p) {
    return find<std::string>(n, p);
}

bool ConfigurationServerYAML::Impl::load(const std::string& n, std::map<std::string, double>* p) {
    return find<std::map<std::string, double>>(n, p);
}

bool ConfigurationServerYAML::Impl::load(const std::string& n, std::vector<std::string>* p) {
    return find<std::vector<std::string>>(n, p);
}

bool ConfigurationServerYAML::Impl::load(const std::string& n, std::vector<double>* p) {
    return find<std::vector<double>>(n, p);
}
}  // namespace sackmesser::runtime

// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <filesystem>
#include <sackmesser/Logger.hpp>
#include <sackmesser_runtime/CallbackHandler.hpp>
#include <sackmesser_runtime/ConfigurationServerYAML.hpp>
#include <sackmesser_runtime/Interface.hpp>

namespace sackmesser::runtime {
Interface::Interface(const std::string& path, const std::string& config_file, const std::shared_ptr<Logger>& logger)
    : sackmesser::Interface(std::make_shared<ConfigurationServerYAML>(std::filesystem::path(path) / ("config/" + config_file), logger), std::make_shared<CallbackHandler>(logger), logger, path) {}

Interface::~Interface() {}

}  // namespace sackmesser::runtime

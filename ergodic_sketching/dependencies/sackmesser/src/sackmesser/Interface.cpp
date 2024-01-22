// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <filesystem>
#include <sackmesser/CallbackHandler.hpp>
#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Interface.hpp>
#include <sackmesser/Logger.hpp>
#include <sackmesser/Visualiser.hpp>

namespace sackmesser {
Interface::Interface(const std::shared_ptr<ConfigurationServer>& config_server,
                     const std::shared_ptr<CallbackHandler>& callback_handler,
                     const std::shared_ptr<Logger>& logger,
                     const std::string& path)
    : config_server_(config_server), callback_handler_(callback_handler), logger_(logger) {
    visualiser_ = std::make_shared<sackmesser::Visualiser>(logger);

    config_server->loadParameter("input_folder", &input_folder_);

    if (!std::filesystem::exists(std::filesystem::path(path))) {
        logger->msg("path '" + path + "' does not exist", LOG::FATAL);
    }

    input_folder_ = std::filesystem::path(std::filesystem::path(path) / input_folder_) / "";

    output_folder_ = std::filesystem::path(path) / "output/";

    if (!std::filesystem::exists(std::filesystem::path(output_folder_))) {
        std::filesystem::create_directories(std::filesystem::path(output_folder_));
    }

    std::time_t current_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream current_folder;
    current_folder << std::put_time(std::localtime(&current_time), "%F_%H-%M-%S");
    time_string_ = current_folder.str();
}

Interface::~Interface() {}

std::shared_ptr<ConfigurationServer> Interface::configServer() {
    return config_server_;
}

std::shared_ptr<Visualiser> Interface::visualiser() {
    return visualiser_;
}

std::shared_ptr<CallbackHandler> Interface::callbacks() {
    return callback_handler_;
}

std::shared_ptr<Logger> Interface::log() {
    return logger_;
}

const std::string& Interface::inputPath() const {
    return input_folder_;
}

const std::string& Interface::outputPath() const {
    return output_folder_;
}

const std::string& Interface::timeString() const {
    return time_string_;
}

}  // namespace sackmesser

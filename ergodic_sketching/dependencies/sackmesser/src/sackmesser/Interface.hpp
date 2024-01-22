// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <memory>
#include <string>

namespace sackmesser {
class Visualiser;
class ConfigurationServer;
class CallbackHandler;
class Logger;

class Interface {
public:
    Interface(const std::shared_ptr<ConfigurationServer>& config_server, const std::shared_ptr<CallbackHandler>& callback_handler, const std::shared_ptr<Logger>& logger, const std::string& path);

    virtual ~Interface();

    std::shared_ptr<ConfigurationServer> configServer();

    std::shared_ptr<Visualiser> visualiser();

    std::shared_ptr<CallbackHandler> callbacks();

    std::shared_ptr<Logger> log();

    const std::string& inputPath() const;

    const std::string& outputPath() const;

    const std::string& timeString() const;

private:
    std::shared_ptr<ConfigurationServer> config_server_;

    std::shared_ptr<Visualiser> visualiser_;

    std::shared_ptr<CallbackHandler> callback_handler_;

    std::shared_ptr<Logger> logger_;

    std::string input_folder_;

    std::string output_folder_;

    std::string time_string_;

public:
    using Ptr = std::shared_ptr<Interface>;
};

}  // namespace sackmesser

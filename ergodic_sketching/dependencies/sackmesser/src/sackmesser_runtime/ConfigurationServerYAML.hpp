// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <sackmesser/ConfigurationServer.hpp>

namespace sackmesser::runtime {
class ConfigurationServerYAML : public ConfigurationServer {
public:
    ConfigurationServerYAML(const std::string& file, const std::shared_ptr<Logger>& logger);

    virtual ~ConfigurationServerYAML();

protected:
private:
    bool load(const std::string& n, bool* p);

    bool load(const std::string& n, double* p);

    bool load(const std::string& n, int* p);

    bool load(const std::string& n, unsigned* p);

    bool load(const std::string& n, std::string* p);

    bool load(const std::string& n, std::map<std::string, double>* p);

    bool load(const std::string& n, std::vector<std::string>* p);

    bool load(const std::string& n, std::vector<double>* p);

public:
    using Ptr = std::shared_ptr<ConfigurationServerYAML>;

private:
    class Impl;

    std::unique_ptr<Impl> impl_;
};

}  // namespace sackmesser::runtime

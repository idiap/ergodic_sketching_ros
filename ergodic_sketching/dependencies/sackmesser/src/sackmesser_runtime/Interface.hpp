// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <sackmesser/Interface.hpp>

namespace sackmesser {
class Logger;
}

namespace sackmesser::runtime {
class Interface : public sackmesser::Interface {
public:
    Interface(const std::string& path, const std::string& config_file, const std::shared_ptr<Logger>& logger = std::make_shared<Logger>());

    virtual ~Interface();

protected:
private:
};

}  // namespace sackmesser::runtime

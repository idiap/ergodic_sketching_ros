// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <sackmesser/CallbackHandler.hpp>

namespace sackmesser::runtime {
class CallbackHandler : public sackmesser::CallbackHandler {
public:
    CallbackHandler(const std::shared_ptr<Logger>& logger) : sackmesser::CallbackHandler(logger) {}

    virtual ~CallbackHandler() {}

    void add(const std::string& /*name*/, const double& /*frequency*/, const std::function<void()>& /*callback*/) { logger_->msg("CallbackHandler::add NOT IMPLEMENTED", LOG::ERROR); }

protected:
private:
};

}  // namespace sackmesser::runtime

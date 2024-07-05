// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <iostream>
#include <sackmesser/Logger.hpp>

namespace sackmesser {
Logger::Logger(std::function<void(const std::string&)> info,  //
               std::function<void(const std::string&)> warn,  //
               std::function<void(const std::string&)> error)
    : info_(info), warn_(warn), error_(error), is_enabled_(true), verbose_(false), debug_(false) {
    start_time_ = std::chrono::high_resolution_clock::now();
}

Logger::~Logger() {}

void Logger::msg(const std::string& message, LOG level) {
    if (!is_enabled_ && level != LOG::FATAL) {
        return;
    }

    switch (level) {
        case LOG::INFO: {
            info_(message);
            break;
        }
        case LOG::WARN: {
            warn_(message);
            break;
        }
        case LOG::ERROR: {
            error_(message);
            break;
        }
        case LOG::FATAL: {
            error_(message);
            exit(-1);
            break;
        }
        case LOG::INFO_VERBOSE: {
            if (verbose_) {
                info_("VERBOSE " + message);
            }
            break;
        }
        case LOG::WARN_VERBOSE: {
            if (verbose_) {
                warn_("VERBOSE " + message);
            }
            break;
        }
        case LOG::ERROR_VERBOSE: {
            if (verbose_) {
                error_("VERBOSE " + message);
            }
            break;
        }
        case LOG::INFO_DEBUG: {
            if (debug_) {
                info_("DEBUG " + message);
            }
            break;
        }
        case LOG::WARN_DEBUG: {
            if (debug_) {
                warn_("DEBUG " + message);
            }
            break;
        }
        case LOG::ERROR_DEBUG: {
            if (debug_) {
                error_("DEBUG " + message);
            }
            break;
        }
    }
}

void Logger::enable() {
    is_enabled_ = true;
}

void Logger::disable() {
    is_enabled_ = false;
}

void Logger::enableDebug() {
    debug_ = true;
}

void Logger::enableVerbose() {
    verbose_ = false;
}

void Logger::disableDebug() {
    debug_ = true;
}

void Logger::disableVerbose() {
    verbose_ = false;
}

void Logger::data(const std::string& id, const double& value) {
    data_.emplace(std::make_pair(id, value));
}

}  // namespace sackmesser

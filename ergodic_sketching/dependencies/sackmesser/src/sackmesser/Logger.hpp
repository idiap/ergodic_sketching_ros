// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <string>

namespace sackmesser {
/** \addtogroup LoggerLevels
 * @{
 */
/**
 * @brief enum for all levels of logging
 */
enum class LOG { INFO, WARN, ERROR, FATAL, INFO_VERBOSE, WARN_VERBOSE, ERROR_VERBOSE, INFO_DEBUG, WARN_DEBUG, ERROR_DEBUG };
/**
 * @}
 * */

/**
 * @brief logger class to handle message
 */
class Logger {
public:
public:
    Logger(
        std::function<void(const std::string&)> info = [](const std::string& message) { std::cout << "\033[37mINFO " << message << std::endl
                                                                                                  << "\033[37m"; },  //
        std::function<void(const std::string&)> warn = [](const std::string& message) { std::cout << "\033[33mWARN " << message << std::endl
                                                                                                  << "\033[37m"; },  //
        std::function<void(const std::string&)> error = [](const std::string& message) { std::cout << "\033[31mERROR " << message << std::endl
                                                                                                   << "\033[37m"; });

    /**
     * @brief default destructor
     */
    virtual ~Logger();

    /**
     * @brief call message functions based on level
     * @param level message level INFO, WARN, ERROR
     * @param message to be printed
     */
    void msg(const std::string& message, LOG level = LOG::INFO);

    /**
     * @brief enable logging
     */
    void enable();

    void enableDebug();

    void enableVerbose();

    /**
     * @brief disable logging
     */
    void disable();

    void disableDebug();

    void disableVerbose();

    /**
     * @brief output the time a function takes to complete
     * @param function function that should be timed
     * @param success_msg message to display on success
     * @param failure_msg message to display on failure
     * @return true if success, false if failure
     */
    template <typename Function>
    inline bool timeFunction(const Function& /*function*/, const std::string& /*success_msg*/, const std::string& /*failure_msg*/) {
        error_("timeFunction NOT IMPLEMENTED");

        return false;
    }

    /**
     * @brief output the time a functino takes to complete on average
     * @param function function that should be timed
     * @param msg message that gets displayed with the average time
     * @param id function id to keep track of different functions
     * @param count number of calls for that functions before the average gets displayed
     */
    template <typename Function>
    inline void timeFunctionAverage(const Function& function, const std::string& msg, const std::string& id, unsigned count) {
        static std::map<std::string, std::pair<unsigned, double>> times;

        function();
        times[id].first++;
        times[id].second += 0.0;

        if (times[id].first >= count) {
            info_(msg + " takes " + std::to_string(times[id].second / static_cast<double>(count)) + "s on average");

            times[id].first = 0;
            times[id].second = 0.0;
        }
    }

    void data(const std::string& id, const double& value);

protected:
private:
    /**
     * @brief dumps info message
     */
    std::function<void(const std::string&)> info_;

    /**
     * @brief dumps warn message
     */
    std::function<void(const std::string&)> warn_;

    /**
     * @brief dumps error message
     */
    std::function<void(const std::string&)> error_;

    /// flag to keep track if logging is enabled
    bool is_enabled_;

    bool verbose_;

    bool debug_;

    /// keep track of the time logging was started, to give each message a time stamp
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;

    std::map<std::string, double> data_;
};

}  // namespace sackmesser

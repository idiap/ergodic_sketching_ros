// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <sackmesser/Logger.hpp>
#include <string>
#include <vector>

namespace sackmesser {
class CallbackQueueBase {
public:
    CallbackQueueBase(const std::string& name) : name_(name) {}

    virtual ~CallbackQueueBase() {}

    template <class Type>
    Type* cast() {
        return dynamic_cast<Type*>(this);
    }

    const std::string& getName() const { return name_; }

private:
    std::string name_;
};

template <class Type>
class CallbackQueue : public CallbackQueueBase {
public:
    /**
     * @brief default constructor
     */
    using CallbackQueueBase::CallbackQueueBase;

    /**
     * @brief destructor
     */
    virtual ~CallbackQueue() {}

    void add(const std::function<Type>& callback) { callbacks_.push_back(callback); }

protected:
    std::vector<std::function<Type>> callbacks_;
};

class TimedCallback : public CallbackQueueBase {
public:
    TimedCallback(const std::string& name) : CallbackQueueBase(name) {}

    virtual ~TimedCallback() {}

    virtual void start() = 0;

    virtual void stop() = 0;
};

class CallbackHandler {
public:
    CallbackHandler(const std::shared_ptr<Logger>& logger) : logger_(logger) {}

    virtual ~CallbackHandler() {}

    void add(std::unique_ptr<CallbackQueueBase>&& queue) { queues_.emplace(queue->getName(), std::move(queue)); }

    template <typename Type>
    void add(const std::string& name, const std::function<Type>& callback) {
        if (queues_.find(name) != queues_.end()) {
            CallbackQueue<Type>* queue = queues_.at(name)->cast<CallbackQueue<Type>>();

            if (queue) {
                queue->add(callback);
            } else {
                // LOG_ERROR("Visualiser: unable to visualise " + name);
                logger_->msg("unable to add callback " + name);
            }
        }
    }

    void startTimer(const std::string& name) {
        if (queues_.find(name) != queues_.end()) {
            TimedCallback* timer = queues_.at(name)->cast<TimedCallback>();

            if (timer) {
                timer->start();
            }
        }
    }

    void stopTimer(const std::string& name) {
        if (queues_.find(name) != queues_.end()) {
            TimedCallback* timer = queues_.at(name)->cast<TimedCallback>();

            if (timer) {
                timer->stop();
            }
        }
    }

    virtual void add(const std::string& name, const double& frequency, const std::function<void()>& callback) = 0;

protected:
    std::shared_ptr<Logger> logger_;

private:
    std::map<std::string, std::unique_ptr<CallbackQueueBase>> queues_;
};

}  // namespace sackmesser

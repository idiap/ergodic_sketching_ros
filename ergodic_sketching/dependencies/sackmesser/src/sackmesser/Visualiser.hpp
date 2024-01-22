// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <functional>
#include <memory>
#include <sackmesser/ConfigurationServer.hpp>
#include <sackmesser/Logger.hpp>

namespace sackmesser {
/**
 * @brief visualiser class (singleton)
 */
class Visualiser {
public:
    template <class Type>
    class Visualisation;

private:
    class VisualisationBase {
    public:
        /**
         * @brief default constructor
         */
        VisualisationBase() {}

        /**
         * @brief destructor
         */
        virtual ~VisualisationBase() {}

        template <class Type>
        const Visualisation<Type>* cast() {
            return dynamic_cast<const Visualisation<Type>*>(this);
        }

        virtual const std::string& getName() const = 0;
    };

public:
    /**
     * @brief base class for all visualisation functions
     * @details implement this class to visualise the different objects
     */
    template <class Type>
    class Visualisation : public VisualisationBase {
    public:
        /**
         * @brief default constructor
         */
        Visualisation(const std::string& name, const std::shared_ptr<ConfigurationServer>& server, std::unique_ptr<ConfigurationBase>&& config) : name_(name) {
            config_ = std::move(config);

            config_->load(server);
        }

        /**
         * @brief destructor
         */
        virtual ~Visualisation() {}

        virtual void show(const Type& value) const = 0;

        const std::string& getName() const { return name_; }

    protected:
        std::unique_ptr<ConfigurationBase> config_;

    private:
        std::string name_;
    };

    /**
     * @brief
     */
    Visualiser(const std::shared_ptr<Logger>& logger);

    /**
     * @brief destructor
     */
    ~Visualiser();

    bool add(std::unique_ptr<VisualisationBase>&& visualisation);

    /**
     * @brief public template function that is called from outside
     * @details calls the correct function of the Visualisation implementation
     *
     * @param value object to visualise
     * @param name name of the object
     * @param debug set to true if the object should only be visualised in debug mode
     */
    template <typename Type>
    void show(const Type& value, const std::string& name) const {
        if (visualisations_.find(name) != visualisations_.end()) {
            const Visualisation<Type>* vis = visualisations_.at(name)->cast<Type>();

            if (vis) {
                vis->show(value);
            } else {
                logger_->msg("Visualiser: unable to visualise " + name, LOG::WARN);
            }
        }
    }

    /**
     * @brief configure the Visualiser
     * @details calls the configure function of the Visualisation
     * @return true if successful
     */
    bool configure();

protected:
private:
    /// Visualisation instance
    std::map<std::string, std::unique_ptr<VisualisationBase>> visualisations_;

    std::shared_ptr<Logger> logger_;
};

}  // namespace sackmesser

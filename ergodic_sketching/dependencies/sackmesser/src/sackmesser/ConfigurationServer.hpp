// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#pragma once

#include <algorithm>
#include <boost/any.hpp>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <sackmesser/Logger.hpp>
#include <string>
#include <vector>

namespace sackmesser {
class ConfigurationServer;

/**
 * @brief base configuration class, all local configuration should inherit from this class
 */
class ConfigurationBase {
public:
    /**
     * @brief constructor
     */
    ConfigurationBase() = delete;

    /**
     * @brief constructor with namespace
     * @param ns namespace of all parameters (in the yaml file)
     */
    ConfigurationBase(const std::string& ns) : ns_(ns) {}

    /**
     * @brief destructor
     */
    virtual ~ConfigurationBase() = default;

    /**
     * @brief load configuration parameters
     * @return true if all parameters were found
     */
    virtual bool load(const std::shared_ptr<ConfigurationServer>& server) = 0;

    /**
     * @brief cast the configuration pointer to a derived configuration to access its paramters
     * @details
     */
    template <class Derived>
    Derived* cast() {
        return dynamic_cast<Derived*>(this);
    }

protected:
    /// namespace of this configuration
    std::string ns_;

private:
};

/**
 * contains all configuration parameters related to the sackmesser library
 * */
class ConfigurationServer {
public:
    /**
     * @brief base configuration loader class
     * @details can be a custom implementation, is given to the ConfigurationServer singleton as a unique_ptr
     */
    // class Loader
    // {
    // public:
    /**
     * @brief default constructor
     */
    // Loader();

    /**
     * @brief virtual destructor
     * @details this class is only meant for inheritance
     */
    // virtual ~Loader();

    // };

    ConfigurationServer(const std::shared_ptr<Logger>& logger);
    /**
     * @brief destructor
     */
    virtual ~ConfigurationServer();

    /**
     * @brief access the class instance
     * @return point to the class instance
     */
    // static std::shared_ptr<ConfigurationServer> create(std::unique_ptr<Loader> &&);

    /**
     * @brief load the given configuration
     * @param config pointer to a configuration
     * @return true if all parameters of the configuration were found
     */
    // static bool load(ConfigurationBase *config);

    /**
     * @brief loads a bool parameter
     * @param name name of the parameter
     * @param param bool parameter
     * @param is_dynamic true if the parameter is reconfigurable
     * @return true if the paramter was found
     */
    bool loadParameter(const std::string& name, bool* param, bool is_dynamic);

    /**
     * @brief loads a int parameter
     * @param name name of the parameter
     * @param param int parameter
     * @param is_dynamic true if the parameter is reconfigurable
     * @param min_val lower limit of the parameter
     * @param max_val upper limit of the parameter
     * @return true if the paramter was found
     */
    bool loadParameter(const std::string& name, int* param, bool is_dynamic, const int& min_val = std::numeric_limits<int>::min(), const int& max_val = std::numeric_limits<int>::max());

    bool loadParameter(const std::string& name, unsigned* param, bool is_dynamic, const unsigned& max_val = std::numeric_limits<unsigned>::max());

    /**
     * @brief loads a double parameter
     * @param name name of the parameter
     * @param param double parameter
     * @param is_dynamic true if the parameter is reconfigurable
     * @param min_val lower limit of the parameter
     * @param max_val upper limit of the parameter
     * @return true if the paramter was found
     */
    bool loadParameter(const std::string& name,
                       double* param,
                       bool is_dynamic,
                       const double& min_val = -std::numeric_limits<double>::max(),
                       const double& max_val = std::numeric_limits<double>::max());

    /**
     * @brief loads a string parameter
     * @param name name of the parameter
     * @param param string parameter
     * @return true if the paramter was found
     */
    bool loadParameter(const std::string& name, std::string* param);

    /**
     * @brief loads a string-double map parameter
     * @param name name of the parameter
     * @param param string parameter
     * @return true if the paramter was found
     */
    bool loadParameter(const std::string& name, std::map<std::string, double>* param);

    /**
     * @brief loads a string-double map parameter
     * @param name name of the parameter
     * @param param string parameter
     * @return true if the paramter was found
     */
    bool loadParameter(const std::string& name, std::vector<double>* param);

    /**
     * @brief loads a string-double map parameter
     * @param name name of the parameter
     * @param param string parameter
     * @return true if the paramter was found
     */
    bool loadParameter(const std::string& name, std::vector<std::string>* param);

    /**
     * @brief reconfigure the parameter with the given name
     * @param name name of the parameter
     * @param new_value new value of the paramter
     * @return true if reconfigured successfully
     */
    template <typename Type>
    bool reconfigure(const std::string& name, const Type& new_value) {
        if (parameters_dynamic_.find(name) == parameters_dynamic_.end()) {
            logger_->msg("ConfigurationServer: unknown parameter " + name, LOG::ERROR);
            return false;
        }

        if (!checkParameter(name, new_value)) {
            return false;
        }

        *boost::any_cast<Type*>(parameters_dynamic_.at(name)) = new_value;

        logger_->msg("ConfigurationServer: reconfigured " + name + " -> " + std::to_string(new_value));

        return true;
    }

    /**
     * @brief return the newly created folder in the data path
     * @return folder name
     */
    static const std::string& getFolder();

    /**
     * @brief get pointer to all dynamic parameters
     * @return pointer to all dynamic parameters
     */
    std::map<std::string, boost::any>* getDynamicParameters();

    /**
     * @brief get reference to all minimum parameters
     * @return reference to all minimum parameters
     */
    const std::map<std::string, boost::any>& getParametersMinimum();

    /**
     * @brief get reference to all maximum parameters
     * @return reference to all maximum parameters
     */
    const std::map<std::string, boost::any>& getParametersMaximum();

    /**
     * @brief get reference to all default parameters
     * @return reference to all default parameters
     */
    const std::map<std::string, boost::any>& getParametersDefault();

    /**
     * @brief add a string parameter to the configuration
     * @param name name of the parameter
     * @param param value of the parameter
     */
    template <typename Type>
    void addParameter(const std::string& name, const Type& param) {
        parameters_default_.emplace(std::make_pair(name, param));
    }

private:
    /**
     * @brief placeholder function for loading bool variables
     * @param n name of the parameter (including namespace)
     * @param p pointer to the parameter
     * @return true if loaded successfully
     */
    virtual bool load(const std::string& n, bool* p) = 0;

    /**
     * @brief placeholder function for loading double variables
     * @param n name of the parameter (including namespace)
     * @param p pointer to the parameter
     * @return true if loaded successfully
     */
    virtual bool load(const std::string& n, double* p) = 0;

    /**
     * @brief placeholder function for loading int variables
     * @param n name of the parameter (including namespace)
     * @param p pointer to the parameter
     * @return true if loaded successfully
     */
    virtual bool load(const std::string& n, int* p) = 0;

    virtual bool load(const std::string& n, unsigned* p) = 0;

    /**
     * @brief placeholder function for loading std::string variables
     * @param n name of the parameter (including namespace)
     * @param p pointer to the parameter
     * @return true if loaded successfully
     */
    virtual bool load(const std::string& n, std::string* p) = 0;

    /**
     * @brief placeholder function for loading std::map<std::string, double> variables
     * @param n name of the parameter (including namespace)
     * @param p pointer to the parameter
     * @return true if loaded successfully
     */
    virtual bool load(const std::string& n, std::map<std::string, double>* p) = 0;

    /**
     * @brief placeholder function for loading std::vector<std::string> variables
     * @param n name of the parameter (including namespace)
     * @param p pointer to the parameter
     * @return true if loaded successfully
     */
    virtual bool load(const std::string& n, std::vector<std::string>* p) = 0;

    /**
     * @brief placeholder function for loading std::vector<std::string> variables
     * @param n name of the parameter (including namespace)
     * @param p pointer to the parameter
     * @return true if loaded successfully
     */
    virtual bool load(const std::string& n, std::vector<double>* p) = 0;

    /**
     * @brief check if the given parameter lies within the specified limits
     * @details limits are generally hardcoded
     * @param name name of the parameter (including namespace)
     * @param param oarameter reference
     * @return true if the parameter lies within the limits
     */
    template <typename Type>
    bool checkParameter(const std::string& name, const Type& param) {
        if (param < boost::any_cast<Type>(parameters_min_val_.at(name))) {
            logger_->msg("ConfigurationServer: " + name + " value (" + std::to_string(param) + ") smaller than minimum (" + std::to_string(boost::any_cast<Type>(parameters_min_val_.at(name))) + ")",
                         LOG::ERROR);
            return false;
        }

        if (param > boost::any_cast<Type>(parameters_max_val_.at(name))) {
            logger_->msg("ConfigurationServer: " + name + " value (" + std::to_string(param) + ") larger than maximum (" + std::to_string(boost::any_cast<Type>(parameters_max_val_.at(name))) + ")",
                         LOG::ERROR);
            return false;
        }

        return true;
    }

    /**
     * @brief add parameter to the list including its limits
     * @details the parameter is stored as a pointer to the correct configuration instance
     *
     * @param name name of the parameter (including namespace)
     * @param param pointer to the parameter
     * @param is_dynamic set to true if the parameter can be modified after loading
     * @param min_val minimum value of the parameter
     * @param max_val maximum value of the parameter
     */
    template <typename Type>
    void addParameter(const std::string& name, Type* param, bool is_dynamic, const Type& min_val, const Type& max_val) {
        parameters_default_.emplace(std::make_pair(name, *param));
        parameters_min_val_.emplace(std::make_pair(name, min_val));
        parameters_max_val_.emplace(std::make_pair(name, max_val));

        if (is_dynamic) {
            parameters_dynamic_.emplace(std::make_pair(name, param));
        }
    }

    /// stores all default parameters
    std::map<std::string, boost::any> parameters_default_;
    /// stores all minimum values of the parameters
    std::map<std::string, boost::any> parameters_min_val_;
    /// stores all maximum values of the parameters
    std::map<std::string, boost::any> parameters_max_val_;
    /// stores all dynamic parameters
    std::map<std::string, boost::any> parameters_dynamic_;

    std::shared_ptr<Logger> logger_;

public:
    using Ptr = std::shared_ptr<ConfigurationServer>;
};

}  // namespace sackmesser

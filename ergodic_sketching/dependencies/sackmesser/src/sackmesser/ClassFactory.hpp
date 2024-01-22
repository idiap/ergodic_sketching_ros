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
#include <string>

namespace sackmesser {
template <class Base>
class Factory {
public:
    Factory() {}

    virtual ~Factory() { loaders_.clear(); }

    class ClassLoaderBase {
    public:
        ClassLoaderBase() {}

        virtual ~ClassLoaderBase() {}

    protected:
    };

    template <class... Args>
    class ClassLoader : public ClassLoaderBase {
    public:
        using Creator = typename Base::Ptr (*)(Args...);

        ClassLoader(const Creator& create) : create_(create) {}

        virtual ~ClassLoader() {}

        template <class... T>
        typename Base::Ptr make(T&&... args) {
            return create_(std::forward<T>(args)...);
        }

    private:
        Creator create_;
    };

public:
    template <class... T>
    void add(const std::string& name, const typename ClassLoader<T...>::Creator& creator) {
        loaders_.emplace(std::make_pair(name, std::make_unique<ClassLoader<T...>>(creator)));
    }

    template <class... T>
    typename Base::Ptr create(const std::string& name, T&&... args) const {
        if (loaders_.find(name) != loaders_.end()) {
            ClassLoader<T...>* class_loader = dynamic_cast<ClassLoader<T...>*>(loaders_.at(name).get());

            if (!class_loader) {
                std::cout << "can't cast " << name << std::endl;
                return nullptr;
            }

            return class_loader->make(std::forward<T>(args)...);
        }

        std::cout << "can't find constructor for " << name << std::endl;

        return nullptr;
    }

private:
    std::map<std::string, std::unique_ptr<ClassLoaderBase>> loaders_;
};

}  // namespace sackmesser

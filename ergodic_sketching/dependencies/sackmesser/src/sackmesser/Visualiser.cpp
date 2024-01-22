// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <sackmesser/Logger.hpp>
#include <sackmesser/Visualiser.hpp>

namespace sackmesser {
Visualiser::Visualiser(const std::shared_ptr<Logger>& logger) : logger_(logger) {}

Visualiser::~Visualiser() {}

bool Visualiser::add(std::unique_ptr<VisualisationBase>&& visualisation) {
    if (visualisations_.find(visualisation->getName()) != visualisations_.end()) {
        logger_->msg("Visualiser: visualisation " + visualisation->getName() + " already exists", LOG::WARN);

        return false;
    }

    visualisations_.emplace(std::make_pair(visualisation->getName(), std::move(visualisation)));

    return true;
}

}  // namespace sackmesser

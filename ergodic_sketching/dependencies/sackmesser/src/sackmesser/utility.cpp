// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <sackmesser/utility.hpp>

namespace sackmesser {
std::vector<std::string> split_string(const std::string& string, const char& delimiter) {
    std::vector<std::string> split;

    std::size_t p1 = 0, p2 = 0;
    while (p2 != std::string::npos) {
        p2 = string.find(delimiter, p1);
        split.push_back(string.substr(static_cast<unsigned>(p1), ((p2 != std::string::npos) ? p2 : string.size()) - static_cast<unsigned>(p1)));
        p1 = p2 + 1;
    }

    return split;
}
}  // namespace sackmesser

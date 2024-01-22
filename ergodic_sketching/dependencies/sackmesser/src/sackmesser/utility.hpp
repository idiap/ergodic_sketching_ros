// SPDX-FileCopyrightText: 2023 Idiap Research Institute <contact@idiap.ch>
//
// SPDX-FileContributor: Tobias Loew <tobias.loew@idiap.ch
//
// SPDX-License-Identifier: GPL-3.0-only

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace sackmesser {
std::vector<std::string> split_string(const std::string& string, const char& delimiter);

template <class Type>
class MultThreadIterate {
public:
    MultThreadIterate(const unsigned& num_threads) : index_(0), num_threads_(num_threads) {}

    void execute(const std::vector<Type>& container, const std::function<void(const Type&)> functor) {
        for (unsigned i = 0; i < num_threads_; ++i) {
            threads_.push_back(std::thread([this, &container, functor]() {
                unsigned index;
                while (getIndex(index, container.size())) {
                    functor(container.at(index));
                }
            }));
        }

        for (std::thread& thread : threads_) {
            thread.join();
        }
    }

private:
    bool getIndex(unsigned& index, const unsigned& max) {
        if (index_ + 1 < max) {
            index = index_++;
            return true;
        } else {
            return false;
        }
    }

private:
    std::atomic<unsigned> index_;

    unsigned num_threads_;

    std::vector<std::thread> threads_;
};
}  // namespace sackmesser

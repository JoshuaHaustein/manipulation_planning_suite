//
// Created by joshua on 8/16/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RANDOM_H
#define MANIPULATION_PLANNING_SUITE_RANDOM_H

#include <ompl/util/RandomNumbers.h>
#include <Eigen/Core>
#include <memory>

namespace ompl {
    typedef std::shared_ptr<::ompl::RNG> RNGPtr;
}

namespace mps {
    namespace planner {
        namespace util {
            namespace random {
                // declare random generator, do not access directly, use getDefaultRandomGenerator instead
                extern ::ompl::RNGPtr random_generator;
                /**
                 * Returns the default random generator. This is a globally shared instance!
                 * @return pointer to random generator
                 */
                ::ompl::RNGPtr getDefaultRandomGenerator();

                /**
                 * Samples a vector of floats uniformly from a bounding box.
                 * @param min_values - minimal values.
                 * @param max_values - maximal values.
                 * @param output - sampled vector
                 * @param indices - vector of indices, all indices, if empty
                 * @param rng - (optional) random generator to use
                 */
                void sampleUniform(const Eigen::VectorXf& min_values, const Eigen::VectorXf& max_values,
                                   Eigen::VectorXf& output, const Eigen::VectorXi& indices=Eigen::VectorXi(),
                                   ::ompl::RNGPtr rng = getDefaultRandomGenerator());
            }
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_RANDOM_H

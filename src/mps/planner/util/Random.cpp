//
// Created by joshua on 8/16/17.
//
#include <mps/planner/util/Random.h>

//using namespace mps::planner::util;

// define random generator
::ompl::RNGPtr mps::planner::util::random::random_generator(nullptr);

::ompl::RNGPtr mps::planner::util::random::getDefaultRandomGenerator() {
    if (!mps::planner::util::random::random_generator) {
        mps::planner::util::random::random_generator = std::make_shared<::ompl::RNG>();
    }
    return mps::planner::util::random::random_generator;
}

void mps::planner::util::random::sampleUniform(const Eigen::VectorXf& min_values, const Eigen::VectorXf& max_values,
                                               Eigen::VectorXf& output, ::ompl::RNGPtr rng) {
    long dim = min_values.size();
    assert(dim == max_values.size());
    output.resize(dim);
    for (long i = 0; i < dim; ++i) {
        output[i] = (float)rng->uniformReal((double)min_values[i], (double)max_values[i]);
    }
}

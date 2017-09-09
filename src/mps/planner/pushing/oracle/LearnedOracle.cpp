//
// Created by joshua on 9/7/17.
//
#include <mps/planner/pushing/oracle/LearnedOracle.h>

mps::planner::pushing::oracle::LearnedPipeOracle::LearnedPipeOracle() {

}

mps::planner::pushing::oracle::LearnedPipeOracle::~LearnedPipeOracle() {

}

void mps::planner::pushing::oracle::LearnedPipeOracle::prepareOracle(const Eigen::VectorXf &current_robot_state,
                                                                     const Eigen::VectorXf &current_obj_state,
                                                                     const Eigen::VectorXf &next_obj_state) {

}

float mps::planner::pushing::oracle::LearnedPipeOracle::predictPushability(const Eigen::VectorXf &current_obj_state,
                                                                           const Eigen::VectorXf &next_obj_state) {
    return 0;
}

float mps::planner::pushing::oracle::LearnedPipeOracle::predictFeasibility(const Eigen::VectorXf &current_robot_state,
                                                                           const Eigen::VectorXf &current_obj_state,
                                                                           const Eigen::VectorXf &next_obj_state) {
    return 0;
}

void mps::planner::pushing::oracle::LearnedPipeOracle::predictAction(const Eigen::VectorXf &current_robot_state,
                                                                     const Eigen::VectorXf &current_obj_state,
                                                                     const Eigen::VectorXf &next_obj_state,
                                                                     Eigen::VectorXf &control) {

}

void mps::planner::pushing::oracle::LearnedPipeOracle::sampleFeasibleState(Eigen::VectorXf &new_robot_state,
                                                                           const Eigen::VectorXf &current_obj_state,
                                                                           const Eigen::VectorXf &next_obj_state) {

}

#include <mps/planner/pushing/oracle/HumanOracle.h>

using namespace mps::planner::pushing::oracle;

HumanOracle::HumanOracle() {

}

HumanOracle::~HumanOracle() {

}

void HumanOracle::prepareOracle(const Eigen::VectorXf &current_robot_state, const Eigen::VectorXf &current_obj_state,
                                const Eigen::VectorXf &next_obj_state) {

}

float HumanOracle::predictPushability(const Eigen::VectorXf &current_obj_state, const Eigen::VectorXf &next_obj_state) {
    return 0;
}

float
HumanOracle::predictFeasibility(const Eigen::VectorXf &current_robot_state, const Eigen::VectorXf &current_obj_state,
                                const Eigen::VectorXf &next_obj_state) {
    return 0;
}

void HumanOracle::predictAction(const Eigen::VectorXf &current_robot_state, const Eigen::VectorXf &current_obj_state,
                                const Eigen::VectorXf &next_obj_state, Eigen::VectorXf &control) {

}

void HumanOracle::sampleFeasibleState(const Eigen::VectorXf &new_robot_state, const Eigen::VectorXf &current_obj_state,
                                      const Eigen::VectorXf &next_obj_state) {

}

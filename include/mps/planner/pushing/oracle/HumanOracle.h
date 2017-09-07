//
// Created by joshua on 9/7/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_HUMANORACLE_H
#define MANIPULATION_PLANNING_SUITE_HUMANORACLE_H

#include <mps/planner/pushing/oracle/Oracle.h>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace oracle {
                class HumanOracle : public PushingOracle {
                public:
                    HumanOracle();
                    ~HumanOracle();

                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_HUMANORACLE_H

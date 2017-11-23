//
// Created by joshua on 9/3/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_TIME_H
#define MANIPULATION_PLANNING_SUITE_TIME_H

#include <chrono>

namespace mps {
    namespace planner {
        namespace util {
            namespace time {
                class Timer {
                public:
                    Timer();
                    ~Timer();
                    void startTimer(float time_out);
                    float stopTimer();
                    bool timeOutExceeded() const;
                private:
                    bool _running;
                    float _time_out;
		    std::chrono::high_resolution_clock::time_point _start_time;
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_TIME_H

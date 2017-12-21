//
// Created by joshua on 9/3/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_TIME_H
#define MANIPULATION_PLANNING_SUITE_TIME_H

#include <ctime>

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
                    void addExternalElapsedTime(float seconds);
                private:
                    bool _running;
                    std::clock_t _start_time;
                    float _time_out;
                    float _external_elapsed_time;
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_TIME_H

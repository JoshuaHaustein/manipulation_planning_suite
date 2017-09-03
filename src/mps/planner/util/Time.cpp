//
// Created by joshua on 9/3/17.
//

#include <mps/planner/util/Time.h>

using namespace mps::planner::util::time;

Timer::Timer() {
    _running = false;
}

Timer::~Timer() = default;

void Timer::startTimer(float time_out) {
    _time_out = time_out;
    _running = true;
    _start_time = std::clock();
}

bool Timer::timeOutExceeded() const {
    auto time_running = (std::clock() - _start_time) / CLOCKS_PER_SEC;
    return time_running > _time_out;
}

float Timer::stopTimer() {
    if (not _running) {
       return 0.0f;
    }
    _running = false;
    return (std::clock() - _start_time) / CLOCKS_PER_SEC;
}

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
    _start_time = std::chrono::high_resolution_clock::now();
}

bool Timer::timeOutExceeded() const {
    auto t_end = std::chrono::high_resolution_clock::now();
    auto time_running = std::chrono::duration<float, std::milli>(t_end - _start_time).count();
    return (time_running / 1000.0) > _time_out;
}

float Timer::stopTimer() {
    if (not _running) {
       return 0.0f;
    }
    _running = false;
    auto t_end = std::chrono::high_resolution_clock::now();
    auto time_running = std::chrono::duration<float, std::milli>(t_end - _start_time).count();
    return time_running;
}

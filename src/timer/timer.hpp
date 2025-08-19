#pragma once
#include <chrono>
#include <iostream>

namespace timer {
    
struct timepoint {
    std::chrono::time_point<std::chrono::high_resolution_clock> time;
    std::string name = "";
    timepoint(std::string name_);
};
struct timer {
    private:
    std::vector<timepoint> timepoints;
    std::string name = "";
    time_unit time_unit_;
    bool active;
    int inactive_counter;
    public:
    enum time_unit {
        SECONDS,
        MILLISECONDS,
        MICROSECONDS,
        NANOSECONDS
    };
    timer(std::string name_, time_unit time_unit__);
    void record(std::string timepoint_name);
    void reset(int interval);
    void print();
};

}
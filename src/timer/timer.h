#pragma once
#include <chrono>
#include <iostream>

namespace timer {
    struct timepoint {
        std::chrono::time_point<std::chrono::high_resolution_clock> time;
        std::string name = "";
        timepoint(std::string name_) {
            time = std::chrono::high_resolution_clock::now();
            name = name_;
        }
    };
    struct timer {
        enum time_unit {
            SECONDS,
            MILLISECONDS,
            MICROSECONDS
        };
        std::vector<timepoint> timepoints;
        std::string name = "";
        time_unit time_unit_;
        timer(std::string name_, time_unit time_unit__) {
            name = name_;
            time_unit_ = time_unit__;
            timepoints.push_back(timepoint("start"));
        }
        void record(std::string timepoint_name) {
            timepoints.push_back(timepoint(timepoint_name));
        }
        void print() {
            std::string time_unit_text = "xx";
            double time_unit_multiplier;

            switch (time_unit_) {
                case SECONDS: {
                    time_unit_text = "s ";
                    time_unit_multiplier = 1;
                    break;
                }
                case MILLISECONDS: {
                    time_unit_text = "ms";
                    time_unit_multiplier = 1e3;
                    break;
                }
                case MICROSECONDS: {
                    time_unit_text = "us";
                    time_unit_multiplier = 1e6;
                    break;
                }
            }

            std::cout << "times for timer " << name << ":\n";
            std::printf("%40s%20s  %20s  \n", "Timepoint Name", "Time since last", "Time since start");
            for (int i = 1; i < timepoints.size(); i++) {

                timepoint& t = timepoints[i];
                timepoint& t_last = timepoints[i - 1];
                timepoint& t_start = timepoints[0];

                std::string timepoint_name = t.name;
                double time_since_last = std::chrono::duration<double>(t.time - t_last.time).count();
                double time_since_start = std::chrono::duration<double>(t.time - t_start.time).count();

                time_since_last *= time_unit_multiplier;
                time_since_start *= time_unit_multiplier;

                std::printf("%40s%19.2f%s%19.2f%s\n", timepoint_name.c_str(), time_since_last, time_unit_text.c_str(), time_since_start, time_unit_text.c_str());

            }
            std::cout << "\n\n\n";
        }
        /*
        ~timer() {
            record("end");
            print();
        }
        */
    };
}
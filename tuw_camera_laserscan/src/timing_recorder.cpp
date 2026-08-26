#include "tuw_camera_laserscan/timing_recorder.hpp"

#include <iomanip>

namespace tuw_camera_laserscan {

void TimingRecorder::start() {
    last_time = Clock::now();
    current = records.begin();
}

void TimingRecorder::push_time(const std::string &name) {
    const auto duration = Clock::now() - last_time;
    if (current == records.end() || current->name != name) {
        current = records.emplace(current, name, 0, Clock::duration::zero());
    }
    current->count++;
    current->avg_time = current->avg_time + (duration - current->avg_time) / current->count;
    ++current;
    last_time = Clock::now();
}

std::ostream &operator<<(std::ostream &os, const TimingRecorder &obj) {
    for (const auto &[name, count, avg_time] : obj.records) {
        std::chrono::duration<double, std::milli> avg_time_ms = avg_time;
        os << std::setw(20) << name << " "
            << std::setw(4) << std::fixed
            << std::setprecision(2) << avg_time_ms.count() << "ms\n";
    }
    return os;
}

} // tuw_camera_laserscan

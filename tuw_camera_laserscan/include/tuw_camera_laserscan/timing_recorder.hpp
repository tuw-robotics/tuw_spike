#ifndef TUW_SPIKE_CAMERA__TIMING_RECORDER_HPP_
#define TUW_SPIKE_CAMERA__TIMING_RECORDER_HPP_

#include <chrono>
#include <list>
#include <ostream>
#include <string>

namespace tuw_camera_laserscan {

class TimingRecorder {
public:
  void start();
  void push_time(const std::string& name);

private:
  using Clock = std::chrono::steady_clock;
  struct TimeRecord {
    std::string name;
    int count;
    Clock::duration avg_time;
  };

  Clock::time_point last_time;
  std::list<TimeRecord> records;
  std::list<TimeRecord>::iterator current;

  friend std::ostream &operator<<(std::ostream &os, const TimingRecorder &obj);
};

} // tuw_camera_laserscan

#endif //TUW_SPIKE_CAMERA__TIMING_RECORDER_HPP_

#ifndef LOGGER_H
#define LOGGER_H

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mission_planner_types.hpp>

class Logger {
 public:
  Logger(const int drone_id) {
    // start time
    initial_time = std::chrono::steady_clock::now();
    // local time
    std::time_t t = std::time(nullptr);
    char str_time[80];
    std::strftime(str_time, sizeof(str_time), "%c", std::localtime(&t));
    std::string string_time(str_time);
    std::cout << string_time << std::endl;
    // file open
    std::string home_path(getenv("HOME"));
    file_.open(home_path + "/" + string_time + "_drone" +
               std::to_string(drone_id));
    file_ << std::fixed << std::setprecision(5);
  }
  ~Logger() { file_.close(); }

  template <typename T>
  void log(const T &obj, const std::string &name = "") {
    auto final_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = final_time - initial_time;
    file_ << elapsed_seconds.count() << " " << name << ": " << obj << std::endl;
  }

  void log(std::vector<state> trajectory, const std::string &name = "") {
    auto final_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = final_time - initial_time;
    file_ << elapsed_seconds.count() << " " << name << ": " << std::endl;
    for (auto &point : trajectory) {
      file_ << point.pos(0) << ", " << point.pos(1) << ", " << point.pos(2)
            << ", " << point.vel(0) << ", " << point.vel(1) << ", "
            << point.vel(2) << ", " << point.acc(0) << ", " << point.acc(1)
            << ", " << point.acc(2) << std::endl;
    }
    file_ << std::endl;
  }
  void log(state state_to_log, const std::string &name = "") {
    auto final_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = final_time - initial_time;
    file_ << elapsed_seconds.count() << " " << name << ": " << std::endl;
    file_ << state_to_log.pos(0) << ", " << state_to_log.pos(1) << ", "
          << state_to_log.pos(2) << ", " << state_to_log.vel(0) << ", "
          << state_to_log.vel(1) << ", " << state_to_log.vel(2) << ", "
          << state_to_log.acc(0) << ", " << state_to_log.acc(1) << ", "
          << state_to_log.acc(2) << std::endl;
    file_ << std::endl;
  }

 private:
  std::ofstream file_;
  std::chrono::time_point<std::chrono::steady_clock> initial_time;
};

#endif
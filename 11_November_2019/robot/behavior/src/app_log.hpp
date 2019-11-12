#ifndef APP_LOG_HPP
#define APP_LOG_HPP

#include <iostream>
#include <sstream>

template <class T>
auto build_log_stream(T t) {
  std::stringstream ss;
  ss << t << std::endl;
  return ss;
}

template <class T, class... Args>
auto build_log_stream(T t, Args... a) {
  std::stringstream ss;
  ss << t;
  ss << build_log_stream(a...).str();
  return ss;
}

template <class... Args>
void ERROR(Args... a) {
  std::cout << "[ERROR] " << build_log_stream(a...).str();
}

template <class... Args>
void INFO(Args... a) {
  std::cout << "[INFO] " << build_log_stream(a...).str();
}

#endif

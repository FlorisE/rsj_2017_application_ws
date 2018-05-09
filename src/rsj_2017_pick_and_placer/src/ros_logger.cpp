#include "logger.h"
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdarg>

void Logger::DEBUG(const char* fmt...) {
  va_list args;
  va_start(args, fmt);
  ROS_DEBUG(fmt, args);
  va_end(args);
}

void Logger::INFO(const char* fmt...) {
  va_list args;
  va_start(args, fmt);
  ROS_DEBUG(fmt, args);
  va_end(args);
}

void Logger::WARN(const char* fmt...) {
  va_list args;
  va_start(args, fmt);
  ROS_DEBUG(fmt, args);
  va_end(args);
}

void Logger::ERROR(const char* fmt...) {
  va_list args;
  va_start(args, fmt);
  ROS_DEBUG(fmt, args);
  va_end(args);
}

void Logger::FATAL(const char* fmt...) {
  va_list args;
  va_start(args, fmt);
  ROS_DEBUG(fmt, args);
  va_end(args);
}

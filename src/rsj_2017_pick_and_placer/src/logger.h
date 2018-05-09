#ifndef RSJ_2017_PICK_AND_PLACER_LOGGER_H
#define RSJ_2017_PICK_AND_PLACER_LOGGER_H

class Logger {
public:
  void DEBUG(const char* fmt...);
  void INFO(const char* fmt...);
  void WARN(const char* fmt...);
  void ERROR(const char* fmt...);
  void FATAL(const char* fmt...);
};

#endif //RSJ_2017_PICK_AND_PLACER_LOGGER_H

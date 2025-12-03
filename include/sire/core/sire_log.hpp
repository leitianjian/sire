#ifndef SIRE_LOG_HPP_
#define SIRE_LOG_HPP_

#include <filesystem>
#include <fstream>
#include <string>

#include <aris/core/log.hpp>

#define SIRE_LOG aris::core::cout()

#define SIRE_DEBUG_LOG \
  SIRE_LOG << std::string(__FILE__) << "_" << std::to_string(__LINE__) << ": "

class LoggerBase {
 public:
  virtual auto log() -> void = 0;
  auto setLogDir(std::filesystem::path& path) { log_dir_path_ = path; }
  auto logDirPath() -> std::filesystem::path { return log_dir_path_; };
  auto setLogFile(std::filesystem::path& path) { log_file_path_ = path; }
  auto logFilePath() -> std::filesystem::path { return log_file_path_; };
  auto logFileStream() -> std::ofstream& { return log_fstream_; }

  LoggerBase() : log_dir_path_() {}

 private:
  std::filesystem::path log_dir_path_;
  std::filesystem::path log_file_path_;
  std::ofstream log_fstream_;
};

#endif
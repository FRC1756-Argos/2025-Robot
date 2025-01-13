/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

/*
  Contains functions / classes useful for logging runtime information
*/

#pragma once

#include <cstdarg>
#include <iostream>
#include <string>

#include "units/base.h"

namespace argos_lib {
  /// @brief Represents a log level of either information, or error
  enum LogLevel { INFO, ERR };

  /// @brief Log to the console in a clean, repeatable manner
  class ArgosLogger {
   public:
    ArgosLogger() = delete;
    explicit ArgosLogger(std::string tag) : m_tag{tag} {}

    void Log(LogLevel level, const char* fmt, ...) const {
      va_list lst;
      va_start(lst, fmt);
      switch (level) {
        case LogLevel::INFO:
          std::fprintf(stdout, "[%s]", m_tag.c_str());
          std::vfprintf(stdout, fmt, lst);
          break;

        case LogLevel::ERR:
          std::fprintf(stderr, "[%s_ERROR]", m_tag.c_str());
          std::vfprintf(stderr, fmt, lst);
          break;

        default:
          std::fprintf(stdout, "[%s]", m_tag.c_str());
          std::vfprintf(stdout, fmt, lst);
          break;
      }
      std::fprintf(stdout, "\n");
    }

   private:
    std::string m_tag;
  };
}  // namespace argos_lib

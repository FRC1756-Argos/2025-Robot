/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <string>

namespace argos_lib {

  class EdgeDetector {
   public:
    enum class EdgeDetectSettings { DETECT_BOTH, DETECT_FALLING, DETECT_RISING };

    enum class edgeStatus { FALLING, RISING, ERROR, NONE };

    explicit EdgeDetector(EdgeDetector::EdgeDetectSettings _settings, bool initialValue = false);

    bool operator()(bool curVal);

    edgeStatus Calculate(bool curVal);

    std::string ToString(edgeStatus status);

   private:
    EdgeDetector::EdgeDetectSettings m_settings;

    bool m_previousValue;

    edgeStatus DetectFalling(bool currentValue);

    edgeStatus DetectRising(bool currentValue);
  };
}  // namespace argos_lib

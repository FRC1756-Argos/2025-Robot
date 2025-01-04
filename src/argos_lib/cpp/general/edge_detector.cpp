/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/general/edge_detector.h"

#include "frc/smartdashboard/SmartDashboard.h"

using argos_lib::EdgeDetector;

EdgeDetector::EdgeDetector(EdgeDetector::EdgeDetectSettings _settings, bool initialValue) {
  m_settings = _settings;
  m_previousValue = initialValue;
}

bool EdgeDetector::operator()(bool curVal) {
  edgeStatus stat = Calculate(curVal);
  switch (m_settings) {
    case EdgeDetectSettings::DETECT_BOTH:
      return (stat == edgeStatus::RISING || stat == edgeStatus::FALLING);
      break;
    case EdgeDetectSettings::DETECT_RISING:
      return (stat == edgeStatus::RISING);
      break;
    case EdgeDetectSettings::DETECT_FALLING:
      return (stat == edgeStatus::FALLING);
      break;
  }
  return false;
}

EdgeDetector::edgeStatus EdgeDetector::Calculate(bool curVal) {
  edgeStatus statusReturn = edgeStatus::NONE;
  switch (m_settings) {
    case EdgeDetectSettings::DETECT_BOTH:
      if (DetectFalling(curVal) == edgeStatus::FALLING) {
        statusReturn = edgeStatus::FALLING;
      } else if (DetectRising(curVal) == edgeStatus::RISING) {
        statusReturn = edgeStatus::RISING;
      }
      break;
    case EdgeDetectSettings::DETECT_FALLING:
      statusReturn = DetectFalling(curVal);
      break;
    case EdgeDetectSettings::DETECT_RISING:
      statusReturn = DetectRising(curVal);
      break;
  }
  m_previousValue = curVal;
  return statusReturn;
}

std::string EdgeDetector::ToString(edgeStatus status) {
  switch (status) {
    case edgeStatus::RISING:
      return "Rising";
      break;
    case edgeStatus::FALLING:
      return "Falling";
      break;
    case edgeStatus::NONE:
      return "None";
      break;
    case edgeStatus::ERROR:
      return "Error";
      break;

    default:
      return "DEFAULT";
      break;
  }
}

EdgeDetector::edgeStatus EdgeDetector::DetectFalling(bool currentValue) {
  if (m_previousValue && !currentValue) {
    return edgeStatus::FALLING;
  } else {
    return edgeStatus::NONE;
  }
}

EdgeDetector::edgeStatus EdgeDetector::DetectRising(bool currentValue) {
  if (!m_previousValue && currentValue) {
    return edgeStatus::RISING;
  } else {
    return edgeStatus::NONE;
  }
}

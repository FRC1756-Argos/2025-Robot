/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <gtest/gtest.h>

#include "argos_lib/general/edge_detector.h"

using argos_lib::EdgeDetector;

TEST(EdgeDetectorTest, NoEdge) {
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, false);
    EXPECT_EQ(detector(false), false);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector(false), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, true);
    EXPECT_EQ(detector(true), false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector(true), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, false);
    EXPECT_EQ(detector(false), false);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector(false), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, true);
    EXPECT_EQ(detector(true), false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector(true), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, false);
    EXPECT_EQ(detector(false), false);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector(false), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, true);
    EXPECT_EQ(detector(true), false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector(true), false);
  }
}

TEST(EdgeDetectorTest, RisingEdge) {
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, false);
    EXPECT_EQ(detector(true), true);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::RISING);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, false);
    EXPECT_EQ(detector(true), true);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::RISING);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, false);
    EXPECT_EQ(detector(true), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::NONE);
  }
}

TEST(EdgeDetectorTest, FallingEdge) {
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, true);
    EXPECT_EQ(detector(false), true);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, true);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::FALLING);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, true);
    EXPECT_EQ(detector(false), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, true);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, true);
    EXPECT_EQ(detector(false), true);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, true);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::FALLING);
  }
}

TEST(EdgeDetectorTest, RiseFall) {
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, false);
    EXPECT_EQ(detector(true), true);
    EXPECT_EQ(detector(false), true);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::RISING);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::FALLING);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, false);
    EXPECT_EQ(detector(true), true);
    EXPECT_EQ(detector(false), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::RISING);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, false);
    EXPECT_EQ(detector(true), false);
    EXPECT_EQ(detector(false), true);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, false);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::FALLING);
  }
}

TEST(EdgeDetectorTest, FallRise) {
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, true);
    EXPECT_EQ(detector(false), true);
    EXPECT_EQ(detector(false), false);
    EXPECT_EQ(detector(true), true);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_BOTH, true);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::FALLING);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::RISING);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, true);
    EXPECT_EQ(detector(false), false);
    EXPECT_EQ(detector(false), false);
    EXPECT_EQ(detector(true), true);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_RISING, true);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::RISING);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, true);
    EXPECT_EQ(detector(false), true);
    EXPECT_EQ(detector(false), false);
    EXPECT_EQ(detector(true), false);
  }
  {
    EdgeDetector detector(EdgeDetector::EdgeDetectSettings::DETECT_FALLING, true);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::FALLING);
    EXPECT_EQ(detector.Calculate(false), EdgeDetector::edgeStatus::NONE);
    EXPECT_EQ(detector.Calculate(true), EdgeDetector::edgeStatus::NONE);
  }
}

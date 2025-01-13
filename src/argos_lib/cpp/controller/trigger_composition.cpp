/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include "argos_lib/controller/trigger_composition.h"

#include <algorithm>
#include <numeric>

namespace {
  frc2::Trigger Or(frc2::Trigger lhs, frc2::Trigger rhs) {
    return lhs || rhs;
  }
  frc2::Trigger And(frc2::Trigger lhs, frc2::Trigger rhs) {
    return lhs && rhs;
  }

  template <class InputIt>
  frc2::Trigger NoneOfIt(InputIt begin, InputIt end) {
    return !std::reduce(std::next(begin), end, *begin, Or);
  }

  template <class InputIt>
  frc2::Trigger AnyOfIt(InputIt begin, InputIt end) {
    return std::reduce(std::next(begin), end, *begin, Or);
  }

  template <class InputIt>
  frc2::Trigger AllOfIt(InputIt begin, InputIt end) {
    return std::reduce(std::next(begin), end, *begin, And);
  }

}  // namespace

frc2::Trigger argos_lib::triggers::OneOf(std::vector<frc2::Trigger> triggers) {
  if (triggers.size() == 1) {
    return triggers.front();
  }
  std::vector<frc2::Trigger> allExclusiveChecks;
  allExclusiveChecks.reserve(triggers.size());

  for (size_t exclusiveTrueIdx = 0; exclusiveTrueIdx < triggers.size(); ++exclusiveTrueIdx) {
    allExclusiveChecks.emplace_back(triggers.at(exclusiveTrueIdx));
    for (size_t otherIdx = 0; otherIdx < triggers.size(); ++otherIdx) {
      if (exclusiveTrueIdx != otherIdx) {
        auto tempTrigger = allExclusiveChecks.back() && !triggers.at(otherIdx);
        std::swap(allExclusiveChecks.back(), tempTrigger);
      }
    }
  }

  return std::reduce(std::next(allExclusiveChecks.begin()), allExclusiveChecks.end(), *allExclusiveChecks.begin(), Or);
}

frc2::Trigger argos_lib::triggers::NoneOf(std::vector<frc2::Trigger> triggers) {
  return NoneOfIt(triggers.begin(), triggers.end());
}

frc2::Trigger argos_lib::triggers::AnyOf(std::vector<frc2::Trigger> triggers) {
  return AnyOfIt(triggers.begin(), triggers.end());
}

frc2::Trigger argos_lib::triggers::AllOf(std::vector<frc2::Trigger> triggers) {
  return AllOfIt(triggers.begin(), triggers.end());
}

/// \copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <type_traits>

/**
 * @brief Helper function generator to detect if a namespace has a member defined
 *
 * @param X Name of member to detect
 */
#define HAS_MEMBER(X)                    \
  template <typename T, typename = void> \
  struct has_##X : std::false_type {};   \
  template <typename T>                  \
  struct has_##X<T, decltype((void)T::X, void())> : std::true_type {};

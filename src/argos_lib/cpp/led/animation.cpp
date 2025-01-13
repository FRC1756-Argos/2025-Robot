/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#include <argos_lib/general/angle_utils.h>
#include <argos_lib/led/animiation.h>
#include <argos_lib/led/geometry.h>
#include <units/angle.h>
#include <units/math.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <optional>

argos_lib::led::Animation argos_lib::led::animation::Pong(unsigned offset,
                                                          unsigned width,
                                                          unsigned height,
                                                          unsigned ballSize,
                                                          bool rainbow,
                                                          frc::AddressableLED::LEDData ballColor,
                                                          frc::AddressableLED::LEDData backgroundColor,
                                                          units::millisecond_t frameTime,
                                                          argos_lib::led::PanelScanParams scanParams) {
  // The animation update function is a mutable lambda so multiple instances of this animation can operate independently
  // and retain state required to bounce around
  return argos_lib::led::Animation{
      std::function<Strip()>([width,
                              height,
                              ballSize,
                              rainbow,
                              ballColor,
                              backgroundColor,
                              frameTime,
                              scanParams,
                              x = width / 2.0,
                              y = height / 2.0,
                              dir = units::degree_t(std::rand() % 360U),
                              lastTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())]() mutable {
        // Start by moving the ball in the previous travel direction the amount required based on the
        // configured speed
        const auto now =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch());

        float travelDist = frameTime < 1_us ? 1.0 : (units::millisecond_t(now - lastTime) / frameTime).to<float>();

        float newX = x + travelDist * units::math::cos(dir);
        float newY = y + travelDist * units::math::sin(dir);

        // Check for the ball moving outside the rectangle as this indicates a bounce is required
        // Multiple bounces can occur in the same update depending on the distance covered since last update
        while (newX < 0 || newY < 0 || newX > (width - 1) || newY > (height - 1)) {
          std::optional<std::pair<float, float>> intersectionPoint = std::nullopt;
          if (!intersectionPoint && newX < 0) {
            intersectionPoint = SegmentIntersection(0, 0, 0, height - 1, x, y, newX, newY);
            if (intersectionPoint) {
              dir = FlipVertical(dir);
            }
          }
          if (!intersectionPoint && newX > width - 1) {
            intersectionPoint = SegmentIntersection(width - 1, 0, width - 1, height - 1, x, y, newX, newY);
            if (intersectionPoint) {
              dir = FlipVertical(dir);
            }
          }
          if (!intersectionPoint && newY < 0) {
            intersectionPoint = SegmentIntersection(0, 0, width - 1, 0, x, y, newX, newY);
            if (intersectionPoint) {
              dir = FlipHorizontal(dir);
            }
          }
          if (!intersectionPoint && newY > height - 1) {
            intersectionPoint = SegmentIntersection(0, height - 1, width - 1, height - 1, x, y, newX, newY);
            if (intersectionPoint) {
              dir = FlipHorizontal(dir);
            }
          }

          if (intersectionPoint) {
            // Choose random color, but always max brightness and saturation
            if (rainbow) {
              ballColor.SetHSV(std::rand() % 180, 255, 255);
            }
            // Update new target position after bounce before doing another bounce check
            travelDist -= std::hypot(intersectionPoint.value().first - x, intersectionPoint.value().second - y);
            x = intersectionPoint.value().first;
            y = intersectionPoint.value().second;
            newX = x + travelDist * units::math::cos(dir);
            newY = y + travelDist * units::math::sin(dir);
          }
        }

        x = newX;
        y = newY;
        lastTime = now;

        // Draw ball on fresh panel and return the new state
        argos_lib::led::Panel panel(width, height, backgroundColor);
        DrawRectangle(panel, ballSize, ballSize, std::round(x), std::round(y), ballColor);
        return argos_lib::led::Serialize(panel, scanParams);
      }),
      width * height,
      offset};
}

argos_lib::led::AnimatedSprite argos_lib::led::animation::ChompingPacMan(float radius,
                                                                         frc::AddressableLED::LEDData color,
                                                                         units::degree_t direction,
                                                                         units::millisecond_t chompPeriod,
                                                                         bool feathered) {
  constexpr units::degree_t maxMouthAngle = 90_deg;
  return argos_lib::led::AnimatedSprite{[radius, color, direction, chompPeriod, feathered, maxMouthAngle]() {
    const auto now =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch());
    const auto timeWithinPeriod = units::math::fmod(units::millisecond_t(now), chompPeriod);
    const auto halfPeriod = chompPeriod / 2.0;
    const auto mouthAngle = maxMouthAngle * units::math::abs(timeWithinPeriod - halfPeriod) / halfPeriod;
    return DrawPacMan(radius, color, direction, mouthAngle, feathered);
  }};
}

argos_lib::led::AnimatedSprite argos_lib::led::animation::DyingPacMan(float radius,
                                                                      frc::AddressableLED::LEDData color,
                                                                      units::degree_t direction,
                                                                      units::degree_t initialMouthAngle,
                                                                      units::millisecond_t animationTime,
                                                                      bool feathered) {
  return argos_lib::led::AnimatedSprite{[radius,
                                         color,
                                         direction,
                                         initialMouthAngle,
                                         animationTime,
                                         animationStartTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                                             std::chrono::steady_clock::now().time_since_epoch()),
                                         feathered]() mutable {
    const auto percentComplete = units::millisecond_t(std::chrono::duration_cast<std::chrono::milliseconds>(
                                                          std::chrono::steady_clock::now().time_since_epoch()) -
                                                      animationStartTime) /
                                 (animationTime * 0.9);
    if (percentComplete.to<double>() > 1.0) {
      return DrawPacMan(radius, color, direction, 360_deg, feathered);
    }
    return DrawPacMan(
        radius, color, direction, initialMouthAngle + (360_deg - initialMouthAngle) * percentComplete, feathered);
  }};
}

argos_lib::led::Animation argos_lib::led::animation::PacManPacing(unsigned offset,
                                                                  unsigned width,
                                                                  unsigned height,
                                                                  bool rainbow,
                                                                  frc::AddressableLED::LEDData pacManColor,
                                                                  argos_lib::led::PrimaryScanDirection paceDirection,
                                                                  units::millisecond_t chompPeriod,
                                                                  units::millisecond_t moveSpeed,
                                                                  argos_lib::led::PanelScanParams scanParams,
                                                                  bool feathered) {
  const unsigned pacManRadius = std::min(width, height) / 2.0;
  const unsigned visibleTravelDist = paceDirection == argos_lib::led::PrimaryScanDirection::Horizontal ? width : height;
  const unsigned travelDist = 4 * pacManRadius + visibleTravelDist;
  const units::millisecond_t dyingTime = travelDist * moveSpeed;
  const unsigned pipRadius = 1;  // std::max(1U, std::min(width, height) / 4);
  const unsigned pipSpacing = pipRadius * 6;
  const unsigned numPips = std::max(1U, visibleTravelDist / pipSpacing);
  const unsigned firstPipOffset = std::round((visibleTravelDist / 2.0) - (((numPips - 1) / 2.0) * pipSpacing));

  return argos_lib::led::Animation{
      std::function<Strip()>([width,
                              height,
                              rainbow,
                              pacManColor,
                              paceDirection,
                              chompPeriod,
                              moveSpeed,
                              scanParams,
                              feathered,
                              pacManRadius,
                              travelDist,
                              dyingTime,
                              pipRadius,
                              pipSpacing,
                              numPips,
                              drawnPips = numPips,
                              firstPipOffset,
                              paceIndex = 0,
                              deathIndex = std::rand() % 5 + 2,
                              dying = false,
                              startTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch()),
                              pacManSprite =
                                  ChompingPacMan(pacManRadius, pacManColor, 0_deg, chompPeriod, false)]() mutable {
        // Canvas must be redrawn every time to prevent possible smearing
        Panel canvas{width, height, {}};

        const frc::AddressableLED::LEDData pipColor{255, 255, 255};

        const auto now =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch());
        const auto cumulativeTime = units::millisecond_t(now - startTime);
        const auto oldPaceIndex = paceIndex;
        paceIndex = units::math::floor(cumulativeTime / dyingTime);
        const auto timeInLap = units::math::fmod(cumulativeTime, dyingTime);
        float paceDist = (timeInLap / moveSpeed).to<float>();

        auto paceOrientation = 0_deg;
        const auto orientationIndex = paceIndex == deathIndex ? paceIndex - 1 : paceIndex;
        {
          if (paceDirection == argos_lib::led::PrimaryScanDirection::Horizontal) {
            if (orientationIndex % 2 == 0) {
              // Right
              paceOrientation = 0_deg;
            } else {
              // Left
              paceOrientation = 180_deg;
            }
          } else {
            if (orientationIndex % 2 == 0) {
              // Up
              paceOrientation = 90_deg;
            } else {
              // Down
              paceOrientation = 270_deg;
            }
          }
        }

        bool wasDying = dying;
        dying =
            paceIndex >= deathIndex || (paceIndex == (deathIndex - 1) && paceDist >= (travelDist - (4 * pacManRadius)));
        if (dying && !wasDying) {
          pacManSprite = DyingPacMan(pacManRadius, pacManColor, paceOrientation, 90_deg, dyingTime, feathered);
        }

        // Draw wall for PacMan to hit
        if (paceIndex >=
            deathIndex - 1) {  // There's an extra pace during death animation.  Don't want wall to move then
          if (paceDirection == argos_lib::led::PrimaryScanDirection::Horizontal) {
            DrawRectangle(canvas,
                          1,
                          canvas.Height(),
                          orientationIndex % 2 == 0 ? canvas.Width() : 0,
                          canvas.Height() / 2.0,
                          {255, 255, 255});
          } else {
            DrawRectangle(canvas,
                          canvas.Width(),
                          1,
                          canvas.Width() / 2.0,
                          orientationIndex % 2 == 0 ? canvas.Height() : 0,
                          {255, 255, 255});
          }
        }

        if (paceIndex > deathIndex) {
          // Animation complete. Reset for next loop
          startTime = now;
          deathIndex = std::rand() % 5 + 2;
          if (rainbow) {
            pacManColor.SetHSV(std::rand() % 180, 255, 255);
          }
          dying = false;
          paceIndex = 1000;  // Set to something that indicates next loop to re-generate sprite animations.
          return Serialize(canvas, scanParams);
        }
        // Extra pace time to finish death animation
        if (paceIndex == deathIndex || dying) {
          // Even indices are right or up, odd are left or down
          if (paceDirection == argos_lib::led::PrimaryScanDirection::Horizontal) {
            RenderSprite(canvas,
                         pacManSprite(),
                         orientationIndex % 2 == 0 ? canvas.Width() - pacManRadius : pacManRadius,
                         canvas.Height() / 2.0);
          } else {
            RenderSprite(canvas,
                         pacManSprite(),
                         canvas.Width() / 2.0,
                         orientationIndex % 2 == 0 ? canvas.Height() - pacManRadius : pacManRadius);
          }
          return Serialize(canvas, scanParams);
        }

        if (oldPaceIndex != paceIndex) {
          // Update animated sprite direction
          pacManSprite = ChompingPacMan(pacManRadius, pacManColor, paceOrientation, chompPeriod, feathered);
        }

        // Draw pips
        auto prevDrawnPips = drawnPips;
        drawnPips = 0;
        for (unsigned pipIndex = 0; pipIndex < numPips; ++pipIndex) {
          unsigned pipOffset = firstPipOffset + pipIndex * pipSpacing;
          if (paceDist - 2 * pacManRadius < pipOffset) {
            // Pip has not been chomped yet
            ++drawnPips;
            if (paceDirection == argos_lib::led::PrimaryScanDirection::Horizontal) {
              if (orientationIndex % 2 == 0) {
                // Right
                DrawCircle(canvas, pipRadius, pipOffset, canvas.Height() / 2.0, pipColor, feathered);
              } else {
                // Left
                DrawCircle(
                    canvas, pipRadius, canvas.Width() - 1 - pipOffset, canvas.Height() / 2.0, pipColor, feathered);
              }
            } else {
              if (orientationIndex % 2 == 0) {
                // Up
                DrawCircle(canvas, pipRadius, canvas.Width() / 2.0, pipOffset, pipColor, feathered);
              } else {
                // Down
                DrawCircle(
                    canvas, pipRadius, canvas.Width() / 2.0, canvas.Height() - 1 - pipOffset, pipColor, feathered);
              }
            }
          }
        }

        if (drawnPips < prevDrawnPips && rainbow) {
          pacManColor.SetHSV(std::rand() % 180, 255, 255);
          pacManSprite = ChompingPacMan(pacManRadius, pacManColor, paceOrientation, chompPeriod, feathered);
        }

        // Finally draw PacMan!
        if (paceDirection == argos_lib::led::PrimaryScanDirection::Horizontal) {
          if (orientationIndex % 2 == 0) {
            // Right
            RenderSprite(canvas, pacManSprite(), paceDist - 2.0 * pacManRadius, canvas.Height() / 2.0);
          } else {
            // Left
            RenderSprite(canvas,
                         pacManSprite(),
                         static_cast<float>(canvas.Width()) - 1.0 - paceDist + 2 * pacManRadius,
                         canvas.Height() / 2.0);
          }
        } else {
          if (orientationIndex % 2 == 0) {
            // Up
            RenderSprite(canvas, pacManSprite(), canvas.Width() / 2.0, paceDist - 2 * pacManRadius);
          } else {
            // Down
            RenderSprite(canvas,
                         pacManSprite(),
                         canvas.Width() / 2.0,
                         static_cast<float>(canvas.Height()) - 1.0 - paceDist + 2.0 * pacManRadius);
          }
        }

        return Serialize(canvas, scanParams);
      }),
      width * height,
      offset};
}

void argos_lib::led::animation::DrawRectangle(
    Panel& dest, unsigned rectWidth, unsigned rectHeight, float x, float y, frc::AddressableLED::LEDData color) {
  // Need to protect against addressing pixels outside the bounds of the panel
  for (unsigned ix = std::max(0.0, x - (rectWidth / 2.0));
       ix < std::min(dest.Width(), static_cast<unsigned>(std::round(x + (rectWidth / 2.0))));
       ++ix) {
    for (unsigned iy = std::max(0.0, y - (rectHeight / 2.0));
         iy < std::min(dest.Height(), static_cast<unsigned>(std::round(y + (rectHeight / 2.0))));
         ++iy) {
      dest.at(ix, iy) = color;
    }
  }
}

void argos_lib::led::animation::DrawCircle(
    Panel& dest, float radius, float x, float y, frc::AddressableLED::LEDData color, bool feathered) {
  const auto circle = DrawCircleSprite(radius, color, feathered);
  RenderSprite(dest, circle, x, y, 1.0);
}

argos_lib::led::Sprite argos_lib::led::animation::DrawCircleSprite(float radius,
                                                                   frc::AddressableLED::LEDData color,
                                                                   bool feathered) {
  const unsigned boundingSquareDim = std::ceil(radius * 2);
  // Just fill everything and we'll make it transparent
  auto colors = argos_lib::led::Panel(boundingSquareDim, boundingSquareDim, color);
  auto alpha = argos_lib::led::Mask(boundingSquareDim, boundingSquareDim, 0.0);

  float center = boundingSquareDim / 2.0;
  float radiusSquared = radius * radius;

  for (unsigned x = 0; x < boundingSquareDim; ++x) {
    for (unsigned y = 0; y < boundingSquareDim; ++y) {
      double dx = x - center;
      double dy = y - center;
      double distanceSquared = dx * dx + dy * dy;

      if (distanceSquared <= radiusSquared) {
        alpha.at(x, y) = 1;
      } else if (feathered) {
        alpha.at(x, y) = std::max(0.0, 1.0 - distanceSquared - radiusSquared);
      }
    }
  }

  return Sprite{colors, alpha};
}

void argos_lib::led::animation::RenderSprite(Panel& dest, const Sprite& sprite, float x, float y, float alpha) {
  const int xAlign = std::round(x - sprite.colors.Width() / 2.0);
  const int yAlign = std::round(y - sprite.colors.Height() / 2.0);

  const unsigned firstSpriteX = std::max(0, -xAlign);
  const unsigned firstSpriteY = std::max(0, -yAlign);

  const unsigned firstPanelX = std::max(0, xAlign);
  const unsigned firstPanelY = std::max(0, yAlign);

  for (unsigned x = 0; x + firstSpriteX < sprite.colors.Width() && x + firstPanelX < dest.Width(); ++x) {
    for (unsigned y = 0; y + firstSpriteY < sprite.colors.Height() && y + firstPanelY < dest.Height(); ++y) {
      frc::AddressableLED::LEDData& destPixel = dest.at(x + firstPanelX, y + firstPanelY);
      const float spritePixelAlpha = sprite.alpha.at(x + firstSpriteX, y + firstSpriteY) * alpha;
      const frc::AddressableLED::LEDData spritePixelColor = sprite.colors.at(x + firstSpriteX, y + firstSpriteY);

      destPixel =
          frc::AddressableLED::LEDData(destPixel.r * (1.0 - spritePixelAlpha) + spritePixelColor.r * spritePixelAlpha,
                                       destPixel.g * (1.0 - spritePixelAlpha) + spritePixelColor.g * spritePixelAlpha,
                                       destPixel.b * (1.0 - spritePixelAlpha) + spritePixelColor.b * spritePixelAlpha);
    }
  }
}

argos_lib::led::Sprite argos_lib::led::animation::DrawPacMan(float radius,
                                                             frc::AddressableLED::LEDData color,
                                                             units::degree_t direction,
                                                             units::degree_t mouthAngle,
                                                             bool feathered) {
  Sprite pacMan = DrawCircleSprite(radius, color, feathered);

  // Closed mouth is a normal circle
  if (mouthAngle <= 3_deg) {
    return pacMan;
  }

  // Dead PacMan :(
  if (mouthAngle >= 360_deg) {
    pacMan.alpha = Mask(pacMan.alpha.Width(), pacMan.alpha.Height(), 0.0);
    return pacMan;
  }

  const auto halfMouthAngle = mouthAngle / 2.0;
  const auto mouthMax = direction + halfMouthAngle;
  const auto mouthMin = direction - halfMouthAngle;

  for (unsigned x = 0; x < pacMan.alpha.Width(); ++x) {
    for (unsigned y = 0; y < pacMan.alpha.Height(); ++y) {
      auto& alpha = pacMan.alpha.at(x, y);
      // Skip pixels that aren't part of closed-mouth PacMan
      if (alpha > 0) {
        units::degree_t normalizedPixelAngle = argos_lib::angle::ConstrainAngle(
            units::radian_t(std::atan2(y - pacMan.alpha.Height() / 2.0, x - pacMan.alpha.Width() / 2.0)),
            direction - 180_deg,
            direction + 180_deg);
        if (normalizedPixelAngle < mouthMax && normalizedPixelAngle > mouthMin) {
          alpha = 0;
        } else if (feathered) {
          // +/- 5_deg fade to soften
          alpha = std::min(1.0f,
                           ((units::math::abs(normalizedPixelAngle - direction) - halfMouthAngle) / 5_deg).to<float>());
        }
      }
    }
  }

  return pacMan;
}

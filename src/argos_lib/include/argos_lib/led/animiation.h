/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <argos_lib/led/panel.h>
#include <frc/AddressableLED.h>
#include <units/angle.h>
#include <units/time.h>

#include <algorithm>
#include <functional>

namespace argos_lib {
  namespace led {
    /**
     * @brief A representation of a custom LED animation.
     */
    struct Animation {
      /**
       * @brief Delete default constructor because it is meaningless
       */
      Animation() = delete;

      /**
       * @brief Construct a new Animation object
       *
       * @param update Function to call that generates the new LED states for this animation
       * @param numLEDs Number of LEDs in the animation
       * @param offset Address of first LED controlled by this animation
       */
      Animation(std::function<Strip()> update, unsigned numLEDs, unsigned offset)
          : update{update}, numLEDs{numLEDs}, offset{offset} {};

      std::function<Strip()> update;  ///< This function can be any callable that returns a sequence of LED colors.
      unsigned numLEDs;               ///< Number of LEDs in the animation
      unsigned offset;                ///< Address of first LED
    };

    /**
     * @brief  https://en.wikipedia.org/wiki/Sprite_(computer_graphics)
     */
    struct Sprite {
      Panel colors;  ///< Colors of the sprite
      Mask alpha;    ///< Per-pixel transparency
    };

    using AnimatedSprite = std::function<Sprite()>;  ///< Callable to get dynamic sprite

    namespace animation {
      /**
       * @brief Animation that kind of works like a DVD player screensaver (https://youtu.be/QOtuX0jL85Y).
       *        A ball bounces around the panel changing color
       *
       * @param offset Address of first LED in controlled panel
       * @param width Width of panel in pixels
       * @param height Height of panel in pixels
       * @param ballSize Width and height of ball in pixels (it's a square... a circle would basically
       *                 be the same at this size and I'm lazy)
       * @param rainbow When true, ball changes to a random hue on every bounce!
       * @param ballColor Initial color of the bouncing ball
       * @param backgroundColor Background color behind the ball
       * @param frameTime Time for ball to travel one pixel distance
       * @param scanParams Parameters controlling the serialization of the internal panel to pixel addresses
       * @return Animation to use to control a panel with this animation pattern
       */
      Animation Pong(unsigned offset,
                     unsigned width,
                     unsigned height,
                     unsigned ballSize,
                     bool rainbow,
                     frc::AddressableLED::LEDData ballColor,
                     frc::AddressableLED::LEDData backgroundColor,
                     units::millisecond_t frameTime,
                     argos_lib::led::PanelScanParams scanParams);

      AnimatedSprite ChompingPacMan(float radius,
                                    frc::AddressableLED::LEDData color,
                                    units::degree_t direction,
                                    units::millisecond_t chompPeriod,
                                    bool feathered = false);
      AnimatedSprite DyingPacMan(float radius,
                                 frc::AddressableLED::LEDData color,
                                 units::degree_t direction,
                                 units::degree_t initialMouthAngle,
                                 units::millisecond_t animationTime,
                                 bool feathered = false);

      Animation PacManPacing(unsigned offset,
                             unsigned width,
                             unsigned height,
                             bool rainbow,
                             frc::AddressableLED::LEDData pacManColor,
                             argos_lib::led::PrimaryScanDirection paceDirection,
                             units::millisecond_t chompPeriod,
                             units::millisecond_t moveSpeed,
                             argos_lib::led::PanelScanParams scanParams,
                             bool feathered = false);

      /**
       * @brief Draws an axis-aligned rectangle to a panel.  All pixels outside the rectangle are unmodified.
       *
       * @param dest Panel to draw to
       * @param rectWidth Width of rectangle in pixels
       * @param rectHeight Height of rectangle in pixels
       * @param x Rectangle center x coordinate
       * @param y Rectangle center y coordinate
       * @param color Color of drawn rectangle
       */
      void DrawRectangle(
          Panel& dest, unsigned rectWidth, unsigned rectHeight, float x, float y, frc::AddressableLED::LEDData color);

      /**
       * @brief Draw a circle to the panel.
       *
       * @param dest Panel to draw to
       * @param float radius
       * @param x Horizontal center
       * @param y Vertical center
       * @param color Fill color
       * @param feathered When true, make border pixels semi-transparent to soften edge
       */
      void DrawCircle(
          Panel& dest, float radius, float x, float y, frc::AddressableLED::LEDData color, bool feathered = false);

      /**
       * @brief Generate a sprite containing a circle with transparent background
       *
       * @param radius Radius in pixels
       * @param color Fill color
       * @param feathered When true, make border pixels semi-transparent to soften edge
       * @return Sprite within square canvas with edge size ceil(2*radius)
       */
      Sprite DrawCircleSprite(float radius, frc::AddressableLED::LEDData color, bool feathered = false);

      /**
       * @brief Draw a sprite centered at a point to a panel.
       *        This will align the sprite to the nearest pixel alignment on the panel
       *
       * @param dest Panel to draw to
       * @param sprite Sprite to draw
       * @param x Horizontal center
       * @param y Vertical center
       * @param alpha Global sprite transparency where 0 is fully transparent and 1 is fully opaque
       */
      void RenderSprite(Panel& dest, const Sprite& sprite, float x, float y, float alpha = 1.0);

      /**
       * @brief Generate PacMan sprite :D
       *
       * @param radius Radius of our fearless hero
       * @param color Why does he *have* to be yellow?
       * @param direction Mouth direction
       * @param mouthAngle Jaw separation angle from mandible to palate
       * @param feathered When true, make border pixels semi-transparent to soften edge
       * @return PacMan
       */
      Sprite DrawPacMan(float radius,
                        frc::AddressableLED::LEDData color,
                        units::degree_t direction,
                        units::degree_t mouthAngle,
                        bool feathered = false);
    }  // namespace animation
  }  // namespace led
}  // namespace argos_lib

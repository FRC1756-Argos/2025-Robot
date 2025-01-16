/// @copyright Copyright (c) Argos FRC Team 1756.
///            Open Source Software; you can modify and/or share it under the terms of
///            the license file in the root directory of this project.

#pragma once

#include <frc/AddressableLED.h>

#include <algorithm>
#include <vector>

namespace argos_lib {
  namespace led {
    /**
     * @brief Array2D representation where origin is at bottom left
     */
    template <typename T>
    class Array2D {
     public:
      /**
       * @brief Disable default constructor because we need dimensions
       */
      Array2D() = delete;
      /**
       * @brief Construct a new Array2D object
       *
       * @param width Number of cells in horizontal axis
       * @param height Number of cells in vertical axis
       * @param fillValue Initial value for all cells
       */
      Array2D(unsigned width, unsigned height, T fillValue = {}) : m_cells(width, std::vector<T>(height, fillValue)) {}

      /**
       * @brief Get mutable reference to cell at a given coordinate
       *
       * @note This can throw an out_of_range exception if a cell is requested outside the bounds
       *       of the array dimensions.
       *
       * @param x Coordinate in x dimesion.  0 is leftmost cell.
       * @param y Coordinate in y dimension.  0 is bottommost cell.
       * @return Cell at specified coordinate
       */
      T& at(unsigned x, unsigned y) { return m_cells.at(x).at(y); }

      /**
       * @brief Const reference to cell at a given coordinate.  Useful to read a value and allowing
       *        the compiler to protect against modifications.
       *
       * @note This can throw an out_of_range exception if a cell is requested outside the bounds
       *       of the array dimensions.
       *
       * @param x Coordinate in x dimesion.  0 is leftmost cell.
       * @param y Coordinate in y dimension.  0 is bottommost cell.
       * @return Cell at specified coordinate
       */
      const T& at(unsigned x, unsigned y) const { return m_cells.at(x).at(y); }

      /**
       * @brief Cells in horizontal dimension
       *
       * @return Array width in cells
       */
      unsigned Width() const noexcept { return m_cells.size(); }

      /**
       * @brief Cells in vertical dimension
       *
       * @return Array height in cells
       */
      unsigned Height() const noexcept {
        try {
          return m_cells.at(0).size();
        } catch (const std::out_of_range&) {
          // Handle empty vector even though this shouldn't ever happen with deleted default constructor
          return 0;
        }
      }

     private:
      std::vector<std::vector<T>>
          m_cells;  ///< 2D grid of cells.  Kept private in case we actually get an ND-Array that would be better than this vector of vectors
    };

    using Panel = Array2D<frc::AddressableLED::LEDData>;  ///< Pixels in a panel
    using Mask = Array2D<float>;  ///< Transparency mask (alpha channel) for a panel.  Range [0,1]
                                  ///  where 0 is fully transparent and 1 is fully opaque.

    /**
     * @brief A strip of LEDs
     */
    using Strip = std::vector<frc::AddressableLED::LEDData>;

    /**
     * @brief When representing a panel as a strip, sequential addresses are in this direction.
     *        Note that the actual scan direction will alternate on every row/column in the secondary
     *        scan direction.  For example, if the primary scan direction is vertical, adjacent columns
     *        will alternate between up and down.
     */
    enum class PrimaryScanDirection { Vertical, Horizontal };

    /**
     * @brief When representing a panel as a strip, which pixel is the first address
     */
    enum class FirstPixelPosition { TopRight, TopLeft, BottomLeft, BottomRight };

    struct PanelScanParams {
      FirstPixelPosition firstPixel;       ///< Location of lowest address pixel
      PrimaryScanDirection scanDirection;  ///< Sequential address direction
    };

    /**
     * @brief Convert a panel to a 1D vector of pixels in addressing order.  This is a helper
     *        function because many panel animations are easier to design in 2D space then convert
     *        back to address order when it comes time to display.
     *
     * @param panel Panel to serialize
     * @param params Parameters describing panel addressing and orientation
     * @return A strip representation of panel with all pixels in address order
     */
    Strip Serialize(const Panel& panel, const PanelScanParams& params);

  }  // namespace led
}  // namespace argos_lib

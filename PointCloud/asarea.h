/*  Copyright 2021 Philippe Even and Phuc Ngo,
      authors of paper:
      Even, P., and Ngo, P., 2021,
      Automatic forest road extraction fromLiDAR data of mountainous areas.
      In the First International Joint Conference of Discrete Geometry
      and Mathematical Morphology (Springer LNCS 12708), pp. 93-106.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef AS_AREA_H
#define AS_AREA_H

#include "pt2i.h"


/** 
 * @class ASArea asarea.h
 * \brief Area composed of rectangles.
 */
class ASArea
{

public:

  /**
   * \brief Creates an area.
   */
  ASArea ();

  /**
   * \brief Deletes the area.
   */
  ~ASArea ();

  /**
   * \brief Returns the area corners (xmin, ymin), (xmax, ymax).
   */
  inline const std::vector<Pt2i> *getCorners () const { return &corners; }

  /**
   * \brief Returns whether a first corner is defined.
   */
  inline bool started () const { return start_p; }

  /**
   * \brief Returns first corner X coordinate.
   */
  inline int startX () const { return start_x; }

  /**
   * \brief Returns first corner Y coordinate.
   */
  inline int startY () const { return start_y; }

  /**
   * \brief Adds a rectangle corner.
   * @param x Point abscissae.
   * @param y Point ordinate.
   */
  void addCorner (int x, int y);

  /**
   * \brief Saves the area in default file.
   * @param xref Leftmost coordinate (in millimeters).
   * @param yref Lower coordinate (in millimeters).
   * @param tomm Pixel to millimeter ratio.
   */
  void save (int64_t xref, int64_t yref, int64_t tomm) const;

  /**
   * \brief Saves the area in given file.
   * @param name Area file name.
   * @param xref Leftmost coordinate (in millimeters).
   * @param yref Lower coordinate (in millimeters).
   * @param tomm Pixel to millimeter ratio.
   */
  void save (std::string name, int64_t xref, int64_t yref, int64_t tomm) const;

  /**
   * \brief Loads the area from given file.
   * @param name File name.
   * @param xref Leftmost coordinate (in millimeters).
   * @param yref Lower coordinate (in millimeters).
   * @param tomm Pixel to millimeter ratio.
   */
  bool load (std::string name, int64_t xref, int64_t yref, int64_t tomm);


private:

  /** Sequence of rectangle corners (xmin, ymin), (xmax, ymax). */
  std::vector<Pt2i> corners;
  /** Selection status (first corner selected or not). */
  bool start_p;
  /** First corner X coordinate. */
  int start_x;
  /** First corner Y coordinate. */
  int start_y;

};
#endif

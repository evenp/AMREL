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

#ifndef AS_TRACK_H
#define AS_TRACK_H

#include "pt2i.h"
#define TRACK_SIZE 6


/** 
 * @class ASTrack astrack.h
 * \brief Delineated track.
 */
class ASTrack
{
public:

  /**
   * \brief Creates a track.
   */
  ASTrack ();

  /**
   * \brief Deletes the track.
   */
  ~ASTrack ();

  /**
   * \brief Adds a point to the track.
   * @param x Point abscissae.
   * @param y Point ordinate.
   */
  void addPoint (int x, int y);

  /**
   * \brief Moves the active point.
   * @param x New point abscissae.
   * @param y New point ordinate.
   */
  void movePoint (int x, int y);

  /**
   * \brief Withdraws the active point from the track.
   */
  void withdrawActivePoint ();

  /**
   * \brief Adds a point between active and next points.
   */
  void addMiddle ();

  /**
   * \brief Unselects active track.
   */
  void unselect ();

  /**
   * \brief Selects a point to activate.
   * Returns whether a point was activated.
   * @param x Selection abscissae.
   * @param y Selection ordinate.
   */
  bool selectPoint (int x, int y);

  /**
   * \brief Selects incident point on the track.
   * @param dir Next point if true, previous one otherwise.
   */
  void selectNext (int dir);

  /**
   * \brief Returns the points of the track.
   */
  std::vector<Pt2i> points ();

  /**
   * \brief Returns the cut points of the track.
   */
  std::vector<Pt2i> cutPoints ();

  /**
   * \brief Returns the active point.
   */
  Pt2i active () const;

  /**
   * \brief Returns whether a point is activated.
   */
  bool isActive () const;

  /**
   * \brief Returns whether cuts edition mode is on.
   */
  inline bool isCutMode () const { return cut_mode; }

  /**
   * \brief Switches cut edition mode on or off.
   */
  void switchCutMode ();

  /**
   * \brief Returns the track length (in meters).
   */
  float length () const;

  /**
   * \brief Increments the estimated width.
   */
  void incWidth (int val);

  /**
   * \brief Returns the estimated width.
   */
  int getWidth () const;

  /**
   * \brief Returns the track file name (or null string if not saved).
   */
  std::string getName () const;

  /**
   * \brief Saves the track points in a file.
   * @param xref Leftmost coordinate (in millimeters).
   * @param yref Lower coordinate (in millimeters).
   * @param tomm Pixel to millimeter ratio.
   */
  void save (int64_t xref, int64_t yref, int64_t tomm) const;

  /**
   * \brief Loads the track points from a file.
   * Returns whether the track file exists.
   * @param name Track file name.
   * @param xref Leftmost coordinate (in millimeters).
   * @param yref Lower coordinate (in millimeters).
   * @param tomm Pixel to millimeter ratio.
   */
  bool load (std::string name, int64_t xref, int64_t yref, int64_t tomm);


private:

  /** Default width. */
  static const int DEFAULT_WIDTH;

  /** Sequence of track points. */
  std::vector<Pt2i> pts;
  /** Active point index. */
  int cur;
  /** Cut points edition mode */
  bool cut_mode;
  /** Set of cut points. */
  std::vector<Pt2i> cuts;
  /** Active cut point index. */
  int ccur;
  /** Estimated width. */
  int ewidth;
  /** File name (for loaded tracks). */
  std::string fname;

};
#endif

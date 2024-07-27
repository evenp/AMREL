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

#ifndef AMREL_MAP_H
#define AMREL_MAP_H

#include "terrainmap.h"
#include "amrelconfig.h"


/** 
 * @class AmrelMap amrelmap.h
 * \brief Map of detected roads geometry.
 */
class AmrelMap
{
public:

  /**
   * \brief Creates a map of detected roads.
   * @param w Map width.
   * @param h Map height.
   * @param config Pointer to current tool configuration.
   */
  AmrelMap (int w, int h, AmrelConfig *config);

  /**
   * \brief Deletes the map.
   */
  ~AmrelMap ();

  /**
   * \brief Returns the map width.
   */
  inline int width () const { return mw; }

  /**
   * \brief Returns the map height.
   */
  inline int height () const { return mh; }

  /**
   * \brief Returns the number of roads in the map.
   */
  inline int numberOfRoads () const { return nbroads; }

  /**
   * \brief Returns the map array.
   */
  inline unsigned short *getMap () const { return track_map; }

  /**
   * \brief Checks a map pixel occupancy.
   * @param pix Pixel in the map.
   */
  inline bool occupied (const Pt2i &pix) const {
    return (track_map[(mh - 1 - pix.y ()) * mw + pix.x ()]
            != (unsigned short) 0); }

  /**
   * \brief Adds a detected road to the map.
   * Returns whether adding succeeded.
   * @param pts Set of plateau points of the detected road.
   */
  bool add (std::vector<std::vector<Pt2i> > &pts, bool verb = false);

  /**
   * \brief Sets displayed seeds.
   * Placeholder for AMRELnet. Just clears the vector here.
   * @param seeds Pointer to a vector of seeds to display.
   */
  inline void setDisplayedSeeds (std::vector<Pt2i> *seeds) { seeds->clear (); }


private:

  /** Map of detected roads. */
  unsigned short *track_map;
  /** Map width. */
  int mw;
  /** Map height. */
  int mh;
  /** Number of displayed roads. */
  unsigned short nbroads;
  /** Tool configuration. */
  AmrelConfig *cfg;

};
#endif

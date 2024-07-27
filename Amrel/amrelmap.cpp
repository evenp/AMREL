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

#include "amrelmap.h"


AmrelMap::AmrelMap (int w, int h, AmrelConfig *config)
{
  mw = w;
  mh = h;
  cfg = config;
  nbroads = (unsigned short) 0;
  track_map = new unsigned short[mw * mh];
  for (int i = 0; i < mh * mw; i++) track_map[i] = (unsigned short) 0;
}


AmrelMap::~AmrelMap ()
{
  if (track_map != NULL) delete [] track_map;
}


bool AmrelMap::add (std::vector<std::vector<Pt2i> > &pts, bool verbose)
{
  (void) verbose;
  ++ nbroads;
  for (std::vector<std::vector<Pt2i> >::iterator lit = pts.begin ();
       lit != pts.end (); lit ++)
    for (std::vector<Pt2i>::iterator pit = lit->begin ();
         pit != lit->end (); pit++)
      track_map[(mh - 1 - pit->y ()) * mw + pit->x ()] = nbroads;
  return true;
}

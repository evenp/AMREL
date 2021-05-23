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

#include <iostream>
#include <fstream>
#include <inttypes.h>
#include "asarea.h"


ASArea::ASArea ()
{
  start_p = false;
}


ASArea::~ASArea ()
{
}


void ASArea::addCorner (int x, int y)
{
  if (start_p)
  {
    if (start_x != x && start_y != y)
    {
      corners.push_back (Pt2i (x < start_x ? x : start_x,
                               y < start_y ? y : start_y));
      corners.push_back (Pt2i (x < start_x ? start_x : x,
                               y < start_y ? start_y : y));
    }
  }
  else
  {
    start_x = x;
    start_y = y;
  }
  start_p = ! start_p;
}


void ASArea::save (int64_t xref, int64_t yref, int64_t tomm) const
{
  if (! corners.empty ())
  {
    std::ofstream outf ("tracks/area.txt", std::ios::out);
    std::vector<Pt2i>::const_iterator it = corners.begin ();
    while (it != corners.end ())
    {
      outf << xref + (it->x () * tomm) << " "
           << yref + (it->y () * tomm) << std::endl;
      it ++;
      outf << xref + (it->x () * tomm) << " "
           << yref + (it->y () * tomm) << std::endl;
      it ++;
    }
    outf.close ();
  }
}


bool ASArea::load (std::string name, int64_t xref, int64_t yref, int64_t tomm)
{
  std::ifstream input (name.c_str (), std::ios::in);
  if (! input) return false;

  int64_t x, y;
  Pt2i c1;
  input >> x;
  while (! input.eof ())
  {
    input >> y;
    if (start_p)
    {
      corners.push_back (c1);
      corners.push_back (Pt2i ((int) ((x - xref) / tomm),
                               (int) ((y - yref) / tomm)));
    }
    else c1.set ((int) ((x - xref) / tomm), (int) ((y - yref) / tomm));
    start_p = ! start_p;
    input >> x;
  }
  input.close ();
  start_p = false;
  return true;
}

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

#include "chvertex.h"


CHVertex::CHVertex () : Pt2i ()
{
  lv = NULL;
  rv = NULL;
}


CHVertex::CHVertex (int x, int y) : Pt2i (x, y)
{
  lv = NULL;
  rv = NULL;
}


CHVertex::CHVertex (const Pt2i &p) : Pt2i (p)
{
  lv = NULL;
  rv = NULL;
}


CHVertex::~CHVertex ()
{
}


/*
ostream& operator<< (ostream &os, const CHVertex &v)
{
  os << "(" << v.xp << ", " << v.yp << ")";
  return os;
}
*/

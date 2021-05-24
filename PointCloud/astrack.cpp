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
#include <cmath>
#include "astrack.h"

const int ASTrack::DEFAULT_WIDTH = 6;


ASTrack::ASTrack ()
{
  cur = -1;
  cut_mode = false;
  ccur = -1;
  ewidth = DEFAULT_WIDTH;
  fname = std::string ("");
}


ASTrack::~ASTrack ()
{
}


void ASTrack::addPoint (int x, int y)
{
  if (cut_mode)
  {
    cuts.push_back (Pt2i (x, y));
    ccur = (int) (cuts.size ()) - 1;
  }
  else
  {
    pts.push_back (Pt2i (x, y));
    cur = (int) (pts.size ()) - 1;
  }
}


 
void ASTrack::movePoint (int x, int y)
{
  if (cut_mode)
  {
    if (ccur != -1) cuts[ccur].set (x, y);
  }
  else if (cur != -1) pts[cur].set (x, y);
}


void ASTrack::withdrawActivePoint ()
{
  if (cut_mode)
  {
    int nb = (int) (cuts.size ());
    if (ccur >= 0 && ccur < nb)
    {
      std::vector<Pt2i> tmpv;
      std::vector<Pt2i>::iterator it = cuts.begin ();
      while (it != cuts.end ()) tmpv.push_back (*it++);
      cuts.clear ();
      it = tmpv.begin ();
      for (int i = 0; i < ccur; i ++) cuts.push_back (*it++);
      it++;
      for (int i = ccur + 1; i < nb; i ++) cuts.push_back (*it++);
      ccur = -1;
    }
  }
  else
  {
    int nb = (int) (pts.size ());
    if (cur >= 0 && cur < nb)
    {
      std::vector<Pt2i> tmpv;
      std::vector<Pt2i>::iterator it = pts.begin ();
      while (it != pts.end ()) tmpv.push_back (*it++);
      pts.clear ();
      it = tmpv.begin ();
      for (int i = 0; i < cur; i ++) pts.push_back (*it++);
      it++;
      for (int i = cur + 1; i < nb; i ++) pts.push_back (*it++);
      cur --;
    }
  }
}


void ASTrack::addMiddle ()
{
  if (! cut_mode)
  {
    int nb = (int) (pts.size ());
    if (cur >= 0 && cur < nb - 1)
    {
      std::vector<Pt2i> tmpv;
      std::vector<Pt2i>::iterator it = pts.begin ();
      while (it != pts.end ()) tmpv.push_back (*it++);
      pts.clear ();
      it = tmpv.begin ();
      for (int i = 0; i < cur + 1; i ++) pts.push_back (*it++);
      pts.push_back (Pt2i ((it->x () + tmpv[cur].x ()) / 2,
                           (it->y () + tmpv[cur].y ()) / 2));
      for (int i = cur + 1; i < nb; i ++) pts.push_back (*it++);
      cur ++;
    }
  }
}


void ASTrack::unselect ()
{
  cur = -1;
}


bool ASTrack::selectPoint (int x, int y)
{
  int num = 0;
  if (cut_mode)
  {
    ccur = -1;
    std::vector<Pt2i>::iterator it = cuts.begin ();
    while (ccur == -1 && it != cuts.end ())
    {
      if ((it->x () - x) * (it->x () - x) + (it->y () - y) * (it->y () - y) < 9)
        ccur = num;
      else num ++;
      it ++;
    }
    return (ccur != -1);
  }
  cur = -1;
  std::vector<Pt2i>::iterator it = pts.begin ();
  while (cur == -1 && it != pts.end ())
  {
    if ((it->x () - x) * (it->x () - x) + (it->y () - y) * (it->y () - y) < 9)
      cur = num;
    else num ++;
    it ++;
  }
  return (cur != -1);
}


void ASTrack::selectNext (int dir)
{
  if (cur >= 0 && cur < ((int) (pts.size ())) - 1) cur += dir;
}


std::vector<Pt2i> ASTrack::points ()
{
  return pts;
}


std::vector<Pt2i> ASTrack::cutPoints ()
{
  return cuts;
}


bool ASTrack::isActive () const
{
  return (cut_mode ? ccur != -1 : cur != -1);
}


void ASTrack::switchCutMode ()
{
  cut_mode = ! cut_mode;
  cur = -1;
  ccur = -1;
  if (cut_mode)
  {
    if (cuts.empty () && ! pts.empty ())
    {
      Pt2i top (pts[0]);
      Pt2i next (pts[1]);
      int dx = next.x () - top.x ();
      int dy = next.y () - top.y ();
      if (dy * dy > dx * dx)
      {
        cuts.push_back (Pt2i (top.x (),
                              (dy < 0 ? top.y () + 1 : top.y () - 1)));
        cuts.push_back (Pt2i (top.x () - TRACK_SIZE,
                              (dy < 0 ? top.y () + 1 : top.y () - 1)));
        cuts.push_back (Pt2i (top.x () + TRACK_SIZE,
                              (dy < 0 ? top.y () + 1 : top.y () - 1)));
      }
      else
      {
        cuts.push_back (Pt2i ((dx < 0 ? top.x () + 1 : top.x () - 1),
                              top.y ()));
        cuts.push_back (Pt2i ((dx < 0 ? top.x () + 1 : top.x () - 1),
                              top.y () - TRACK_SIZE));
        cuts.push_back (Pt2i ((dx < 0 ? top.x () + 1 : top.x () - 1),
                              top.y () + TRACK_SIZE));
      }
      top.set (pts[((int) (pts.size ())) - 1]);
      next.set (pts[((int) (pts.size ())) - 2]);
      dx = next.x () - top.x ();
      dy = next.y () - top.y ();
      if (dy * dy > dx * dx)
      {
        cuts.push_back (Pt2i (top.x (),
                              (dy < 0 ? top.y () + 1 : top.y () - 1)));
        cuts.push_back (Pt2i (top.x () - TRACK_SIZE,
                              (dy < 0 ? top.y () + 1 : top.y () - 1)));
        cuts.push_back (Pt2i (top.x () + TRACK_SIZE,
                              (dy < 0 ? top.y () + 1 : top.y () - 1)));
      }
      else
      {
        cuts.push_back (Pt2i ((dx < 0 ? top.x () + 1 : top.x () - 1),
                              top.y ()));
        cuts.push_back (Pt2i ((dx < 0 ? top.x () + 1 : top.x () - 1),
                              top.y () - TRACK_SIZE));
        cuts.push_back (Pt2i ((dx < 0 ? top.x () + 1 : top.x () - 1),
                              top.y () + TRACK_SIZE));
      }
    }
  }
}


Pt2i ASTrack::active () const
{
  if (cut_mode)
  {
    if (ccur != -1) return cuts[ccur];
    return Pt2i (0, 0);
  }
  if (cur != -1) return pts[cur];
  return Pt2i (0, 0);
}


float ASTrack::length () const
{
  float tl = 0.0f;
  std::vector<Pt2i>::const_iterator it = pts.begin ();
  Pt2i pt = *it;
  while (it != pts.end ())
  {
    tl += (float) sqrt ((it->x () - pt.x ()) * (it->x () - pt.x ())
                        + (it->y () - pt.y ()) * (it->y () - pt.y ()));
    pt.set (it->x (), it->y ());
    it ++;
  }
  return (tl * 0.5f);
}


void ASTrack::incWidth (int val)
{
  ewidth += val;
  if (val < 2) val = 2;
}


int ASTrack::getWidth () const
{
  return (ewidth);
}


std::string ASTrack::getName () const
{
  return (fname);
}


void ASTrack::save (int64_t xref, int64_t yref, int64_t tomm) const
{
  save ("tracks/last.txt", xref, yref, tomm);
}


void ASTrack::save (std::string name,
                    int64_t xref, int64_t yref, int64_t tomm) const
{
  if (! pts.empty ())
  {
    std::ofstream outf (name.c_str (), std::ios::out);
    outf << "0 " << ewidth << std::endl;
    outf << (int) (pts.size ()) << std::endl;
    std::vector<Pt2i>::const_iterator it = pts.begin ();
    while (it != pts.end ())
    {
      outf << xref + (it->x () * tomm) << " "
           << yref + (it->y () * tomm) << std::endl;
      it ++;
    }
    outf << (int) (cuts.size ()) << std::endl;
    it = cuts.begin ();
    while (it != cuts.end ())
    {
      outf << xref + (it->x () * tomm) << " "
           << yref + (it->y () * tomm) << std::endl;
      it ++;
    }
    outf.close ();
  }
}


bool ASTrack::load (std::string name, int64_t xref, int64_t yref, int64_t tomm)
{
  int64_t x, y;
  int w = 0, nb = 0;
  std::ifstream input (name.c_str (), std::ios::in);
  if (input)
  {
    pts.clear ();
    cur = -1;
    input >> nb;
    if (nb == 0)
    {
      input >> w;
      ewidth = w;
      input >> nb;
    }
    while (nb --)
    {
      input >> x;
      input >> y;
      pts.push_back (Pt2i ((int) ((x - xref) / tomm),
                           (int) ((y - yref) / tomm)));
    }
    cuts.clear ();
    ccur = -1;
    input >> nb;
    while (nb --)
    {
      input >> x;
      input >> y;
      cuts.push_back (Pt2i ((int) ((x - xref) / tomm),
                            (int) ((y - yref) / tomm)));
    }
    input.close ();
    fname = name;
    return true;
  }
  return false;
}

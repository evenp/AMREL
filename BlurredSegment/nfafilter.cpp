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

#include <cmath>
#include "nfafilter.h"

const double NFAFilter::NFA_EPSILON = 1.0;
const double NFAFilter::DEFAULT_LRATIO = 1.0;
const int NFAFilter::DEFAULT_MIN_SECTION_LENGTH = 3;


NFAFilter::NFAFilter ()
{
  min_section_length = DEFAULT_MIN_SECTION_LENGTH;
  max_grad2 = 0;
  gradient_map = NULL;
  cum_histo = NULL;
  bs_section_count = 0;
  lratio = DEFAULT_LRATIO;
}


NFAFilter::~NFAFilter ()
{
  delete [] cum_histo;
}


void NFAFilter::init (VMap *gmap)
{
  // Stores the reference to the gradient map
  gradient_map = gmap;
  int width = gradient_map->getWidth ();
  int height = gradient_map->getHeight ();

  // Counts the number of non-gradient pixels
  int m = 0;
  max_grad2 = 0;
  for (int j = 0; j < height; j++)
  {
    for (int i = 0; i < width; i++)
    {
      int gradval = gradient_map->sqNorm (i, j);
      if (gradval != 0)
      {
        m++;
        if (gradval > max_grad2) max_grad2 = gradval;
      }
    }
  }

  // Rectification 
  m = (width -2) * (height - 2);

  // Gets gradient histogram
  int gmax = (int) (sqrt (max_grad2));
  cum_histo = new double[gmax + 1];
  for (int j = 0; j < height; j++)
    for (int i = 0; i < width; i++)
      cum_histo[(int) (sqrt (gradient_map->sqNorm (i, j)))] ++;

  // Gets cumulated histogram
  for (int i = gmax; i > 0; i--)
    cum_histo[i-1] += cum_histo[i];

  // Normalization
  for (int i = 0; i <= gmax; i++)
    cum_histo[i] /= m;
}


double NFAFilter::nfaValue (double proba, int length)
{
  length = (int) (length / lratio); // Magic number : divForTestSeg
  double nfa = bs_section_count;
  for (int i = 0; i < length && nfa > NFA_EPSILON; i++) nfa *= proba;
  return nfa;
}


bool NFAFilter::filter (const BlurredSegment *bs, int start, int end)
{
  int length = end - start;
  if (length < min_section_length) return false;

  // Gets point with small gradient
  int gmin = max_grad2;
  int pmin = -1;
  std::vector<Pt2i> pts = bs->getAllPoints ();
  for (int i = start; i < end; i++)
  {
    int gn = (gradient_map->getValue (pts[i])).norm2 ();
    if (gn < gmin)
    {
      gmin = gn;
      pmin = i;
    }
  }

  // Gets NFA and accept or split the segment
  double nfa = nfaValue (cum_histo[(int) (sqrt (gmin))], length);
  if (nfa < NFA_EPSILON) return true;
  return (filter (bs, start, pmin) && filter (bs, pmin + 1, end));
}


void NFAFilter::filter (const std::vector<BlurredSegment *> &bss,
                        std::vector<BlurredSegment *> &vsegs,
                        std::vector<BlurredSegment *> &rsegs)
{
  vsegs.clear ();
  rsegs.clear ();

  // Computes Np
  bs_section_count = 0;
  std::vector<BlurredSegment *>::const_iterator it = bss.begin ();
  while (it != bss.end ())
  {
    int length = (*it)->size ();
    bs_section_count += length * (length - 1) / 2;
    it ++;
  }

  // Computes and test each segment NFA
  it = bss.begin ();
  while (it != bss.end ())
  {
    if (filter (*it, 0, (*it)->size ())) vsegs.push_back (*it);
    else rsegs.push_back (*it);
    it ++;
  }
}


void NFAFilter::incLengthRatio (int inc)
{
  lratio += inc * 0.05;
  if (lratio < 1.0) lratio = 1.0;
  else if (lratio > 3.0) lratio = 3.0;
}

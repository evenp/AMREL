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
#include <chrono>
#include "amreltimer.h"

#define PERF_FILE "perf.txt"


const int AmrelTimer::NO_TEST = 0;
const int AmrelTimer::FULL = 1;
const int AmrelTimer::FULL_WITHOUT_LOAD = 2;
const int AmrelTimer::ONLY_LOAD = 3;
const int AmrelTimer::BY_STEP = 4;


AmrelTimer::AmrelTimer (AmrelTool *amreltool)
{
  amrel = amreltool;
  test_type = NO_TEST;
  test_count = 1;
}


AmrelTimer::~AmrelTimer ()
{
}

void AmrelTimer::run ()
{
  if (! amrel->config()->setTiles ()) return;
  bool verb = amrel->config()->isVerboseOn ();
  amrel->config()->setVerbose (false);
  if (test_type == FULL) performanceTest (true);
  else if (test_type == FULL_WITHOUT_LOAD) performanceTest (false);
  else if (test_type == ONLY_LOAD) tileLoadPerf ();
  else if (test_type == BY_STEP)
  {
    if (amrel->config()->step () == AmrelConfig::STEP_ALL)
      allStepsTest ();
    else if (amrel->config()->step () == AmrelConfig::STEP_SAWING)
      sawingTest ();
    else if (amrel->config()->step () == AmrelConfig::STEP_SHADE)
      shadingTest ();
    else if (amrel->config()->step () == AmrelConfig::STEP_RORPO)
      rorpoTest ();
    else if (amrel->config()->step () == AmrelConfig::STEP_SOBEL)
      sobelTest ();
    else if (amrel->config()->step () == AmrelConfig::STEP_FBSD)
      fbsdTest ();
    else if (amrel->config()->step () == AmrelConfig::STEP_SEEDS)
      seedsTest ();
    else if (amrel->config()->step () == AmrelConfig::STEP_ASD)
      asdTest ();
  }
  if (verb) amrel->config()->setVerbose (true);
}

void AmrelTimer::tileLoadPerf ()
{
  bool ok = true;
  std::cout << "Time perf for tile loading..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
      = std::chrono::high_resolution_clock::now ();
  for (int i = 0; i < test_count; i++)
  {
    if (amrel->loadTileSet (true, true)) amrel->clear ();
    else
    {
      std::cout << "Run " << (i + 1) << " : load failed" << std::endl;
      ok = false;
    }
  }
  if (ok)
  {
    std::chrono::high_resolution_clock::time_point end
      = std::chrono::high_resolution_clock::now ();
    std::chrono::duration<double> time_span
      = std::chrono::duration_cast<std::chrono::duration<double>> (end - start);
    std::cout << "Tile load: timing for 1 run = "
              << time_span.count () << " s" << std::endl;
  }
}


void AmrelTimer::performanceTest (bool with_load)
{
  if (! with_load)
  {
    amrel->clear ();
    if (! amrel->loadTileSet (true, true)) return;
    amrel->addTrackDetector ();
  }

  double m_rorpo = 0.0;
  double m_fbsd = 0.0;
  double m_asd = 0.0;
  double m_amrel = 0.0;

  for (int i = 0; i < test_count; i ++)
  {
    std::cout << "\nTIME IN" << std::endl;
    std::chrono::high_resolution_clock::time_point start
         = std::chrono::high_resolution_clock::now ();

    // Shading step
    if (with_load && ! amrel->isDtmLoaded ())
    {
      // DTM loading if not done once for all
      if (! amrel->loadTileSet (true, false))
      {
        std::cout << "Run " << (i + 1) << " : Dtm loading failed" << std::endl;
        return;
      }
    }
    amrel->processShading ();
    if (with_load) amrel->clearDtm ();
    std::chrono::high_resolution_clock::time_point t0
         = std::chrono::high_resolution_clock::now ();

    // Rorpo step
    amrel->processRorpo ();
    amrel->clearShading ();
    std::chrono::high_resolution_clock::time_point t1
         = std::chrono::high_resolution_clock::now ();
    std::chrono::duration<double> time_span
         = std::chrono::duration_cast<std::chrono::duration<double>> (
             t1 - t0);
    m_rorpo += time_span.count ();
    std::cout << "Rorpo: " << time_span.count () << " s" << std::endl;

    // FBSD step
    amrel->processSobel (amrel->vmWidth (), amrel->vmHeight ());
    amrel->clearRorpo ();
    amrel->processFbsd ();
    amrel->clearSobel ();
    amrel->processSeeds ();
    amrel->clearFbsd ();
    std::chrono::high_resolution_clock::time_point t2
         = std::chrono::high_resolution_clock::now ();
    time_span
         = std::chrono::duration_cast<std::chrono::duration<double>> (t2 - t1);
    m_fbsd += time_span.count ();
    std::cout << "Fbsd: " << time_span.count () << " s" << std::endl;

    // Tracks detection
    if (with_load)
    {
      // Raw points loading if not done once at all
      if (! amrel->loadPoints ())
      {
        std::cout << "Run " << (i + 1)
                  << " : Point loading failed" << std::endl;
        return;
      }
      amrel->processAsd ();
      amrel->clearSeeds ();
      amrel->clearAsd ();
      amrel->clearPoints ();
    }
    else amrel->processAsd ();
    std::chrono::high_resolution_clock::time_point t3
      = std::chrono::high_resolution_clock::now ();
    time_span
      = std::chrono::duration_cast<std::chrono::duration<double>> (t3 - t2);
    m_asd += time_span.count ();
    std::cout << "Asd: " << time_span.count () << " s" << std::endl;
    time_span
      = std::chrono::duration_cast<std::chrono::duration<double>> (t3 - start);
    m_amrel += time_span.count ();
    std::cout << "Amrel: " << time_span.count () << " s" << std::endl;
  }

  std::ofstream output (PERF_FILE, std::ios::out);
  output << "rorpo: " << m_rorpo / test_count << " s ("
         << 100 * m_rorpo / m_amrel << " %) " << std::endl;
  output << "fbsd: " << m_fbsd / test_count << " s ("
         << 100 * m_fbsd / m_amrel << " %) " << std::endl;
  output << "asd: " << m_asd / test_count << " s ("
         << 100 * m_asd / m_amrel << " %) " << std::endl;
  output << "amrel: " << m_amrel / test_count << " s" << std::endl;
  output.close ();
}


void AmrelTimer::allStepsTest ()
{
  std::cout << "Time perf for AMREL..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
        = std::chrono::high_resolution_clock::now ();
  for (int i = 0; i < test_count; i++)
  {
    bool ok = amrel->processSawing ();
    if (ok) ok = amrel->processAsd ();
    if (ok) amrel->saveAsdImage ();
    else
    {
      std::cout << "Run " << (i + 1) << " : process failed" << std::endl;
      return;
    }
    amrel->clearSeeds ();
    amrel->clearAsd ();
    amrel->clear ();
  }
  std::chrono::high_resolution_clock::time_point end
        = std::chrono::high_resolution_clock::now ();
  std::chrono::duration<double> time_span
        = std::chrono::duration_cast<std::chrono::duration<double>> (
            end - start);
  std::cout << "AMREL: timing for " << test_count << " run = "
       << time_span.count () << " s" << std::endl;
}


void AmrelTimer::sawingTest ()
{
  std::cout << "Time perf for sawing..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
        = std::chrono::high_resolution_clock::now ();
  for (int i = 0; i < test_count; i++)
    amrel->processSawing ();
  std::chrono::high_resolution_clock::time_point end
        = std::chrono::high_resolution_clock::now ();
  std::chrono::duration<double> time_span
        = std::chrono::duration_cast<std::chrono::duration<double>> (
            end - start);
  std::cout << "Sawing: timing for " << test_count << " run = "
       << time_span.count () << " s" << std::endl;
  if (! amrel->saveSeeds ())
    std::cout << "Sawing : seeds saving failed" << std::endl;
  else if (amrel->config()->isOutMapOn ()) amrel->saveSeedsImage ();
}


void AmrelTimer::shadingTest ()
{
  // Tile files loading (only DTM and raw point file headers)
  if (! amrel->loadTileSet (true, false))
  {
    std::cout << "Shading : seeds loading failed" << std::endl;
    return;
  }
  std::cout << "Time perf for shading..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
        = std::chrono::high_resolution_clock::now ();
  for (int i = 0; i < test_count; i++)
    amrel->processShading ();
  std::chrono::high_resolution_clock::time_point end
        = std::chrono::high_resolution_clock::now ();
  std::chrono::duration<double> time_span
        = std::chrono::duration_cast<std::chrono::duration<double>> (
            end - start);
  std::cout << "Shading: timing for " << test_count << " run = "
            << time_span.count () << " s" << std::endl;
  if (! amrel->saveShadingMap ())
    std::cout << "Shading : map saving failed" << std::endl;
  else if (amrel->config()->isOutMapOn ()) amrel->saveShadingImage ();
}


void AmrelTimer::rorpoTest ()
{
  if (! amrel->loadShadingMap ())
  {
    std::cout << "Rorpo : shading map loading failed" << std::endl;
    return;
  }
  std::cout << "Time perf for Rorpo..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
        = std::chrono::high_resolution_clock::now ();
  for (int i = 0; i < test_count; i++)
    amrel->processRorpo ();
  std::chrono::high_resolution_clock::time_point end
        = std::chrono::high_resolution_clock::now ();
  std::chrono::duration<double> time_span
        = std::chrono::duration_cast<std::chrono::duration<double>> (
            end - start);
  std::cout << "Rorpo: timing for " << test_count << " run = "
            << time_span.count () << " s" << std::endl;
  if (! amrel->saveRorpoMap ())
    std::cout << "Rorpo : map saving failed" << std::endl;
  else
  {
    if (amrel->config()->isOutMapOn ()) amrel->saveRorpoImage ();
    amrel->clearShading ();
  }
}


void AmrelTimer::sobelTest ()
{
  if (! amrel->loadRorpoMap ())
  {
    std::cout << "Sobel : rorpo map loading failed" << std::endl;
    return;
  }
  std::cout << "Time perf for Sobel..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
        = std::chrono::high_resolution_clock::now ();
  for (int i = 0; i < test_count; i++)
  {
    amrel->clearSobel ();
    amrel->processSobel (amrel->vmWidth (), amrel->vmHeight ());
  }
  std::chrono::high_resolution_clock::time_point end
        = std::chrono::high_resolution_clock::now ();
  std::chrono::duration<double> time_span
        = std::chrono::duration_cast<std::chrono::duration<double>> (
            end - start);
  std::cout << "Sobel: timing for " << test_count << " run = "
            << time_span.count () << " s" << std::endl;
  if (! amrel->saveSobelMap ())
    std::cout << "Sobel : map saving failed" << std::endl;
  else
  {
    if (amrel->config()->isOutMapOn ()) amrel->saveSobelImage ();
    amrel->clearRorpo ();
  }
}


void AmrelTimer::fbsdTest ()
{
  if (! amrel->loadSobelMap ())
  {
    std::cout << "Fbsd : sobel map loading failed" << std::endl;
    return;
  }
  std::cout << "Time perf for FBSD..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
        = std::chrono::high_resolution_clock::now ();
  for (int i = 0; i < test_count; i++)
  {
    amrel->clearFbsd ();
    amrel->processFbsd ();
  }
  std::chrono::high_resolution_clock::time_point end
        = std::chrono::high_resolution_clock::now ();
  std::chrono::duration<double> time_span
        = std::chrono::duration_cast<std::chrono::duration<double>> (
            end - start);
  std::cout << "FBSD: timing for " << test_count << " run = "
            << time_span.count () << " s" << std::endl;
  if (! amrel->saveFbsdSegments ())
    std::cout << "Fbsd : segments saving failed" << std::endl;
  else
  {
    if (amrel->config()->isOutMapOn ())
      amrel->saveFbsdImage (amrel->vmWidth (), amrel->vmHeight ());
    amrel->clearSobel ();
  }
}


void AmrelTimer::seedsTest ()
{
  // Tile set and blurred segments loading
  if (! amrel->loadTileSet (false, false))
  {
    std::cout << "Seeds : tile loading failed" << std::endl;
    return;
  }
  if (! amrel->loadFbsdSegments ())
  {
    std::cout << "Seeds : FBSD segments loading failed" << std::endl;
    return;
  }
  std::cout << "Time perf for seeds production..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
        = std::chrono::high_resolution_clock::now ();
  
  for (int i = 0; i < test_count; i++)
  {
    amrel->clearSeeds ();
    amrel->processSeeds ();
  }
  std::chrono::high_resolution_clock::time_point end
        = std::chrono::high_resolution_clock::now ();
  std::chrono::duration<double> time_span
        = std::chrono::duration_cast<std::chrono::duration<double>> (
            end - start);
  std::cout << "Seeds: timing for " << test_count << " run = "
            << time_span.count () << " s" << std::endl;
  if (! amrel->saveSeeds ())
    std::cout << "Seeds : seeds saving failed" << std::endl;
  else
    if (amrel->config()->isOutMapOn ()) amrel->saveSeedsImage ();
}


void AmrelTimer::asdTest ()
{
  // Raw points and seeds loading
  if (! amrel->loadSeeds ())
  {
    std::cout << "Asd : seeds loading failed" << std::endl;
    return;
  }
  if (! amrel->loadTileSet (false, false)) // requires width & height
  {
    std::cout << "Asd : tile loading failed" << std::endl;
    return;
  }
  amrel->addTrackDetector ();

  // Tracks detection and output
  std::cout << "Time perf for ASD..." << std::endl;
  std::chrono::high_resolution_clock::time_point start
        = std::chrono::high_resolution_clock::now ();
  for (int i = 0; i < test_count; i++)
  {
    amrel->clearAsd ();
    amrel->processAsd ();
  }
  std::chrono::high_resolution_clock::time_point end
        = std::chrono::high_resolution_clock::now ();
  std::chrono::duration<double> time_span
        = std::chrono::duration_cast<std::chrono::duration<double>> (
            end - start);
  std::cout << "Asd: timing for " << test_count << " run = "
            << time_span.count () << " s" << std::endl;
  amrel->saveAsdImage ();
}

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
#include <cmath>
#include <ctime>
#include "amreltool.h"
#include "shapefil.h"

#include "rorpo.hpp"
#include "image_png.hpp"


const int AmrelTool::NOMINAL_PLATEAU_LACK_TOLERANCE = 5;
const int AmrelTool::NOMINAL_PLATEAU_MAX_TILT = 10;
const float AmrelTool::NOMINAL_MAX_SHIFT_LENGTH = 0.5f;
const float AmrelTool::NOMINAL_PLATEAU_MIN_LENGTH = 2.0f;
const float AmrelTool::NOMINAL_PLATEAU_THICKNESS_TOLERANCE = 0.25f;
const float AmrelTool::NOMINAL_SLOPE_TOLERANCE = 0.10f;
const float AmrelTool::NOMINAL_SIDE_SHIFT_TOLERANCE = 0.5f;

const unsigned int AmrelTool::HUE_BACK = (unsigned int) 16777215;
const unsigned int AmrelTool::HUE_GRAY = (unsigned int) 65793;
const unsigned int AmrelTool::HUE_RED = (unsigned int) 65536;
const unsigned int AmrelTool::HUE_GREEN = (unsigned int) 256;
const unsigned int AmrelTool::HUE_BLUE = (unsigned int) 1;


AmrelTool::AmrelTool ()
{
  sub_div = AmrelConfig::DTM_GRID_SUBDIVISION_FACTOR;
  dtm_in = NULL;
  rorpo_map = NULL;
  ctdet = NULL;
  dtm_map = NULL;
  gmap = NULL;
  ptset = NULL;
  tile_loaded = false;
  buf_created = false;
  iratio = 1.0f;
  out_seeds = NULL;
  out_sucseeds = NULL;
  vm_width = 0;
  vm_height = 0;
  if (bsdet.isSingleEdgeModeOn ()) bsdet.switchSingleOrDoubleEdge ();
  if (bsdet.isNFA ()) bsdet.switchNFA ();
  save_seeds = true;
  detection_map = NULL;
}


AmrelTool::~AmrelTool ()
{
  clearFbsd ();
  clearSeeds ();
  clearAsd ();
  if (detection_map != NULL) delete detection_map;
}


void AmrelTool::clear ()
{
  if (ptset != NULL) delete ptset;
  ptset = NULL;
  if (dtm_in != NULL) delete dtm_in;
  dtm_in = NULL;
  tile_loaded = false;
  buf_created = false;
}


void AmrelTool::clearPoints ()
{
  if (ptset != NULL) delete ptset;
  ptset = NULL;
  tile_loaded = false;
  buf_created = false;
}


void AmrelTool::clearDtm ()
{
  if (dtm_in != NULL) delete dtm_in;
  dtm_in = NULL;
}


void AmrelTool::clearShading ()
{
  if (dtm_map != NULL) delete [] dtm_map;
  dtm_map = NULL;
}


void AmrelTool::clearRorpo ()
{
  if (rorpo_map != NULL) delete [] rorpo_map;
  rorpo_map = NULL;
}


void AmrelTool::clearSobel ()
{
  bsdet.clearAll ();
  if (gmap != NULL) delete gmap;
  gmap = NULL;
}


void AmrelTool::clearFbsd ()
{
  dss.clear ();
}


void AmrelTool::clearSeeds ()
{
  if (out_seeds != NULL)
  {
    int tsize = ptset->rowsOfTiles () * ptset->columnsOfTiles ();
    for (int i = 0; i < tsize; i++) out_seeds[i].clear ();
    delete [] out_seeds;
  }
  out_seeds = NULL;
}


void AmrelTool::clearAsd ()
{
  if (out_sucseeds != NULL)
  {
    int tsize = ptset->rowsOfTiles () * ptset->columnsOfTiles ();
    for (int i = 0; i < tsize; i++) out_sucseeds[i].clear ();
    delete [] out_sucseeds;
    out_sucseeds = NULL;
  }
}


void AmrelTool::addTrackDetector ()
{
  ctdet = new CTrackDetector ();
  ctdet->setPlateauLackTolerance (NOMINAL_PLATEAU_LACK_TOLERANCE);
  ctdet->setMaxShiftLength (NOMINAL_MAX_SHIFT_LENGTH);
  if (ctdet->isInitializationOn ()) ctdet->switchInitialization ();
  ctdet->model()->setMinLength (NOMINAL_PLATEAU_MIN_LENGTH);
  ctdet->model()->setThicknessTolerance (NOMINAL_PLATEAU_THICKNESS_TOLERANCE);
  ctdet->model()->setSlopeTolerance (NOMINAL_SLOPE_TOLERANCE);
  ctdet->model()->setSideShiftTolerance (NOMINAL_SIDE_SHIFT_TOLERANCE);
  ctdet->model()->setBSmaxTilt (NOMINAL_PLATEAU_MAX_TILT);
  if (ptset != NULL)
    ctdet->setPointsGrid (ptset, vm_width, vm_height, sub_div, csize);
  cfg.setDetector (ctdet);
  ctdet->setAutomatic (true);
  adaptTrackDetector ();
}


void AmrelTool::checkDetector ()
{
  if (ctdet == NULL) addTrackDetector ();
  std::cout << "Lack tol = " << 
    ctdet->getPlateauLackTolerance () << std::endl;
  std::cout << "Max shift length = " << 
    ctdet->maxShiftLength () << std::endl;
  std::cout << "Initializ = " << 
    ctdet->isInitializationOn () << std::endl;
  std::cout << "Min length = " << 
    ctdet->model()->minLength () << std::endl;
  std::cout << "Th tol = " << 
    ctdet->model()->thicknessTolerance () << std::endl;
  std::cout << "Sl tol = " << 
    ctdet->model()->slopeTolerance () << std::endl;
  std::cout << "Side shift tol = " << 
    ctdet->model()->sideShiftTolerance () << std::endl;
  std::cout << "BS max tilt = " << 
    ctdet->model()->bsMaxTilt () << std::endl;
  std::cout << "Sub div = " << sub_div << std::endl;
  std::cout << "Csize = " << csize << std::endl;
}


bool AmrelTool::loadTileSet (bool dtm_on, bool pts_on)
{
  if (dtm_on && dtm_in == NULL) dtm_in = new TerrainMap ();
  if (ptset == NULL) ptset = new IPtTileSet (cfg.bufferSize ());
  if (ctdet != NULL)
    ctdet->setPointsGrid (ptset, vm_width, vm_height, sub_div, csize);

  char sval[200];
  std::vector<int> vals;
  std::ifstream input (cfg.tiles().c_str (), std::ios::in);
  bool reading = true;
  if (input)
  {
    while (reading)
    {
      input >> sval;
      if (input.eof ()) reading = false;
      else
      {
        std::string nvmfile (cfg.nvmDir ());
        if (dtm_on) nvmfile += sval + TerrainMap::NVM_SUFFIX;
        std::string ptsfile (cfg.tilPrefix ());
        ptsfile += sval + IPtTile::TIL_SUFFIX;
        if (dtm_on) dtm_in->addNormalMapFile (nvmfile);
        if (cfg.isVerboseOn ())
          std::cout << "Reading " << nvmfile << std::endl;
        if (! ptset->addTile (ptsfile, pts_on))
        {
          bool ok = cfg.createAltXyz (sval);
          if (ok) ok = ptset->addTile (ptsfile, pts_on);
          if (! ok)
          {
            std::cout << "Header of " << ptsfile << " inconsistent"
                      << std::endl;
            return false;
          }
        }
        if (cfg.isVerboseOn ())
          std::cout << "Reading " << ptsfile << std::endl;
      }
    }
    input.close ();
  }
  else
  {
    std::cout << "No " << cfg.tiles () << " file found" << std::endl;
    return false;
  }

  if (! ptset->create ()) return false;
  if (cfg.isVerboseOn ())
    std::cout << ptset->size () << " points in the whole tile set" << std::endl;
  if (dtm_on)
  {
    if (! dtm_in->assembleMap (ptset->columnsOfTiles (), ptset->rowsOfTiles (),
                               ptset->xref (), ptset->yref ())) return false;
    vm_width = dtm_in->width ();
    vm_height = dtm_in->height ();
    csize = dtm_in->cellSize ();
  }
  iratio = vm_width / ptset->xmSpread ();
  return true;
}


bool AmrelTool::loadPoints ()
{
  return (ptset != NULL && ptset->loadPoints ());
}


void AmrelTool::run ()
{
  if (cfg.isNewLidarOn ())
  {
    cfg.importAllDtmFiles ();
    return;
  }
  // TILE IMPORTS
  if (cfg.isDtmImportOn () || cfg.isXyzImportOn ())
  {
    if (cfg.isDtmImportOn ()) cfg.importDtm ();
    if (cfg.isXyzImportOn ()) cfg.importXyz ();
    return;
  }
  if (! cfg.setTiles ()) return;
  if (cfg.isSeedCheckOn ())
  {
    if (loadTileSet (false, false)) checkSeeds ();
  }
  else if (cfg.isHillMapOn ())
  {
    if (loadTileSet (true, false))
    {
      saveHillImage ();
      clear ();
    }
    return;
  }

  // FULL AUTOMATIC DETECTION
  //-------------------------------------------------------------------------
  else if (cfg.step () == AmrelConfig::STEP_ALL)
  {
    if (processSawing ())
      if (processAsd ())
      {
        detection_map->setDisplayedSeeds (&connection_seeds);
        saveAsdImage (AmrelConfig::RES_DIR
                      + AmrelConfig::ROAD_FILE + AmrelConfig::IM_SUFFIX);
        if (cfg.isExportOn ())
        {
          if (cfg.isExportBoundsOn ()) exportRoads ();
          else exportRoadCenters ();
        }
      }
  }


  // FULL AUTOMATIC SEED SELECTION
  //-------------------------------------------------------------------------
  else if (cfg.step () == AmrelConfig::STEP_SAWING)
  {
    if (processSawing ()) saveSeeds ();
    if (cfg.isVerboseOn () && cfg.isOutMapOn ())
      std::cout
        << "--map : only with --shade, --rorpo, --sobel, --fbsd or --seeds"
        << std::endl;
  }


  // AUTOMATIC DETECTION STEP 1 : SHADE
  //-------------------------------------------------------------------------
  else if (cfg.step () == AmrelConfig::STEP_SHADE)
  {
    // Tile files loading (only DTM and raw point file headers)
    if (! loadTileSet (true, false)) return;
    processShading ();
    if (saveShadingMap ())
    {
      if (cfg.isOutMapOn ()) saveShadingImage ();
      clearDtm ();
    }
  }


  // AUTOMATIC DETECTION STEP 2 : RORPO
  //-------------------------------------------------------------------------
  else if (cfg.step () == AmrelConfig::STEP_RORPO)
  {
    if (! loadShadingMap ()) return;
    processRorpo (vm_width, vm_height);
    if (saveRorpoMap ())
    {
      if (cfg.isOutMapOn ()) saveRorpoImage ();
      clearShading ();
    }
  }


  // AUTOMATIC DETECTION STEP 3 : SOBEL
  //-------------------------------------------------------------------------
  else if (cfg.step () == AmrelConfig::STEP_SOBEL)
  {
    if (cfg.rorpoSkipped ())
    {
      if (! loadShadingMap ()) return;
    }
    else if (! loadRorpoMap ()) return;
    processSobel (vm_width, vm_height);
    if (saveSobelMap ())
    {
      if (cfg.isOutMapOn ()) saveSobelImage ();
      if (cfg.rorpoSkipped ()) clearShading ();
      else clearRorpo ();
    }
  }


  // AUTOMATIC DETECTION STEP 4 : FBSD
  //-------------------------------------------------------------------------
  else if (cfg.step () == AmrelConfig::STEP_FBSD)
  {
    if (! loadSobelMap ()) return;
    processFbsd ();
    if (saveFbsdSegments ())
    {
      if (cfg.isOutMapOn ()) saveFbsdImage (vm_width, vm_height);
      clearSobel ();
    }
  }


  // AUTOMATIC DETECTION STEP 5 : SEEDS
  //-------------------------------------------------------------------------
  else if (cfg.step () == AmrelConfig::STEP_SEEDS)
  {
    // Tile set and blurred segments loading
    if (! loadTileSet (false, false)) return;
    if (! loadFbsdSegments ()) return;
    processSeeds ();
    if (saveSeeds ())
      if (cfg.isOutMapOn ()) saveSeedsImage ();
  }


  // AUTOMATIC DETECTION STEP 6 : ASD
  //-------------------------------------------------------------------------
  else if (cfg.step () == AmrelConfig::STEP_ASD)
  {
    // Raw points and seeds loading
    if (! loadSeeds ()) return;
    if (! loadTileSet (false, false)) return; // requires width & height
    processAsd ();
    saveAsdImage (AmrelConfig::RES_DIR
                  + AmrelConfig::ROAD_FILE + AmrelConfig::IM_SUFFIX);
    if (cfg.isExportOn ())
    {
      if (cfg.isExportBoundsOn ()) exportRoads ();
      else exportRoadCenters ();
    }
  }
}


void AmrelTool::processShading ()
{
  if (cfg.isVerboseOn ()) std::cout << "Shading ..." << std::endl;
  if (dtm_map == NULL) dtm_map = new unsigned char[vm_width * vm_height];
  unsigned char *dtmp = dtm_map;
  int shtype = (cfg.rorpoSkipped () ? TerrainMap::SHADE_EXP_SLOPE
                                    : TerrainMap::SHADE_SLOPE);
  for (int j = 0; j < vm_height; j ++)
    for (int i = 0; i < vm_width; i ++)
      *dtmp ++ = (unsigned char) dtm_in->get (i, j, shtype);
  if (cfg.isVerboseOn ()) std::cout << "Shading OK" << std::endl;
}


void AmrelTool::processSobel (int w, int h)
{
  if (cfg.isVerboseOn ()) std::cout << "Sobel 5x5 ..." << std::endl;
  if (cfg.rorpoSkipped ())
    gmap = new VMap (w, h, dtm_map, VMap::TYPE_SOBEL_5X5);
  else gmap = new VMap (w, h, rorpo_map, VMap::TYPE_SOBEL_5X5);
  bsdet.setGradientMap (gmap);
  if (cfg.isVerboseOn ()) std::cout << "Sobel 5x5 OK" << std::endl;
}


void AmrelTool::processFbsd ()
{
  if (cfg.isVerboseOn ()) std::cout << "FBSD ..." << std::endl;
  bsdet.setAssignedThickness (cfg.maxBSThickness ());
  bsdet.resetMaxDetections ();
  bsdet.detectAll ();
  bsdet.copyDigitalStraightSegments (dss);
  if (cfg.isVerboseOn ()) std::cout << "FBSD OK : " << dss.size ()
                                    << " blurred segments" << std::endl;
}


void AmrelTool::processSeeds (int kref)
{
  if (cfg.isVerboseOn ()) std::cout << "Seeds ..." << std::endl;
  int nbs = 0;
  int nbsmall = 0;
  int nbout = 0;
  int max = 0;
  AbsRat x1r, y1r, x2r, y2r;
  float x1, y1, x2, y2, ln, dx, dy;

  int tsw = ptset->columnsOfTiles();
  int tsh = ptset->rowsOfTiles();
  if (out_seeds == NULL) out_seeds = new std::vector<Pt2i>[tsw * tsh];
  int tw = vm_width / tsw;
  int th = vm_height / tsh;
  if (dtm_in != NULL)
  {
    tw = dtm_in->tileWidth ();
    th = dtm_in->tileHeight ();
  }
  int kx = 0, ky = 0;
  int pim_h = vm_height;
  if (kref != -1)
  {
    kx = kref % tsw;
    ky = kref / tsw;
    if (dtm_in != NULL) pim_h = dtm_in->padHeight () * th;
  }
  int skx = kx * tw;
  int sky = ky * th + pim_h - 1;
  int mbsl2 = cfg.minBSLength () * cfg.minBSLength ();

  std::vector<DigitalStraightSegment>::iterator it = dss.begin ();
  int sshift = cfg.seedShift ();
  int sw2 = cfg.seedWidth () / 2;
  while (it != dss.end ())
  {
    int dsl = it->length2 ();
    if (dsl > max) max = dsl;
    if (dsl < mbsl2) nbsmall ++;
    else
    {
      it->naiveLine (x1r, y1r, x2r, y2r);
      x1 = x1r.num () / (float) x1r.den ();
      y1 = y1r.num () / (float) y1r.den ();
      x2 = x2r.num () / (float) x2r.den ();
      y2 = y2r.num () / (float) y2r.den ();
      ln = (float) sqrt ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
      dx = (x2 - x1) / ln;
      dy = (y2 - y1) / ln;
      for (float pos = 0.0f; pos <= ln; pos += sshift)
      {
        Pt2i pt1 (skx + (int) (x1 + pos * dx - sw2 * dy + 0.5f),
                  sky - (int) (y1 + pos * dy + sw2 * dx + 0.5f));
        Pt2i pt2 (skx + (int) (x1 + pos * dx + sw2 * dy + 0.5f),
                  sky - (int) (y1 + pos * dy - sw2 * dx + 0.5f));
        if (pt1.x () < 0 || pt1.x () >= tsw * tw
            || pt1.y () < 0 || pt1.y () >= tsh * th
            || pt2.x () < 0 || pt2.x () >= tsw * tw
            || pt2.y () < 0 || pt2.y () >= tsh * th) nbout ++;
        else
        {
          int tilex = ((pt1.x () + pt2.x ()) / 2) / tw;
          if (tilex < 0) tilex = 0;
          else if (tilex >= tsw) tilex = tsw - 1;
          int tiley = ((pt1.y () + pt2.y ()) / 2) / th;
          if (tiley < 0) tiley = 0;
          else if (tiley >= tsh) tiley = tsh - 1;
          // Ckecks the tile exists ...
          if (ptset->isLoaded (tiley * tsw + tilex))
          {
            out_seeds[tiley * tsw + tilex].push_back (pt1);
            out_seeds[tiley * tsw + tilex].push_back (pt2);
            nbs ++;
          }
          else nbout ++;
        }
      }
    }
    it ++;
  }
  if (cfg.isVerboseOn ())
    std::cout << "Seeds OK : " << nbs << " seeds, " << nbsmall
    //          << " rejected segments, " << nbout << " seeds out BS"
              << " rejected segments"
              << std::endl;
}


bool AmrelTool::processAsd ()
{
  if (cfg.isVerboseOn ()) std::cout << "ASD ..." << std::endl;
  road_sections.clear ();
  int num = 0;
  int unused = 0;
  if (cfg.bufferSize () == 0 && ! tile_loaded)
  {
    if (ptset->loadPoints ()) tile_loaded = true;
    else
    {
      std::cout << "Tiles cannot be loaded" << std::endl;
      return false;
    }
  }
  int cot = ptset->columnsOfTiles ();
  int rot = ptset->rowsOfTiles ();
  out_sucseeds = new std::vector<Pt2i>[cot * rot];
  if (detection_map != NULL) delete detection_map;
  detection_map = new AmrelMap (vm_width, vm_height, &cfg);
  if (ctdet == NULL) addTrackDetector ();
  std::vector<Pt2i>::iterator it;

  if (cfg.bufferSize () != 0)
  {
    if (! buf_created) ptset->createBuffers ();
    buf_created = true; // avoids re-creation
    int k = ptset->nextTile ();
    while (k != -1)
    {
      if (cfg.isVerboseOn ())
        std::cout << "  --> Tile " << k << " (" << k % cot << ", " << k / cot
                  << ") : " << out_seeds[k].size () << " seeds" << std::endl;
      it = out_seeds[k].begin ();
      while (it != out_seeds[k].end ())
      {
        Pt2i p1 (*it++);
        Pt2i p2 (*it++);
        Pt2i center ((p1.x () + p2.x ()) / 2, (p1.y () + p2.y ()) / 2);
        if (detection_map->occupied (center)) unused ++;
        else
        {
          CarriageTrack *ct = ctdet->detect (p1, p2);
          if (ct != NULL && ct->plateau (0) != NULL)
          {
            std::vector<std::vector<Pt2i> > pts;
            if (cfg.isConnectedOn ())
              ct->getConnectedPoints (&pts, true, vm_width, vm_height, iratio);
            else ct->getPoints (&pts, true, vm_width, vm_height, iratio);
            if (detection_map->add (pts))
            {
              out_sucseeds[k].push_back (p1);
              out_sucseeds[k].push_back (p2);
              if (cfg.isExportOn ())
              {
                road_sections.push_back (ct);
                ctdet->preserveDetection ();
              }
            }
            num ++;
          }
        }
      }
      if (ctdet->getOuts () != 0)
        std::cout << "  " << ctdet->getOuts () << " requests outside\n"
                  << std::endl;
      ctdet->resetOuts ();
      k = ptset->nextTile ();
    }
  }

  else
  {
    for (int j = 0; j < rot; j++)
    {
      for (int i = 0; i < cot; i++)
      {
        int k = j * cot + ((j % 2 != 0) ? cot - 1 - i : i);
        it = out_seeds[k].begin ();
        while (it != out_seeds[k].end ())
        {
          Pt2i p1 (*it++);
          Pt2i p2 (*it++);
          Pt2i center ((p1.x () + p2.x ()) / 2, (p1.y () + p2.y ()) / 2);
          if (detection_map->occupied (center)) unused ++;
          else
          {
            CarriageTrack *ct = ctdet->detect (p1, p2);
            if (ct != NULL && ct->plateau (0) != NULL)
            {
              std::vector<std::vector<Pt2i> > pts;
              if (cfg.isConnectedOn ())
                ct->getConnectedPoints (&pts, true,
                                        vm_width, vm_height, iratio);
              else ct->getPoints (&pts, true, vm_width, vm_height, iratio);
              if (isConnected (pts))
              {
                if (detection_map->add (pts))
                {
                  out_sucseeds[k].push_back (p1);
                  out_sucseeds[k].push_back (p2);
                  if (cfg.isExportOn ())
                  {
                    road_sections.push_back (ct);
                    ctdet->preserveDetection ();
                  }
                }
              }
              else std::cout << "Road section " << num
                             << " is not connected" << std::endl;
              num ++;
            }
          }
        }
      }
    }
  }
  if (save_seeds)
  {
    saveSuccessfulSeeds ();
    cfg.saveDetectorStatus ();
  }
  if (cfg.isVerboseOn ()) std::cout << "ASD OK : " << num << " roads and "
                                    << unused << " unused seeds" << std::endl;
  return true;
}


bool AmrelTool::processSawing ()
{
  if (cfg.padSize () == 0)
  {
    if (! loadTileSet (true, false)) return false;
    processShading ();
    clearDtm ();
    if (! cfg.rorpoSkipped ())
    {
      processRorpo (vm_width, vm_height);
      clearShading ();
    }
    processSobel (vm_width, vm_height);
    if (cfg.rorpoSkipped ()) clearShading ();
    else clearRorpo ();
    processFbsd ();
    clearSobel ();
    processSeeds ();
    clearFbsd ();
    return true;
  }

  dtm_in = new TerrainMap ();
  dtm_in->setPadSize (cfg.padSize ());
  ptset = new IPtTileSet ();
  char sval[12];
  std::vector<int> vals;
  std::ifstream input (cfg.tiles().c_str (), std::ios::in);
  bool reading = true;
  if (input.is_open ())
  {
    while (reading)
    {
      input >> sval;
      if (input.eof ()) reading = false;
      else
      {
        std::string nvmfile (cfg.nvmDir ());
        nvmfile += sval + TerrainMap::NVM_SUFFIX;
        std::string ptsfile (cfg.tilPrefix ());
        ptsfile += sval + IPtTile::TIL_SUFFIX;
        dtm_in->addNormalMapFile (nvmfile);
        if (cfg.isVerboseOn ()) std::cout << "Reading " << nvmfile << std::endl;
        if (! ptset->addTile (ptsfile, false))
        {
          std::cout << "Header of " << ptsfile << " inconsistent" << std::endl;
          delete dtm_in;
          delete ptset;
          return false;
        }
      }
    }
    input.close ();
  }
  else
  {
    std::cout << "No " << cfg.tiles () << " file found" << std::endl;
    delete dtm_in;
    delete ptset;
    return false;
  }
  if (! ptset->create ())
  {
    std::cout << "Unable to create the point tile set" << std::endl;
    delete dtm_in;
    delete ptset;
    return false;
  }
  if (! dtm_in->assembleMap (ptset->columnsOfTiles (), ptset->rowsOfTiles (),
                             ptset->xref (), ptset->yref (),
                             true))     // for AMREL std
                             // false))    // for AMRELnet
  {
    std::cout << "Unable to arrange DTM files in space" << std::endl;
    delete dtm_in;
    delete ptset;
    return false;
  }
  dtm_in->adjustPadSize ();
  int pad_w = dtm_in->padWidth ();
  int pad_h = dtm_in->padHeight ();
  int dtm_w = dtm_in->tileWidth ();
  int dtm_h = dtm_in->tileHeight ();
  vm_width = dtm_w * ptset->columnsOfTiles ();
  vm_height = dtm_h * ptset->rowsOfTiles ();
  csize = dtm_in->cellSize ();
  dtm_map = new unsigned char[pad_w * dtm_w * pad_h * dtm_h];
  if (! cfg.rorpoSkipped ())
    rorpo_map = new unsigned char[pad_w * dtm_w * pad_h * dtm_h];
  out_seeds =
    new std::vector<Pt2i>[ptset->columnsOfTiles() * ptset->rowsOfTiles()];

  // Creates seed map
  int k = dtm_in->nextPad (dtm_map);
  while (k != -1)
  {
    if (cfg.isVerboseOn ())
      std::cout << "  --> Pad " << k << " (" << (k % ptset->columnsOfTiles ())
                << ", " << (k / ptset->columnsOfTiles ()) << "):" << std::endl;
    if (! cfg.rorpoSkipped ()) processRorpo (pad_w * dtm_w, pad_h * dtm_h);
    processSobel (pad_w * dtm_w, pad_h * dtm_h);
    if (! cfg.rorpoSkipped ())
    {
      unsigned char *mymap = rorpo_map;
      for (int i = 0; i < pad_h * dtm_h * pad_w * dtm_w; i++)
        *mymap++ = (unsigned char) 0;
    }
    processFbsd ();
    clearSobel ();
    processSeeds (k);
    clearFbsd ();
    k = dtm_in->nextPad (dtm_map);
  }
  if (! cfg.rorpoSkipped ()) clearRorpo ();
  clearShading ();
  return true;
}


bool AmrelTool::saveShadingMap ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::SLOPE_FILE
                    + AmrelConfig::MAP_SUFFIX);
  std::ofstream shading_out (name.c_str (), std::ios::out);
  if (! shading_out.is_open ())
  {
    std::cout << "Can't save shaded-DTM in " << name << std::endl;
    return false;
  }
  shading_out.write ((char *) (&vm_width), sizeof (int));
  shading_out.write ((char *) (&vm_height), sizeof (int));
  shading_out.write ((char *) (&csize), sizeof (float));
  shading_out.write ((char *) dtm_map,
                     vm_width * vm_height * sizeof (unsigned char));
  shading_out.close ();
  return true;
}


bool AmrelTool::loadShadingMap ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::SLOPE_FILE
                    + AmrelConfig::MAP_SUFFIX);
  std::ifstream shading_in (name.c_str (), std::ios::in);
  if (! shading_in.is_open ())
  {
    std::cout << name << ": can't be opened" << std::endl;
    return false;
  }
  shading_in.read ((char *) (&vm_width), sizeof (int));
  shading_in.read ((char *) (&vm_height), sizeof (int));
  shading_in.read ((char *) (&csize), sizeof (float));
  dtm_map = new unsigned char[vm_width * vm_height];
  shading_in.read ((char *) dtm_map,
                   vm_width * vm_height * sizeof (unsigned char));
  shading_in.close ();
  return true;
}


bool AmrelTool::saveRorpoMap ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::RORPO_FILE
                    + AmrelConfig::MAP_SUFFIX);
  std::ofstream rorpo_out (name.c_str (), std::ios::out);
  if (! rorpo_out.is_open ())
  {
    std::cout << "Can't save Rorpo map in " << name << std::endl;
    return false;
  }
  rorpo_out.write ((char *) (&vm_width), sizeof (int));
  rorpo_out.write ((char *) (&vm_height), sizeof (int));
  rorpo_out.write ((char *) (&csize), sizeof (float));
  rorpo_out.write ((char *) rorpo_map,
                   vm_width * vm_height * sizeof (unsigned char));
  rorpo_out.close ();
  return true;
}


bool AmrelTool::loadRorpoMap ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::RORPO_FILE
                    + AmrelConfig::MAP_SUFFIX);
  std::ifstream rorpo_in (name.c_str (), std::ios::in);
  if (! rorpo_in.is_open ())
  {
    std::cout << name << ": can't be opened" << std::endl;
    return false;
  }
  rorpo_in.read ((char *) (&vm_width), sizeof (int));
  rorpo_in.read ((char *) (&vm_height), sizeof (int));
  rorpo_in.read ((char *) (&csize), sizeof (float));
  rorpo_map = new unsigned char[vm_width * vm_height];
  rorpo_in.read ((char *) rorpo_map,
                 vm_width * vm_height * sizeof (unsigned char));
  rorpo_in.close ();
  return true;
}


bool AmrelTool::saveSobelMap ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::SOBEL_FILE
                    + AmrelConfig::MAP_SUFFIX);
  std::ofstream sobel_out (name.c_str (), std::ios::out);
  if (! sobel_out.is_open ())
  {
    std::cout << "Can't save Sobel map in " << name << std::endl;
    return false;
  }
  sobel_out.write ((char *) (&vm_width), sizeof (int));
  sobel_out.write ((char *) (&vm_height), sizeof (int));
  sobel_out.write ((char *) (&csize), sizeof (float));
  Vr2i *vmap = gmap->getVectorMap ();
  sobel_out.write ((char *) vmap, vm_width * vm_height * sizeof (Vr2i));
  sobel_out.close ();
  return true;
}


bool AmrelTool::loadSobelMap ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::SOBEL_FILE
                    + AmrelConfig::MAP_SUFFIX);
  std::ifstream sobel_in (name.c_str (), std::ios::in);
  if (! sobel_in.is_open ())
  {
    std::cout << name << ": can't be opened" << std::endl;
    return false;
  }
  sobel_in.read ((char *) (&vm_width), sizeof (int));
  sobel_in.read ((char *) (&vm_height), sizeof (int));
  sobel_in.read ((char *) (&csize), sizeof (float));
  Vr2i *im = new Vr2i[vm_width * vm_height];
  sobel_in.read ((char *) im, vm_width * vm_height * sizeof (Vr2i));
  sobel_in.close ();
  gmap = new VMap (vm_width, vm_height, im);
  bsdet.setGradientMap (gmap);
  return true;
}


bool AmrelTool::saveFbsdSegments ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::FBSD_FILE
                    + AmrelConfig::FBSD_SUFFIX);
  std::ofstream fbsd_out (name.c_str (), std::ios::out);
  if (! fbsd_out.is_open ())
  {
    std::cout << "Can't save FBSD segments in " << name << std::endl;
    return false;
  }
  fbsd_out.write ((char *) (&vm_width), sizeof (int));
  fbsd_out.write ((char *) (&vm_height), sizeof (int));
  fbsd_out.write ((char *) (&csize), sizeof (float));
  int nb = 0;
  DigitalStraightSegment *ds
    = new DigitalStraightSegment[(int) (dss.size ())];
  std::vector<DigitalStraightSegment>::iterator it = dss.begin ();
  while (it != dss.end ()) ds[nb++].set (*it++);
  fbsd_out.write ((char *) (&nb), sizeof (int));
  fbsd_out.write ((char *) ds, nb * sizeof (DigitalStraightSegment));
  fbsd_out.close ();
  delete [] ds;
  return true;
}


bool AmrelTool::loadFbsdSegments ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::FBSD_FILE
                    + AmrelConfig::FBSD_SUFFIX);
  std::ifstream fbsd_in (name.c_str (), std::ios::in);
  if (! fbsd_in.is_open ())
  {
    std::cout << name << ": can't be opened" << std::endl;
    return false;
  }
  fbsd_in.read ((char *) (&vm_width), sizeof (int));
  fbsd_in.read ((char *) (&vm_height), sizeof (int));
  fbsd_in.read ((char *) (&csize), sizeof (float));
  int nb = 0;
  fbsd_in.read ((char *) (&nb), sizeof (int));
  char *dss_in = new char[nb * sizeof (DigitalStraightSegment)];
  fbsd_in.read ((char *) dss_in, nb * sizeof (DigitalStraightSegment));
  fbsd_in.close ();
  DigitalStraightSegment *ds = (DigitalStraightSegment *) dss_in;
  for (int i = 0; i < nb; i++) dss.push_back (*ds++);
  delete [] dss_in;
  return true;
}


bool AmrelTool::saveSeeds ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::SEED_FILE
                    + AmrelConfig::SEED_SUFFIX);
  if (cfg.isVerboseOn ())
    std::cout << "Saving seeds in " << name << std::endl;
  std::ofstream seeds_out (name.c_str (), std::ios::out);
  if (! seeds_out.is_open ())
  {
    std::cout << "Can't save seeds in " << name << std::endl;
    return false;
  }
  int nb = 0, rot = ptset->rowsOfTiles (), cot = ptset->columnsOfTiles ();
  int vmw = vm_width, vmh = vm_height, vmc = cot, vmr = rot;
  float vms = csize;
  if (cfg.isHalfSizeSeedsOn ())
  {
    vmw *= 2;
    vmh *= 2;
    vmc *= 2;
    vmr *= 2;
    vms /= 2;
  }
  seeds_out.write ((char *) (&vmw), sizeof (int));
  seeds_out.write ((char *) (&vmh), sizeof (int));
  seeds_out.write ((char *) (&vms), sizeof (float));
  seeds_out.write ((char *) (&vmc), sizeof (int));
  seeds_out.write ((char *) (&vmr), sizeof (int));
  if (cfg.isHalfSizeSeedsOn ())
  {
    int kx = -1, ky = -1;
    int tw = vm_width / (2 * cot);
    int th = vm_height / (2 * rot);
    std::vector<Pt2i> *reseeds = new std::vector<Pt2i>[rot * cot * 4];
    std::vector<Pt2i>::iterator it;
    int numk = 0, outl = 0;
    for (int j = 0; j < rot; j++)
    {
      for (int i = 0; i < cot; i ++)
      {
        it = out_seeds[numk].begin ();
        while (it != out_seeds[numk].end ())
        {
          Pt2i pt1 (*it++);
          Pt2i pt2 (*it++);
          kx = ((pt1.x () + pt2.x ()) / 2) / tw;
          ky = ((pt1.y () + pt2.y ()) / 2) / th;
          if (kx < 0 || ky < 0 || kx >= 2 * cot || ky >= 2 * rot) outl ++;
          else
          {
            pt1.set (pt1.x () * 2, pt1.y () * 2);
            pt2.set (pt2.x () * 2, pt2.y () * 2);
            if (pt2.x () < pt1.x ()) pt1.set (pt1.x () + 1, pt1.y ());
            else pt2.set (pt2.x () + 1, pt2.y ());
            if (pt2.y () < pt1.y ()) pt1.set (pt1.x (), pt1.y () + 1);
            else pt2.set (pt2.x (), pt2.y () + 1);
            reseeds[ky * cot * 2 + kx].push_back (pt1);
            reseeds[ky * cot * 2 + kx].push_back (pt2);
          }
        }
        numk ++;
      }
    }
    if (outl != 0) std::cout << outl << " ousiders when retiling" << std::endl;
    for (int i = 0; i < cot * rot * 4; i++) nb += (int) (reseeds[i].size ());
    seeds_out.write ((char *) (&nb), sizeof (int));
    numk = 0;
    for (int j = 0; j < rot * 2; j++)
    {
      for (int i = 0; i < cot * 2; i ++)
      {
        it = reseeds[numk].begin ();
        while (it != reseeds[numk].end ())
        {
          Pt2i pt1 (*it++);
          Pt2i pt2 (*it++);
          seeds_out.write ((char *) &pt1, sizeof (Pt2i));
          seeds_out.write ((char *) &pt2, sizeof (Pt2i));
        }
        numk ++;
      }
    }
    delete [] reseeds;
  }
  else
  {
    for (int i = 0; i < cot * rot; i++) nb += (int) (out_seeds[i].size ());
    seeds_out.write ((char *) (&nb), sizeof (int));
    std::vector<Pt2i>::iterator it;
    for (int j = 0; j < rot; j++)
    {
      for (int i = 0; i < cot; i ++)
      {
        int k = j * cot + ((j % 2 != 0) ? cot - 1 - i : i);
        it = out_seeds[k].begin ();
        while (it != out_seeds[k].end ())
        {
          Pt2i pt1 (*it++);
          Pt2i pt2 (*it++);
          seeds_out.write ((char *) &pt1, sizeof (Pt2i));
          seeds_out.write ((char *) &pt2, sizeof (Pt2i));
        }
      }
    }
  }
  seeds_out.close ();
  return true;
}


bool AmrelTool::loadSeeds ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::SEED_FILE
                    + AmrelConfig::SEED_SUFFIX);
  std::ifstream seeds_in (name.c_str (), std::ios::in);
  if (! seeds_in.is_open ())
  {
    std::cout << name << ": can't be opened" << std::endl;
    return false;
  }
  if (cfg.isVerboseOn ())
    std::cout << "Loading seeds from " << name << std::endl;
  int tsw = 1, tsh = 1, nb = 0;
  seeds_in.read ((char *) (&vm_width), sizeof (int));
  seeds_in.read ((char *) (&vm_height), sizeof (int));
  seeds_in.read ((char *) (&csize), sizeof (float));
  seeds_in.read ((char *) (&tsw), sizeof (int));
  seeds_in.read ((char *) (&tsh), sizeof (int));
  seeds_in.read ((char *) (&nb), sizeof (int));
  Pt2i *pts = new Pt2i[nb];
  seeds_in.read ((char *) pts, nb * sizeof (Pt2i));
  seeds_in.close ();

  out_seeds = new std::vector<Pt2i>[tsh * tsw];
  int tw = vm_width / tsw;
  int th = vm_height / tsh;
  Pt2i *ppts = pts;
  for (int i = 0; i < nb; i += 2)
  {
    Pt2i pt1 = *ppts++;
    Pt2i pt2 = *ppts++;
    int tilex = ((pt1.x () + pt2.x ()) / 2) / tw;
    if (tilex < 0) tilex = 0;
    else if (tilex >= tsw) tilex = tsw - 1;
    int tiley = ((pt1.y () + pt2.y ()) / 2) / th;
    if (tiley < 0) tiley = 0;
    else if (tiley >= tsh) tiley = tsh - 1;
    out_seeds[tiley * tsw + tilex].push_back (pt1);
    out_seeds[tiley * tsw + tilex].push_back (pt2);
  }
  delete [] pts;
  return true;
}


void AmrelTool::checkSeeds ()
{
  std::cout << "Check seeds" << std::endl;
  int cot = ptset->columnsOfTiles ();
  int rot = ptset->rowsOfTiles ();
  std::vector<Pt2i>::iterator it;
  for (int j = 0; j < rot; j++)
  {
    for (int i = 0; i < cot; i++)
    {
      int k = j * cot + ((j % 2 != 0) ? cot - 1 - i : i);
      std::cout << "Seeds " << k << " (" << ((j % 2 != 0) ? cot - 1 - i : i)
                << ", " << j << ") : " << out_seeds[k].size () << std::endl;
      it = out_seeds[k].begin ();
      while (it != out_seeds[k].end ())
      {
        Pt2i p1 (*it++);
        Pt2i p2 (*it++);
        std::cout << "  seed (" << p1.x () << ", " << p1.y () << ") ("
                  << p2.x () << ", " << p2.y () << ")" << std::endl;
      }
    }
  }
}


void AmrelTool::saveSuccessfulSeeds ()
{
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::SUCCESS_SEED_FILE
                    + AmrelConfig::TEXT_SUFFIX);
  std::ofstream output (name.c_str (), std::ios::out);
  std::vector<Pt2i>::const_iterator it;
  int cot = ptset->columnsOfTiles ();
  int rot = ptset->rowsOfTiles ();
  for (int j = 0; j < rot; j++)
  {
    for (int i = 0; i < cot; i++)
    {
      int k = j * cot + ((j % 2 != 0) ? cot - 1 - i : i);
      it = out_sucseeds[k].begin ();
      while (it != out_sucseeds[k].end ())
      {
        Pt2i p1 = *it++;
        Pt2i p2 = *it++;
        output << ptset->xref () + p1.x () * 500 + 25 << " "
               << ptset->yref () + p1.y () * 500 + 25 << " "
               << ptset->xref () + p2.x () * 500 + 25 << " "
               << ptset->yref () + p2.y () * 500 + 25 << std::endl;
      }
    }
  }
  output.close ();
  if (cfg.isVerboseOn ())
    std::cout << "Successful seeds saved in " << name << std::endl;
}


void AmrelTool::exportRoads ()
{
  if (road_sections.empty ()) return;
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::ROAD_FILE
                    + AmrelConfig::SHAPE_SUFFIX);
  std::cout << "Exporting road bounds in " << name << std::endl;
  SHPHandle hSHPHandle = SHPCreate (name.c_str (), SHPT_ARC);
  SHPObject *shp = NULL;
  std::vector<Pt2i> pts;
  std::vector<Pt2i> pts2;
  std::vector<CarriageTrack *>::iterator it = road_sections.begin ();
  while (it != road_sections.end ())
  {
    (*it)->getPosition (pts, pts2, CTRACK_DISP_SCANS, iratio, true);
    int sz = 2 * ((int) (pts.size ())) + 1;
    double *x = new double[sz];
    double *y = new double[sz];
    double *ax = x;
    double *ay = y;
    std::vector<Pt2i>::iterator pit = pts.begin ();
    while (pit != pts.end ())
    {
      *ax++ = ((double) (ptset->xref () + pit->x () * 500 + 25)) / 1000;
      *ay++ = ((double) (ptset->yref () + pit->y () * 500 + 25)) / 1000;
      pit ++;
    }
    if (! pts2.empty ())
    {
      pit = pts2.end ();
      do
      {
        pit --;
        *ax++ = ((double) (ptset->xref () + pit->x () * 500 + 25)) / 1000;
        *ay++ = ((double) (ptset->yref () + pit->y () * 500 + 25)) / 1000;
      }
      while (pit != pts2.begin ());
      *ax = *x;
      *ay = *y;
    }
    shp = SHPCreateObject (SHPT_ARC, -1, 0, NULL, NULL, sz, x, y, NULL, NULL);
    SHPWriteObject (hSHPHandle, -1, shp);
    SHPDestroyObject (shp);
    delete [] x;
    delete [] y;
    pts.clear ();
    pts2.clear ();
    it ++;
  }
  SHPClose (hSHPHandle);
}


void AmrelTool::exportRoadCenters ()
{
  if (road_sections.empty ()) return;
  std::string name (AmrelConfig::RES_DIR + AmrelConfig::LINE_FILE
                    + AmrelConfig::SHAPE_SUFFIX);
  std::cout << "Exporting road centers in " << name << std::endl;
  SHPHandle hSHPHandle = SHPCreate (name.c_str (), SHPT_ARC);
  SHPObject *shp = NULL;
  std::vector<Pt2i> pts;
  std::vector<Pt2i> pts2;
  std::vector<CarriageTrack *>::iterator it = road_sections.begin ();
  while (it != road_sections.end ())
  {
    (*it)->getPosition (pts, pts2, CTRACK_DISP_CENTER, iratio, true);
    int sz = (int) (pts.size ());
    double *x = new double[sz];
    double *y = new double[sz];
    double *ax = x;
    double *ay = y;
    std::vector<Pt2i>::iterator pit = pts.begin ();
    while (pit != pts.end ())
    {
      *ax++ = ((double) (ptset->xref () + pit->x () * 500 + 25)) / 1000;
      *ay++ = ((double) (ptset->yref () + pit->y () * 500 + 25)) / 1000;
      pit ++;
    }
    shp = SHPCreateObject (SHPT_ARC, -1, 0, NULL, NULL, sz, x, y, NULL, NULL);
    SHPWriteObject (hSHPHandle, -1, shp);
    SHPDestroyObject (shp);
    delete [] x;
    delete [] y;
    pts.clear ();
    pts2.clear ();
    it ++;
  }
  SHPClose (hSHPHandle);
}


// DIFF AMREL STD/MULTI


void AmrelTool::processRorpo (int rwidth, int rheight)
{
  if (cfg.isVerboseOn ()) std::cout << "Rorpo ..." << std::endl;
  Image2D<unsigned char> inmap (rwidth, rheight);
  inmap.add_data_from_pointer (dtm_map);
  Image2D<unsigned char> outmap (rwidth, rheight);
  RORPO (outmap, inmap, 30, 1);
  if (rorpo_map == NULL)
    rorpo_map = new unsigned char[rwidth * rheight];
  unsigned char *rmap = rorpo_map;
  unsigned char *rout = outmap.get_pointer ();
  for (int i = 0 ; i < rheight * rwidth; i++) *rmap++ = *rout++;
  if (cfg.isVerboseOn ()) std::cout << "Rorpo OK" << std::endl;
}


void AmrelTool::saveHillImage ()
{
  if (! loadTileSet (true, false)) return;
  Image2D<unsigned char> im (vm_width, vm_height);
  for (int j = 0; j < vm_height; j ++)
    for (int i = 0; i < vm_width; i ++)
    {
      int val = dtm_in->get (i, j, TerrainMap::SHADE_HILL);
      if (val > 255) val = 255;
      else if (val < 0) val = 0;
      im (i, j) = (unsigned char) val;
    }
  write_2D_png_image (im, AmrelConfig::RES_DIR + AmrelConfig::HILL_FILE
                          + AmrelConfig::IM_SUFFIX);
  clear ();
}


void AmrelTool::saveShadingImage ()
{
  int shtype = (cfg.rorpoSkipped () ? TerrainMap::SHADE_EXP_SLOPE
                                    : TerrainMap::SHADE_SLOPE);
  Image2D<unsigned char> im (vm_width, vm_height);
  for (int j = 0; j < vm_height; j ++)
    for (int i = 0; i < vm_width; i ++)
      im (i, j) = (unsigned char) dtm_in->get (i, j, shtype);
  write_2D_png_image (im, AmrelConfig::RES_DIR + AmrelConfig::SLOPE_FILE
                          + AmrelConfig::IM_SUFFIX);
}


void AmrelTool::saveRorpoImage ()
{
  Image2D<unsigned char> im (vm_width, vm_height);
  im.add_data_from_pointer (dtm_map);
  write_2D_png_image (im, AmrelConfig::RES_DIR
                      + AmrelConfig::RORPO_FILE + AmrelConfig::IM_SUFFIX);
}


void AmrelTool::saveSobelImage ()
{
  int w = gmap->getWidth ();
  int h = gmap->getHeight ();
  double *gn = new double[w * h];
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++)
      gn[j * w + i] = gmap->magn (i, j);
  double min = gn[0];
  double max = gn[0];
  for (int i = 1; i < w * h; i++)
  {
    if (max < gn[i]) max = gn[i];
    if (min > gn[i]) min = gn[i];
  }
  Image2D<unsigned char> im (w, h);
  unsigned char *pim = im.get_pointer ();
  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++)
      *pim++ = (unsigned char) ((gn[j * w + i] - min) * 255 / (max - min));
  write_2D_png_image (im, AmrelConfig::RES_DIR + AmrelConfig::SOBEL_FILE
                          + AmrelConfig::IM_SUFFIX);
}


void AmrelTool::saveFbsdImage (int im_w, int im_h)
{
  std::vector<BlurredSegment *> bss = bsdet.getBlurredSegments ();
  if (bss.empty ()) return;

  if (cfg.isFalseColorOn ())
  {
    Image2D<unsigned int> im (im_w, im_h);
    unsigned int *pim = im.get_pointer ();
    for (int i = 0; i < im_w * im_h; i++)
      *pim++ = (unsigned int) (255 + 255 * 256 + 255 * 256 * 256);
    srand (time (NULL));

    if (cfg.isBackDtmOn ())
    {
      if (dtm_in == NULL) loadTileSet (true, false);
      if (dtm_in != NULL)
      {
        pim = im.get_pointer ();
        for (int j = 0; j < im_h; j++)
          for (int i = 0; i < im_w; i++)
            *pim++ = (unsigned int) (dtm_in->get (i, j)) * (257 + 256 * 256);
      }
    }

    std::vector<BlurredSegment *>::iterator it = bss.begin ();
    while (it != bss.end ())
    {
      bool nok = true;
      int red = 0, green = 0, blue = 0;
      while (nok)
      {
        red = rand () % 256;
        green = rand () % 256;
        blue = rand () % 256;
        nok = ((red + green + blue) > 300);     // < 300 si fond noir
      }
      pim = im.get_pointer ();
      std::vector<Pt2i> pts = (*it)->getAllPoints ();
      std::vector<Pt2i>::iterator pit = pts.begin ();
      while (pit != pts.end ())
      {
        *(pim + pit->y () * im_w + pit->x ())
                  = (unsigned int) (red + green * 256 + blue * 256 * 256);
        pit ++;
      }
      it ++;
    }
    write_2D_png_color_image (im,
      AmrelConfig::RES_DIR + AmrelConfig::FBSD_FILE + AmrelConfig::IM_SUFFIX);
  }
  else
  {
    Image2D<unsigned char> im (im_w, im_h);
    unsigned char *pim = im.get_pointer ();
    for (int i = 0; i < im_w * im_h; i++) *pim++ = (unsigned char) 255;

    if (cfg.isBackDtmOn ())
    {
      if (dtm_in == NULL) loadTileSet (true, false);
      if (dtm_in != NULL)
      {
        pim = im.get_pointer ();
        for (int j = 0; j < im_h; j++)
          for (int i = 0; i < im_w; i++)
            *pim++ = (unsigned char) (dtm_in->get (i, j));
      }
    }

    std::vector<BlurredSegment *>::iterator it = bss.begin ();
    while (it != bss.end ())
    {
      pim = im.get_pointer ();
      std::vector<Pt2i> pts = (*it)->getAllPoints ();
      std::vector<Pt2i>::iterator pit = pts.begin ();
      while (pit != pts.end ())
      {
        *(pim + pit->y () * im_w + pit->x ()) = (unsigned char) 0;
        pit ++;
      }
      it ++;
    }
    write_2D_png_image (im,
      AmrelConfig::RES_DIR + AmrelConfig::FBSD_FILE + AmrelConfig::IM_SUFFIX);
  }
}


void AmrelTool::saveSeedsImage ()
{
  int i_w = vm_width, i_h = vm_height;
  if (dtm_in != NULL)
  {
    i_w = dtm_in->tileWidth ();
    i_h = dtm_in->tileHeight ();
  }
  Image2D<unsigned char> im (i_w, i_h);
  unsigned char *pim = im.get_pointer ();
  for (int i = 0; i < i_w * i_h; i++) *pim++ = (unsigned char) 255;
  if (cfg.isBackDtmOn ())
  {
    if (dtm_in == NULL) loadTileSet (true, false);
    if (dtm_in != NULL)
    {
      pim = im.get_pointer ();
      for (int j = 0; j < i_h; j++)
        for (int i = 0; i < i_w; i++)
          *pim++ = (unsigned char) (dtm_in->get (i, j));
    }
  }

  pim = im.get_pointer ();
  if (out_seeds != NULL)
  {
    std::vector<Pt2i>::iterator it;
    int tsize = ptset->columnsOfTiles () * ptset->rowsOfTiles ();
    for (int i = 0; i < tsize; i++)
    {
      it = out_seeds[i].begin ();
      while (it != out_seeds[i].end ())
      {
        Pt2i pt1 = *it++;
        Pt2i pt2 = *it++;
        std::vector<Pt2i> line;
        pt1.draw (line, pt2);
        std::vector<Pt2i>::iterator pit = line.begin ();
        while (pit != line.end ())
        {
          if (pit->x () >= 0 && pit->x () < i_w
              && pit->y () >= 0 && pit->y () < i_h)
            *(pim + (i_h - 1 - pit->y ()) * i_w + pit->x ())
              = (unsigned char) 0;
          pit ++;
        }
      }
    }
  }
  write_2D_png_image (im, AmrelConfig::RES_DIR + AmrelConfig::SEED_FILE
                          + AmrelConfig::IM_SUFFIX);
}


void AmrelTool::saveAsdImage (std::string name)
{
  if (cfg.isBackDtmOn () && dtm_in == NULL) loadTileSet (true, false);
  saveAsdImage (name, cfg.isFalseColorOn (),
                      cfg.isBackDtmOn () ? dtm_in : NULL);
}


void AmrelTool::saveAsdImage (std::string name, bool colorOn, TerrainMap *bg)
{
  unsigned short *map = detection_map->getMap ();
  if (map == NULL) return;
  int mw = detection_map->width ();
  int mh = detection_map->height ();
  int nbroads = detection_map->numberOfRoads ();

  if (colorOn)
  {
    srand (time (NULL));
    unsigned char *red = new unsigned char[nbroads];
    unsigned char *green = new unsigned char[nbroads];
    unsigned char *blue = new unsigned char[nbroads];
    red[0] = (unsigned char) 255;
    green[0] = (unsigned char) 255;
    blue[0] = (unsigned char) 255;
    for (int i = 1; i < nbroads; i ++)
    {
      bool nok = true;
      while (nok)
      {
        red[i] = (unsigned char) (rand () % 256);
        green[i] = (unsigned char) (rand () % 256);
        blue[i] = (unsigned char) (rand () % 256);
        nok = ((int) red[i] + (int) green[i] + (int) blue[i] > 300);
        // < 300 if black background
      }
    }

    Image2D<unsigned int> im (mw, mh);
    unsigned int *pim = im.get_pointer ();
    if (bg != NULL)
    {
      for (int j = 0; j < mh; j++)
        for (int i = 0; i < mw; i++)
          *pim++ = (unsigned int) (bg->get (i, j)) * HUE_GRAY;
    }
    else for (int k = 0; k < mw * mh; k++) *pim++ = HUE_BACK;
    pim = im.get_pointer ();
    for (int i = 0; i < mw * mh; i++)
    {
      if (*map != (unsigned short) 0)
        *pim = (unsigned int) (red[*map] * HUE_RED
                 + green[*map] * HUE_GREEN + blue[*map] * HUE_BLUE);
      pim++;
      map++;
    }
    delete [] red;
    delete [] green;
    delete [] blue;
    write_2D_png_color_image (im, name);
  }
  else
  {
    Image2D<unsigned char> im (mw, mh);
    unsigned char *pim = im.get_pointer ();
    if (bg != NULL)
    {
      for (int j = 0; j < mh - 2; j++)
        for (int i = 0; i < mw - 2; i++)
          *pim++ = (unsigned char) (bg->get (i, j));
    }
    else for (int k = 0; k < mw * mh; k++) *pim++ = (unsigned char) 0;
    pim = im.get_pointer ();
    for (int i = 0; i < mw * mh; i++)
    {
      if (cfg.isColorInversion ())
      {
        if (*map == (unsigned short) 0) *pim = (unsigned char) 255;
      }
      else
      {
        if (*map != (unsigned short) 0) *pim = (unsigned char) 255;
      }
      pim++;
      map++;
    }
    write_2D_png_image (im, name);
  }
}


int AmrelTool::countRoadPixels ()
{
  Image2D<unsigned char> im
    = read_2D_png_image<unsigned char> (
        AmrelConfig::RES_DIR + AmrelConfig::ROAD_FILE + AmrelConfig::IM_SUFFIX);
  int w = im.dimx (), h = im.dimy ();
  unsigned char *pim = im.get_pointer ();
  int nbi = 0, nbr = 0;
  for (int j = 0; j < h; j ++)
    for (int i = 0; i < w; i ++)
    {
      if (*pim++ > 100) nbr ++;
//      if (*pim++ != 0) nbr ++;
      nbi ++;
    }
  if (cfg.isVerboseOn ())
    std::cout << "# road pixels = " << nbr << " / " << nbi << std::endl;
  return nbr;
}


bool AmrelTool::isConnected (std::vector<std::vector<Pt2i> > &pts) const
{
  (void) pts;
  return true;
}


void AmrelTool::adaptTrackDetector ()
{
  if (cfg.tailMinSizeDefined ())
    ctdet->model()->setTailMinSize (cfg.tailMinSize ());
}

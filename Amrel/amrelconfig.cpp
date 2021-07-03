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
#include "amrelconfig.h"
#include "ipttile.h"
#include "terrainmap.h"

#define CONFIG_FILE "config.ini"
#define DET_FILE "steps/autodet.ini"
#define TILE_FILE_DIR "tilesets/"
#define LAST_SET "last_set"
#define LAST_TILES "last_tiles"
#define NVM_DEFAULT_DIR "nvm/"
#define TIL_DEFAULT_DIR "til/"
#define TXT_SUFFIX ".txt"


const std::string AmrelConfig::VERSION = "1.1.1";

const int AmrelConfig::DTM_GRID_SUBDIVISION_FACTOR = 5;
const int AmrelConfig::STEP_ALL = 0;
const int AmrelConfig::STEP_SHADE = 1;
const int AmrelConfig::STEP_RORPO = 2;
const int AmrelConfig::STEP_SOBEL = 3;
const int AmrelConfig::STEP_FBSD = 4;
const int AmrelConfig::STEP_SEEDS = 5;
const int AmrelConfig::STEP_ASD = 6;
const int AmrelConfig::STEP_SAWING = 7;

const int AmrelConfig::DEFAULT_MAX_BS_THICKNESS = 7;
const int AmrelConfig::DEFAULT_MIN_BS_LENGTH = 80;
const int AmrelConfig::DEFAULT_SEED_SHIFT = 24;
const int AmrelConfig::DEFAULT_SEED_WIDTH = 40;


AmrelConfig::AmrelConfig ()
{
  ctdet = NULL;
  nvm_dir = std::string (NVM_DEFAULT_DIR);
  til_dir = std::string (TIL_DEFAULT_DIR);
  dtm_dir = "";
  xyz_dir = "";
  xyz_file = "";
  dtm_import = false;
  xyz_import = false;
  sector_name = std::string (LAST_SET);
  cloud_access = IPtTile::TOP;
  max_bs_thickness = DEFAULT_MAX_BS_THICKNESS;
  min_bs_length = DEFAULT_MIN_BS_LENGTH;
  seed_shift = DEFAULT_SEED_SHIFT;
  seed_width = DEFAULT_SEED_WIDTH;
  pad_size = 0;
  buf_size = 0;
  extraction_step = STEP_ALL;
  connected_mode = true;
  hill_map = false;
  out_map = false;
  back_dtm = false;
  false_color = false;
  seed_check = false;
  verbose = true;

  char cfg_param[100];
  bool reading = true;
  std::ifstream input (CONFIG_FILE, std::ios::in);
  if (input)
  {
    while (reading)
    {
      input >> cfg_param;
      if (input.eof ()) reading = false;
      else
      {
        if (std::string (cfg_param) == std::string ("CLOUD_ACCESS")) 
        {
          std::string clac = getName (input, "CLOUD_ACCESS");
          if (clac == std::string ("TOP")) cloud_access = IPtTile::TOP;
          else if (clac == std::string ("MID")) cloud_access = IPtTile::MID;
          else if (clac == std::string ("ECO")) cloud_access = IPtTile::ECO;
        }
        else if (std::string (cfg_param) == std::string ("MAX_BS_THICKNESS"))
          setMaxBSThickness (getValue (input, "MAX_BS_THICKNESS"));
        else if (std::string (cfg_param) == std::string ("MIN_BS_LENGTH"))
          setMinBSLength (getValue (input, "MIN_BS_LENGTH"));
        else if (std::string (cfg_param) == std::string ("SEED_SHIFT"))
          setSeedShift (getValue (input, "SEED_SHIFT"));
        else if (std::string (cfg_param) == std::string ("SEED_WIDTH"))
          setSeedWidth (getValue (input, "SEED_WIDTH"));
        else if (std::string (cfg_param) == std::string ("PAD_SIZE")) 
          setPadSize (getValue (input, "PAD_SIZE"));
        else if (std::string (cfg_param) == std::string ("BUFFER_SIZE")) 
          setBufferSize (getValue (input, "BUFFER_SIZE"));
        else if (std::string (cfg_param) == std::string ("CONNECTED")) 
          connected_mode = getStatus (input, "CONNECTED");
        else if (std::string (cfg_param) == std::string ("STEP")) 
        {
          std::string clac = getName (input, "STEP");
          if (clac == std::string ("ALL")) extraction_step = STEP_ALL;
          else if (clac == std::string ("SHADING"))
            extraction_step = STEP_SHADE;
          else if (clac == std::string ("RORPO"))
            extraction_step = STEP_RORPO;
          else if (clac == std::string ("SOBEL"))
            extraction_step = STEP_SOBEL;
          else if (clac == std::string ("FBSD"))
            extraction_step = STEP_FBSD;
          else if (clac == std::string ("SEEDS"))
            extraction_step = STEP_SEEDS;
          else if (clac == std::string ("ASD"))
            extraction_step = STEP_ASD;
          else if (clac == std::string ("SAWING"))
            extraction_step = STEP_SAWING;
        }
        else if (std::string (cfg_param) == std::string ("OUT_MAP")) 
          out_map = getStatus (input, "OUT_MAP");
        else if (std::string (cfg_param) == std::string ("BACK_DTM")) 
          back_dtm = getStatus (input, "BACK_DTM");
        else if (std::string (cfg_param) == std::string ("FALSE_COLOR")) 
          false_color = getStatus (input, "FALSE_COLOR");
        else if (std::string (cfg_param) == std::string ("VERBOSE")) 
          verbose = getStatus (input, "VERBOSE");
      }
    }
    input.close ();
  }
}


AmrelConfig::~AmrelConfig ()
{
}


void AmrelConfig::setDetector (CTrackDetector *det)
{
  ctdet = det;
}


std::string AmrelConfig::nvmDir () const
{
  return nvm_dir;
}


std::string AmrelConfig::tilPrefix () const
{
  std::string tfile (til_dir);
  if (cloud_access == IPtTile::TOP)
    tfile += IPtTile::TOP_DIR + IPtTile::TOP_PREFIX;
  else if (cloud_access == IPtTile::MID)
    tfile += IPtTile::MID_DIR + IPtTile::MID_PREFIX;
  else if (cloud_access == IPtTile::ECO)
    tfile += IPtTile::ECO_DIR + IPtTile::ECO_PREFIX;
  return tfile;
}


void AmrelConfig::addTileName (const std::string &name)
{
  tile_names.push_back (name);
}


std::string AmrelConfig::tiles () const
{
  std::string tsname (TILE_FILE_DIR);
  tsname += std::string (LAST_SET) + std::string (TXT_SUFFIX);
  std::ifstream tsf (tsname.c_str (), std::ios::in);
  char text[200];
  tsf >> text;
  tsf.close ();
  return (std::string (TILE_FILE_DIR)
          + std::string (text) + std::string (TXT_SUFFIX));
}


bool AmrelConfig::setTiles ()
{
  bool unspec = true;
  std::string tsname (TILE_FILE_DIR);
  tsname += sector_name + std::string (TXT_SUFFIX);
  std::ifstream tsf (tsname.c_str (), std::ios::in);
  if (tsf.is_open ())
  {
    if (verbose) std::cout << "Using " << tsname << std::endl;
    char text[200];
    tsf >> text;
    if (tsf.eof ()) tsf.close (); // Empty file
    else
    {
      tsf.close ();
      if (sector_name != std::string (LAST_SET))
      {
        std::string deftsname (TILE_FILE_DIR);
        deftsname += std::string (LAST_SET) + std::string (TXT_SUFFIX);
        std::ofstream defts (deftsname.c_str (), std::ios::out);
        defts << sector_name << std::endl;
        defts.close ();
        unspec = false;
      }
      else if (tile_names.empty ()) unspec = false;
    }
  }
  if (unspec)
  {
    if (tile_names.empty ())
    {
      std::cout << "No tile specified in " << sector_name << std::endl;
      return false;
    }
    std::vector<std::string>::iterator it = tile_names.begin ();
    while (it != tile_names.end ())
    {
      std::string nvmn (NVM_DEFAULT_DIR);
      nvmn += *it + TerrainMap::NVM_SUFFIX;
      std::ifstream tf (nvmn.c_str (), std::ios::in);
      if (tf.is_open ()) tf.close ();
      else
      {
        std::cout << "Unknown file " << nvmn << std::endl;
        unspec = false;
      }
      std::string tiln (TIL_DEFAULT_DIR);
      tiln += IPtTile::ECO_DIR + IPtTile::ECO_PREFIX
              + *it + IPtTile::TIL_SUFFIX;
      std::ifstream ecof (tiln.c_str (), std::ios::in);
      if (ecof.is_open ()) ecof.close ();
      else
      {
        tiln = std::string (TIL_DEFAULT_DIR)
               + IPtTile::MID_DIR + IPtTile::MID_PREFIX
               + *it + IPtTile::TIL_SUFFIX;
        std::ifstream midf (tiln.c_str (), std::ios::in);
        if (midf.is_open ()) midf.close ();
        else
        {
          tiln = std::string (TIL_DEFAULT_DIR)
                 + IPtTile::TOP_DIR + IPtTile::TOP_PREFIX
                 + *it + IPtTile::TIL_SUFFIX;
          std::ifstream topf (tiln.c_str (), std::ios::in);
          if (topf.is_open ()) topf.close ();
          else
          {
            std::cout << "Unknown til file for " << *it << std::endl;
            unspec = false;
          }
        }
      }
      it ++;
    }
    if (! unspec) return false;
    if (sector_name == std::string (LAST_SET))
      sector_name = std::string (LAST_TILES);
    std::string deftname (TILE_FILE_DIR);
    deftname += sector_name + std::string (TXT_SUFFIX);
    std::ofstream deft (deftname.c_str (), std::ios::out);
    it = tile_names.begin ();
    while (it != tile_names.end ()) deft << *it++ << std::endl;
    deft.close ();
    std::string deftsname (TILE_FILE_DIR);
    deftsname += std::string (LAST_SET) + std::string (TXT_SUFFIX);
    std::ofstream defts (deftsname.c_str (), std::ios::out);
    defts << sector_name << std::endl;
    defts.close ();
    if (verbose) std::cout << "Using " << sector_name << std::endl;
  }
  return true;
}


std::string AmrelConfig::inputName () const
{
  return sector_name;
}


bool AmrelConfig::setInputName (std::string name)
{
  if (sector_name != std::string (LAST_SET)) return false;
  sector_name = name;
  return true;
}


void AmrelConfig::setMaxBSThickness (int val)
{
  max_bs_thickness = val;
  if (max_bs_thickness < 3) max_bs_thickness = 3;
}


void AmrelConfig::setMinBSLength (int val)
{
  min_bs_length = val;
  if (min_bs_length < 0) min_bs_length = 0;
}


void AmrelConfig::setSeedShift (int val)
{
  seed_shift = val;
  if (seed_shift < 10) seed_shift = 10;
}


void AmrelConfig::setSeedWidth (int val)
{
  seed_width = val;
  if (seed_width < 10) seed_width = 10;
}


bool AmrelConfig::setPadSize (int size)
{
  if (size < 0 || size % 2 == 0)
  {
    std::cout << "Beware : only positive odd values for tile set size !"
              << std::endl;
    return false;
  }
  pad_size = size;
  return true;
}


bool AmrelConfig::setBufferSize (int size)
{
  if (size < 0 || size % 2 == 0)
  {
    std::cout << "Beware : only positive odd values for tile set size !"
              << std::endl;
    return false;
  }
  buf_size = size;
  return true;
}


bool AmrelConfig::getStatus (std::ifstream &input, const char *param)
{
  char cfg_status[100];
  input >> cfg_status;
  if (! input.eof ())
  {
    if (std::string (cfg_status) == std::string ("ON")) return true;
    else if (std::string (cfg_status) == std::string ("OFF")) return false;
  }
  std::cout << "Bad status for " << param << " in " << CONFIG_FILE << std::endl;
  return false;
}


std::string AmrelConfig::getName (std::ifstream &input, const char *param)
{
  char cfg_name[200];
  input >> cfg_name;
  if (! input.eof ()) return (std::string (cfg_name));
  std::cout << "Bad status for " << param << " in " << CONFIG_FILE << std::endl;
  return std::string ("");
}


int AmrelConfig::getValue (std::ifstream &input, const char *param)
{
  int val = 0;
  input >> val;
  if (! input.eof ()) return (val);
  std::cout << "Bad value for " << param << " in " << CONFIG_FILE << std::endl;
  return 0;
}


void AmrelConfig::saveDetectorStatus () const
{
  std::ofstream output (DET_FILE, std::ios::out);
  output << "[AMREL]" << std::endl;
  output << "Version=" << VERSION << std::endl;
  output << "Tile=" << sector_name << std::endl;
  output << "MaxBSThickness=" << max_bs_thickness << std::endl;
  output << "MinBSLength=" << min_bs_length << std::endl;
  output << "SeedShift=" << seed_shift << std::endl;
  output << "SeedWidth=" << seed_width << std::endl;
  output << "PadSize=" << pad_size << std::endl;
  output << "BufferSize=" << buf_size << std::endl;
  output << "Connected=" << (connected_mode ? "true" : "false") << std::endl;
  output << std::endl;

  output << "[ASD]" << std::endl;
  output << "CloudAccess=" << cloud_access << std::endl;
  output << "DetectionMode=1" << std::endl;
  output << std::endl;

  output << "[CTrack]" << std::endl;
  output << "InitialDetection="
    << (ctdet->isInitializationOn () ? "true" : "false") << std::endl;
  output << "DensityCheck="
    << (ctdet->isDensitySensitive () ? "true" : "false") << std::endl;
  output << "DirectionAware="
    << (ctdet->model()->isDeviationPredictionOn () ? "true" : "false")
    << std::endl;
  output << "SlopeAware="
    << (ctdet->model()->isSlopePredictionOn () ? "true" : "false")
    << std::endl;
  output << "PlateauLackTolerance="
    << ctdet->getPlateauLackTolerance () << std::endl;
  output << "PlateauMaxTilt=" << ctdet->model()->bsMaxTilt () << std::endl;
  output << "PlateauMinLength=" << ctdet->model()->minLength () << std::endl;
  output << "PlateauMaxLength=" << ctdet->model()->maxLength () << std::endl;
  output << "MaxThicknessShift="
    << ctdet->model()->thicknessTolerance () << std::endl;
  output << "MaxSlopeShift=" << ctdet->model()->slopeTolerance () << std::endl;
  output << "MaxPositionShift="
    << ctdet->model()->sideShiftTolerance () << std::endl;
  output << "CenterStabilityTest="
    << (ctdet->isShiftLengthPruning () ? "true" : "false") << std::endl;
  output << "MaxCenterShift=" << ctdet->maxShiftLength () << std::endl;
  output << "DetectionRatioTest="
    << (ctdet->isDensityPruning () ? "true" : "false") << std::endl;
  output << "MaxUndetectedRatio=" << ctdet->minDensity () << std::endl;
  output << "TailCompactnessTest="
    << (ctdet->tailPruning () != 0 ? "true" : "false") << std::endl;
  output << "MinTailLength=" << ctdet->model()->tailMinSize () << std::endl;
  output.close ();
}


void AmrelConfig::setDtmDir (const std::string &name)
{
  dtm_dir = name;
  if (name.find_last_of ('/') != name.length () - 1)
    dtm_dir += std::string ("/");
  dtm_import = true;
}


void AmrelConfig::setXyzDir (const std::string &name)
{
  xyz_dir = name;
  if (name.find_last_of ('/') != name.length () - 1)
    xyz_dir += std::string ("/");
  xyz_import = true;
}


void AmrelConfig::setImportFile (const std::string &name)
{
  if (name.substr (name.find_last_of ('.')) == std::string (".asc"))
    dtm_files.push_back (name);
  else xyz_file = name;
}


bool AmrelConfig::importDtm ()
{
  TerrainMap tm;
  std::vector<std::string>::const_iterator it = dtm_files.begin ();
  while (it != dtm_files.end ())
  {
    if (! tm.addDtmFile (dtm_dir + *it))
    {
      std::cout << "Loading of " << (dtm_dir + *it) << " failed" << std::endl;
      return false;
    }
    it ++;
  }
  if (! tm.create ())
  {
    std::cout << "Tile set assembling failed" << std::endl;
    return false;
  }
  std::string tn (tile_names.empty () ?
                  dtm_files[0].substr (0, dtm_files[0].find_last_of ('.')) :
                  tile_names[0]);
  tm.saveFirstNormalMap (std::string ("nvm/") + tn + std::string (".nvm"));
  if (verbose) std::cout << "Saved " << std::string ("nvm/") << tn
                         << std::string (".nvm") << std::endl;
  return true;
}


bool AmrelConfig::importXyz ()
{
  std::string tn (tile_names.empty () ?
                  xyz_file.substr (0, xyz_file.find_last_of ('.')) :
                  tile_names[0]);
  TerrainMap tm;
  if (! tm.loadNormalMapInfo (std::string ("nvm/")
                              + tn + std::string (".nvm")))
  {
    std::cout << "Can't read tile features in "
              << std::string ("nvm/") << tn << std::string (".nvm")
              << " file" << std::endl;
    return false;
  }
  IPtTile tile ((tm.tileHeight () * DTM_GRID_SUBDIVISION_FACTOR)
                / cloud_access,
                (tm.tileWidth () * DTM_GRID_SUBDIVISION_FACTOR)
                / cloud_access);
  tile.setArea ((int64_t) (tm.xMin () * IPtTile::XYZ_UNIT + 0.5f),
                (int64_t) (tm.yMin () * IPtTile::XYZ_UNIT + 0.5f),
                (int64_t) 0,
                (int) ((tm.cellSize () * IPtTile::XYZ_UNIT * cloud_access)
                       / DTM_GRID_SUBDIVISION_FACTOR + 0.5));
  if (! tile.loadXYZFile (xyz_dir + xyz_file, cloud_access))
  {
    std::cout << "Can't read " << xyz_dir << xyz_file << " file" << std::endl;
    return false;
  }
  std::string sname ("til/");
  if (cloud_access == IPtTile::TOP)
    sname += std::string ("top/top_") + tn + std::string (".til");
  else if (cloud_access == IPtTile::MID)
    sname += std::string ("mid/mid_") + tn + std::string (".til");
  else if (cloud_access == IPtTile::ECO)
    sname += std::string ("eco/eco_") + tn + std::string (".til");
  tile.save (sname);

  return true;
}


bool AmrelConfig::createAltXyz (const std::string &name)
{
  if (cloud_access == IPtTile::ECO)
  {
    std::string oldname (til_dir);
    oldname += IPtTile::MID_DIR + IPtTile::MID_PREFIX
               + name + IPtTile::TIL_SUFFIX;
    IPtTile *oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += IPtTile::ECO_DIR + IPtTile::ECO_PREFIX
                 + name + IPtTile::TIL_SUFFIX;
      IPtTile *newt = new IPtTile (newname);
      newt->setSize ((oldt->countOfColumns () * IPtTile::MID) / IPtTile::ECO,
                     (oldt->countOfRows () * IPtTile::MID) / IPtTile::ECO);
      newt->setArea (oldt->xref (), oldt->yref (), oldt->top (),
                     IPtTile::MIN_CELL_SIZE * IPtTile::ECO);
      newt->setPoints (*oldt);
      newt->save (newname);
      delete oldt;
      delete newt;
      return true;
    }
    oldname = std::string (til_dir);
    oldname += IPtTile::TOP_DIR + IPtTile::TOP_PREFIX
               + name + IPtTile::TIL_SUFFIX;
    oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += IPtTile::ECO_DIR + IPtTile::ECO_PREFIX
                 + name + IPtTile::TIL_SUFFIX;
      IPtTile *newt = new IPtTile (newname);
      newt->setSize ((oldt->countOfColumns () * IPtTile::TOP) / IPtTile::ECO,
                     (oldt->countOfRows () * IPtTile::TOP) / IPtTile::ECO);
      newt->setArea (oldt->xref (), oldt->yref (), oldt->top (),
                     IPtTile::MIN_CELL_SIZE * IPtTile::ECO);
      newt->setPoints (*oldt);
      newt->save (newname);
      delete oldt;
      delete newt;
      return true;
    }
    return false;
  }
  else if (cloud_access == IPtTile::MID)
  {
    std::string oldname (til_dir);
    oldname += IPtTile::TOP_DIR + IPtTile::TOP_PREFIX
               + name + IPtTile::TIL_SUFFIX;
    IPtTile *oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += IPtTile::MID_DIR + IPtTile::MID_PREFIX
                 + name + IPtTile::TIL_SUFFIX;
      IPtTile *newt = new IPtTile (newname);
      newt->setSize ((oldt->countOfColumns () * IPtTile::TOP) / IPtTile::MID,
                     (oldt->countOfRows () * IPtTile::TOP) / IPtTile::MID);
      newt->setArea (oldt->xref (), oldt->yref (), oldt->top (),
                     IPtTile::MIN_CELL_SIZE * IPtTile::MID);
      newt->setPoints (*oldt);
      newt->save (newname);
      delete oldt;
      delete newt;
      return true;
    }
    oldname = std::string (til_dir);
    oldname += IPtTile::ECO_DIR + IPtTile::ECO_PREFIX
               + name + IPtTile::TIL_SUFFIX;
    oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += IPtTile::MID_DIR + IPtTile::MID_PREFIX
                 + name + IPtTile::TIL_SUFFIX;
      IPtTile *newt = new IPtTile (newname);
      newt->setSize ((oldt->countOfColumns () * IPtTile::ECO) / IPtTile::MID,
                     (oldt->countOfRows () * IPtTile::ECO) / IPtTile::MID);
      newt->setArea (oldt->xref (), oldt->yref (), oldt->top (),
                     IPtTile::MIN_CELL_SIZE * IPtTile::MID);
      newt->setPoints (*oldt);
      newt->save (newname);
      delete oldt;
      delete newt;
      return true;
    }
    return false;
  }
  else
  {
    std::string oldname (til_dir);
    oldname += IPtTile::MID_DIR + IPtTile::MID_PREFIX
               + name + IPtTile::TIL_SUFFIX;
    IPtTile *oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += IPtTile::TOP_DIR + IPtTile::TOP_PREFIX
                 + name + IPtTile::TIL_SUFFIX;
      IPtTile *newt = new IPtTile (newname);
      newt->setSize ((oldt->countOfColumns () * IPtTile::MID) / IPtTile::TOP,
                     (oldt->countOfRows () * IPtTile::MID) / IPtTile::TOP);
      newt->setArea (oldt->xref (), oldt->yref (), oldt->top (),
                     IPtTile::MIN_CELL_SIZE * IPtTile::TOP);
      newt->setPoints (*oldt);
      newt->save (newname);
      delete oldt;
      delete newt;
      return true;
    }
    oldname = std::string (til_dir);
    oldname += IPtTile::ECO_DIR + IPtTile::ECO_PREFIX
               + name + IPtTile::TIL_SUFFIX;
    oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += IPtTile::TOP_DIR + IPtTile::TOP_PREFIX
                 + name + IPtTile::TIL_SUFFIX;
      IPtTile *newt = new IPtTile (newname);
      newt->setSize ((oldt->countOfColumns () * IPtTile::ECO) / IPtTile::TOP,
                     (oldt->countOfRows () * IPtTile::ECO) / IPtTile::TOP);
      newt->setArea (oldt->xref (), oldt->yref (), oldt->top (),
                     IPtTile::MIN_CELL_SIZE * IPtTile::TOP);
      newt->setPoints (*oldt);
      newt->save (newname);
      delete oldt;
      delete newt;
      return true;
    }
    return false;
  }
}

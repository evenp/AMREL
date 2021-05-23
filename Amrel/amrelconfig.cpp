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
#define LAST_SET "last_set.txt"
#define LAST_TILE "last_tile.txt"
#define NVM_DEFAULT_DIR "nvm/"
#define TIL_DEFAULT_DIR "til/"
#define NVM_SUFFIX ".nvm"
#define TIL_SUFFIX ".til"


const std::string AmrelConfig::VERSION = "1.1.0";

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
  spec_name = std::string (LAST_SET);
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


std::string AmrelConfig::nvmSuffix () const
{
  return (std::string (NVM_SUFFIX));
}


std::string AmrelConfig::tilPrefix () const
{
  std::string tfile (til_dir);
  if (cloud_access == IPtTile::TOP)
    tfile += std::string (TILE_ACCESS_DIR_TOP)
             + std::string (TILE_ACCESS_PREF_TOP);
  else if (cloud_access == IPtTile::MID)
    tfile += std::string (TILE_ACCESS_DIR_MID)
             + std::string (TILE_ACCESS_PREF_MID);
  else if (cloud_access == IPtTile::ECO)
    tfile += std::string (TILE_ACCESS_DIR_ECO)
             + std::string (TILE_ACCESS_PREF_ECO);
  return tfile;
}


std::string AmrelConfig::tilSuffix () const
{
  return (std::string (TIL_SUFFIX));
}


void AmrelConfig::addToSet (const std::string &name)
{
  added_tiles.push_back (name);
}


void AmrelConfig::completeTileSet ()
{
  if (spec_name != std::string (LAST_SET))
  {
    std::string deftsname (TILE_FILE_DIR);
    deftsname += std::string (spec_name);
    std::ofstream defts (deftsname.c_str (), std::ios::app);
    if (defts.is_open ())
    {
      std::vector<std::string>::iterator it = added_tiles.begin ();
      while (it != added_tiles.end ()) defts << *it++ << std::endl;
      defts.close ();
    }
    else std::cout << deftsname << " can't be opened" << std::endl;
  }
  else std::cout << "Tile set name missing to add tiles" << std::endl;
}


std::string AmrelConfig::tiles () const
{
  std::string tsname (TILE_FILE_DIR);
  tsname += std::string (LAST_SET);
  std::ifstream tsf (tsname.c_str (), std::ios::in);
  char text[200];
  tsf >> text;
  tsf.close ();
  return (TILE_FILE_DIR + std::string (text));
}


bool AmrelConfig::setTiles ()
{
  if (! added_tiles.empty ()) completeTileSet ();
 
  std::string tsname (TILE_FILE_DIR);
  tsname += spec_name;
  std::ifstream tsf (tsname.c_str (), std::ios::in);
  if (tsf.is_open ())
  {
    if (verbose) std::cout << "Using " << tsname << std::endl;
    char text[200];
    tsf >> text;
    if (tsf.eof ()) 
    {
      std::cout << "No tile specified in " << spec_name << std::endl;
      return false;
    }
    tsf.close ();
    if (spec_name != std::string (LAST_SET))
    {
      std::string deftsname (TILE_FILE_DIR);
      deftsname += std::string (LAST_SET);
      std::ofstream defts (deftsname.c_str (), std::ios::out);
      defts << spec_name << std::endl;
      defts.close ();
    }
  }
  else
  {
    if (spec_name == std::string (LAST_SET))
    {
      std::cout << "No tile has been specified yet" << std::endl;
      return false;
    }
    std::string nvmname (nvm_dir);
    nvmname += spec_name + NVM_SUFFIX;
    std::ifstream tf (nvmname.c_str (), std::ios::in);
    if (tf.is_open ())
    {
      tf.close ();
      std::string deftname (TILE_FILE_DIR);
      deftname += LAST_TILE;
      std::ofstream deft (deftname.c_str (), std::ios::out);
      deft << spec_name << std::endl;
      deft.close ();
      std::string deftsname (TILE_FILE_DIR);
      deftsname += std::string (LAST_SET);
      std::ofstream defts (deftsname.c_str (), std::ios::out);
      defts << LAST_TILE << std::endl;
      defts.close ();
      if (verbose) std::cout << "Using " << spec_name << std::endl;
    }
    else
    {
      std::cout << spec_name << " not found" << std::endl;
      return false;
    }
  }
  return true;
}


std::string AmrelConfig::inputName () const
{
  return spec_name;
}


bool AmrelConfig::setInputName (std::string name)
{
  if (spec_name != std::string (LAST_SET)) return false;
  spec_name = name;
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
  output << "Tile=" << spec_name << std::endl;
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
  dtm_import = true;
}


void AmrelConfig::setXyzDir (const std::string &name)
{
  xyz_dir = name;
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
  if (spec_name == std::string (LAST_SET))
    spec_name = dtm_files[0].substr (0, dtm_files[0].find_last_of ('.'));
  tm.saveFirstNormalMap (std::string ("nvm/")
                         + spec_name + std::string (".nvm"));
  if (verbose) std::cout << "Saved " << std::string ("nvm/") << spec_name
                         << std::string (".nvm") << std::endl;
  return true;
}


bool AmrelConfig::importXyz ()
{
  if (spec_name == std::string (LAST_SET))
    spec_name = xyz_file.substr (0, xyz_file.find_last_of ('.'));

  TerrainMap tm;
  if (! tm.loadNormalMapInfo (std::string ("nvm/")
                              + spec_name + std::string (".nvm")))
  {
    std::cout << "Can't read tile features in "
              << std::string ("nvm/") << spec_name << std::string (".nvm")
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
    sname += std::string ("top/top_") + spec_name + std::string (".til");
  else if (cloud_access == IPtTile::MID)
    sname += std::string ("mid/mid_") + spec_name + std::string (".til");
  else if (cloud_access == IPtTile::ECO)
    sname += std::string ("eco/eco_") + spec_name + std::string (".til");
  tile.save (sname);

  return true;
}


bool AmrelConfig::createAltXyz (const std::string &name)
{
  if (cloud_access == IPtTile::ECO)
  {
    std::string oldname (til_dir);
    oldname += std::string (TILE_ACCESS_DIR_MID)
               + std::string (TILE_ACCESS_PREF_MID)
               + name + std::string (TIL_SUFFIX);
    IPtTile *oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += std::string (TILE_ACCESS_DIR_ECO)
                 + std::string (TILE_ACCESS_PREF_ECO)
                 + name + std::string (TIL_SUFFIX);
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
    oldname += std::string (TILE_ACCESS_DIR_TOP)
               + std::string (TILE_ACCESS_PREF_TOP)
               + name + std::string (TIL_SUFFIX);
    oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += std::string (TILE_ACCESS_DIR_ECO)
                 + std::string (TILE_ACCESS_PREF_ECO)
                 + name + std::string (TIL_SUFFIX);
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
    oldname += std::string (TILE_ACCESS_DIR_TOP)
               + std::string (TILE_ACCESS_PREF_TOP)
               + name + std::string (TIL_SUFFIX);
    IPtTile *oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += std::string (TILE_ACCESS_DIR_MID)
                 + std::string (TILE_ACCESS_PREF_MID)
                 + name + std::string (TIL_SUFFIX);
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
    oldname += std::string (TILE_ACCESS_DIR_ECO)
               + std::string (TILE_ACCESS_PREF_ECO)
               + name + std::string (TIL_SUFFIX);
    oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += std::string (TILE_ACCESS_DIR_MID)
                 + std::string (TILE_ACCESS_PREF_MID)
                 + name + std::string (TIL_SUFFIX);
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
    oldname += std::string (TILE_ACCESS_DIR_MID)
               + std::string (TILE_ACCESS_PREF_MID)
               + name + std::string (TIL_SUFFIX);
    IPtTile *oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += std::string (TILE_ACCESS_DIR_TOP)
                 + std::string (TILE_ACCESS_PREF_TOP)
                 + name + std::string (TIL_SUFFIX);
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
    oldname += std::string (TILE_ACCESS_DIR_ECO)
               + std::string (TILE_ACCESS_PREF_ECO)
               + name + std::string (TIL_SUFFIX);
    oldt = new IPtTile (oldname);
    if (oldt->load ())
    {
      if (verbose) std::cout << "Creating from " << oldname << std::endl;
      std::string newname (til_dir);
      newname += std::string (TILE_ACCESS_DIR_TOP)
                 + std::string (TILE_ACCESS_PREF_TOP)
                 + name + std::string (TIL_SUFFIX);
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

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

#ifndef AMREL_CONFIG_H
#define AMREL_CONFIG_H

#include "ctrackdetector.h"


/** 
 * @class AmrelConfig amrelconfig.h
 * \brief Configuration for automatic mountain road extraction from LiDAR data.
 */
class AmrelConfig
{
public:

  /** Version number */
  static const std::string VERSION;

  /** DTM grid subdivision factor for 3D points loading. */
  static const int DTM_GRID_SUBDIVISION_FACTOR;
  /** Road detection step: all (complete automatic mode). */
  static const int STEP_ALL;
  /** Road detection step: DTM shading. */
  static const int STEP_SHADE;
  /** Road detection step: elongated shape enhancement. */
  static const int STEP_RORPO;
  /** Road detection step: gradient extraction. */
  static const int STEP_SOBEL;
  /** Road detection step: blurred segment detection. */
  static const int STEP_FBSD;
  /** Road detection step: seeds generation. */
  static const int STEP_SEEDS;
  /** Road detection step: tracks detection. */
  static const int STEP_ASD;
  /** Road detection step: from shading up to seeds generation. */
  static const int STEP_SAWING;

  /** Name of output directory. */
  static const std::string RES_DIR;
  /** Name of tile set directory. */
  static const std::string TSET_DIR;
  /** Name of default terrain map directory. */
  static const std::string NVM_DEFAULT_DIR;
  /** Name of default point tile directory. */
  static const std::string TIL_DEFAULT_DIR;

  /** Name of AMREL configuration file. */
  static const std::string CONFIG_FILE;
  /** Name of detector parameters file. */
  static const std::string DETECTOR_FILE;
  /** Name of last set file. */
  static const std::string LAST_SET_FILE;
  /** Name of last tiles file. */
  static const std::string LAST_TILES_FILE;
  /** Name of performance result file. */
  static const std::string PERF_FILE;

  /** Name of hill-shading file. */
  static const std::string HILL_FILE;
  /** Name of slope-shading file. */
  static const std::string SLOPE_FILE;
  /** Name of RORPO file. */
  static const std::string RORPO_FILE;
  /** Name of Sobel file. */
  static const std::string SOBEL_FILE;
  /** Name of FBSD digital straight segment file. */
  static const std::string FBSD_FILE;
  /** Name of seed file. */
  static const std::string SEED_FILE;
  /** Name of successful seed file. */
  static const std::string SUCCESS_SEED_FILE;
  /** Name of output road file. */
  static const std::string ROAD_FILE;
  /** Name of output road line file. */
  static const std::string LINE_FILE;

  /** AMREL file suffix. */
  static const std::string AMREL_SUFFIX;
  /** Configuration file suffix. */
  static const std::string INI_SUFFIX;
  /** Seed file suffix. */
  static const std::string SEED_SUFFIX;
  /** FBSD file suffix. */
  static const std::string FBSD_SUFFIX;
  /** Shape file suffix. */
  static const std::string SHAPE_SUFFIX;
  /** Map file suffix. */
  static const std::string MAP_SUFFIX;
  /** Image file suffix. */
  static const std::string IM_SUFFIX;
  /** Text file suffix (for tests, parameters, ...). */
  static const std::string TEXT_SUFFIX;


  /**
   * \brief Creates a configuration for AMREL tool.
   */
  AmrelConfig ();

  /**
   * \brief Deletes the configuration.
   */
  ~AmrelConfig ();

  /**
   * \brief Sets the carriage track detector.
   */
  void setDetector (CTrackDetector *det);

  /**
   * \brief Returns NVM directory name.
   */
  std::string nvmDir () const;

  /**
   * \brief Returns TIL directory name.
   */
  std::string tilPrefix () const;

  /**
   * \brief Completes current tile set with new tiles.
   */
  void completeTileSet ();

  /**
   * \brief Declares a tile to be processed.
   * @param name Added tile name.
   */
  void addTileName (const std::string &name);

  /**
   * \brief Returns the name of the tile set file.
   */
  std::string tiles () const;

  /**
   * \brief Prepares a tile set with tiles to be processed.
   * Returns whether tile set specified is consistent.
   */
  bool setTiles ();

  /**
   * \brief Returns the name of the tile or tile set to process.
   */
  std::string inputName () const;

  /**
   * \brief Sets the name of the tile or tile set to process.
   * Returns if the name is accepted (no conflict with previous input).
   * @param name Input name.
   */
  bool setInputName (std::string name);

  /**
   * \brief Sets cloud access level.
   * @param status New level value.
   */
  inline void setCloudAccess (int val) { cloud_access = val; }

  /**
   * Gets the assigned thickness of blurred segments.
   */
  inline int maxBSThickness () const { return max_bs_thickness; }

  /**
   * Sets the assigned length of blurred segments.
   */
  void setMaxBSThickness (int val);

  /**
   * Gets the minimal length of accepted blurred segments.
   */
  inline int minBSLength () const { return min_bs_length; }

  /**
   * Sets the minimal length of accepted blurred segments.
   */
  void setMinBSLength (int val);

  /**
   * Gets the distance between successive seeds.
   */
  inline int seedShift () const { return seed_shift; }

  /**
   * Sets the distance between successive seeds.
   */
  void setSeedShift (int val);

  /**
   * Gets the width of seeds.
   */
  inline int seedWidth () const { return seed_width; }

  /**
   * Sets the width of seeds.
   */
  void setSeedWidth (int val);

  /**
   * Inquires if half-size seed production is required.
   */
  inline bool isHalfSizeSeedsOn () const { return half_size; }

  /**
   * Requires half-size seed production.
   */
  void setHalfSizeSeeds ();

  /**
   * \brief Returns pad size for seed generation.
   */
  inline int padSize () const { return pad_size; }

  /**
   * \brief Sets pad size for seed generation.
   * Returns if new size is accepted.
   * @param size New size.
   */
  bool setPadSize (int size);

  /**
   * \brief Returns tile set size for road extraction.
   */
  inline int bufferSize () const { return buf_size; }

  /**
   * \brief Sets tile set size for road extraction.
   * Returns if new size is accepted.
   * @param size New size.
   */
  bool setBufferSize (int size);

  /**
   * \brief Returns tail pruning minimal size.
   */
  inline int tailMinSize () const { return tail_min_size; }

  /**
   * \brief Returns whether a specific tail pruning minimal size is defined.
   */
  inline bool tailMinSizeDefined () const { return (tail_min_size != -1); }

  /**
   * \brief Sets tail pruning minimal size.
   * Returns if new size is accepted.
   * @param size New minimal tail size allowed.
   */
  bool setTailMinSize (int size);

  /**
   * \brief Returns road extraction step to be processed.
   */
  inline int step () const { return extraction_step; }

  /**
   * \brief Sets road extraction step to be processed.
   * @param step Extraction step.
   */
  inline void setStep (int step) { extraction_step = step; }

  /**
   * \brief Returns road connection status.
   */
  inline bool isConnectedOn () const { return connected_mode; }

  /**
   * \brief Sets road connection status.
   * @param status New status value.
   */
  inline void setConnected (bool status) { connected_mode = status; }

  /**
   * \brief Returns hill-shaded map display status.
   */
  inline bool isHillMapOn () const { return hill_map; }

  /**
   * \brief Sets hill-shaded map display status.
   * @param status New status value.
   */
  inline void setHillMap (bool status) { hill_map = status; }

  /**
   * \brief Returns map output status.
   */
  inline bool isOutMapOn () const { return out_map; }

  /**
   * \brief Sets map output status.
   * @param status New status value.
   */
  inline void setOutMap (bool status) { out_map = status; }

  /**
   * \brief Returns DTM background status.
   */
  inline bool isBackDtmOn () const { return back_dtm; }

  /**
   * \brief Sets DTM background status.
   * @param status New status value.
   */
  inline void setBackDtm (bool status) { back_dtm = status; }

  /**
   * \brief Returns false color output status.
   */
  inline bool isFalseColorOn () const { return false_color; }

  /**
   * \brief Sets false color output status.
   * @param status New status value.
   */
  inline void setFalseColor (bool status) { false_color = status; }

  /**
   * \brief Returns color inversion status.
   */
  inline bool isColorInversion () const { return inv_color; }

  /**
   * \brief Sets color inversion status.
   * @param status New status value.
   */
  inline void setColorInversion (bool status) { inv_color = status; }

  /**
   * \brief Returns seed check modality status.
   */
  inline bool isSeedCheckOn () const { return seed_check; }

  /**
   * \brief Sets seed check modality status.
   * @param status New status value.
   */
  inline void setSeedCheck (bool status) { seed_check = status; }

  /**
   * \brief Returns text information output status.
   */
  inline bool isVerboseOn () const { return verbose; }

  /**
   * \brief Sets text information output status.
   * @param status New status value.
   */
  inline void setVerbose (bool status) { verbose = status; }

  /**
   * \brief Returns road export modality status.
   */
  inline bool isExportOn () const { return (exporting != 0); }

  /**
   * \brief Returns road bound export modality status.
   */
  inline bool isExportBoundsOn () const { return (exporting == 2); }

  /**
   * \brief Sets road export modality status.
   * @param status New status value (0 = none, 1 = center, 2 = bounds).
   */
  inline void setExport (int status) { exporting = status; }

  /**
   * \brief Registers the detector status in default file.
   */
  void saveDetectorStatus () const;

  /**
   * \brief Returns DTM import request status.
   */
  inline bool isDtmImportOn () const { return dtm_import; }

  /**
   * \brief Sets path to DTM files.
   * @param name Name of the path.
   */
  void setDtmDir (const std::string &name);

  /**
   * \brief Returns XYZ import request status.
   */
  inline bool isXyzImportOn () const { return xyz_import; }

  /**
   * \brief Sets path to XYZ files.
   * @param name Name of the path.
   */
  void setXyzDir (const std::string &name);

  /**
   * \brief Sets DTM or XYZ import file name.
   * @param name Name of the file to be imported.
   */
  void setImportFile (const std::string &name);

  /**
   * \brief Imports a DTM tile file.
   * Returns import success status.
   */
  bool importDtm ();

  /**
   * \brief Imports a point tile file.
   * Returns import success status.
   */
  bool importXyz ();

  /**
   * \brief Tries to create a point set file for a specific cloud access.
   * Returns creation success status.
   * @param name Point set name.
   */
  bool createAltXyz (const std::string &name);


private:

  /** Default value for assigned thickness of blurred segments. */
  static const int DEFAULT_MAX_BS_THICKNESS;
  /** Default value for minimal length of retained blurred segments. */
  static const int DEFAULT_MIN_BS_LENGTH;
  /** Default value for space between successive seeds. */
  static const int DEFAULT_SEED_SHIFT;
  /** Default value for width of seeds. */
  static const int DEFAULT_SEED_WIDTH;


  /** Pointer to used carriage track detector. */
  CTrackDetector *ctdet;

  /** NVM-formatted DTM directory name. */
  std::string nvm_dir;
  /** TIL-formatted point tile directory name. */
  std::string til_dir;
  /** Name of a specified tile set to process. */
  std::string sector_name;
  /** Names of specified tiles to process. */
  std::vector<std::string> tile_names;
  /** Cloud access level. */
  int cloud_access;
  /** Blurred segment assigned thickness. */
  int max_bs_thickness;
  /** Minimal length of retained blurred segments. */
  int min_bs_length;
  /** Distance between successive seeds. */
  int seed_shift;
  /** Half-width of seeds. */
  int seed_width;
  /** Half-size seed production status. */
  bool half_size;
  /** Pad size for seed generation. */
  int pad_size;
  /** Tile set size for road extraction. */
  int buf_size;
  /** Tail pruning minimal size. */
  int tail_min_size;

  /** Road extraction step to be processed. */
  int extraction_step;
  /** Road connection status. */
  bool connected_mode;
  /** Hill-shaded map production status. */
  bool hill_map;
  /** Output map production status. */
  bool out_map;
  /** DTM background status. */
  bool back_dtm;
  /** False color output status. */
  bool false_color;
  /** Color inversion status. */
  bool inv_color;
  /** Seed check modality status. */
  bool seed_check;
  /** Text information output status. */
  bool verbose;
  /** Road export modality (0 = no export, 1 = centerline, 2 = bounds). */
  int exporting;

  /** DTM import request status. */
  bool dtm_import;
  /** Path to DTM files. */
  std::string dtm_dir;
  /** Name of DYM central tile and its neighbours. */
  std::vector<std::string> dtm_files;
  /** XYZ import request status. */
  bool xyz_import;
  /** Path to XYZ files. */
  std::string xyz_dir;
  /** XYZ file to be imported. */
  std::string xyz_file;


  /**
   * \brief Reads a parameter status in configuration file.
   * @param input Input file.
   * @param param Parameter name.
   */
  bool getStatus (std::ifstream &input, const char *param);

  /**
   * \brief Reads a name in configuration file.
   * @param input Input file.
   * @param param Parameter name.
   */
  std::string getName (std::ifstream &input, const char *param);

  /**
   * \brief Reads an integer value in configuration file.
   * @param input Input file.
   * @param param Parameter name.
   */
  int getValue (std::ifstream &input, const char *param);

};
#endif

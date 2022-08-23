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

#ifndef AMREL_TOOL_H
#define AMREL_TOOL_H

#include "image.hpp"
#include "terrainmap.h"
#include "vmap.h"
#include "bsdetector.h"
#include "amrelconfig.h"


/** 
 * @class AmrelTool amreltool.h
 * \brief Automatic mountain road extractor from LiDAR data.
 */
class AmrelTool
{

public:

  /** Nominal value for plateau lack tolerance when detecting. */
  static const int NOMINAL_PLATEAU_LACK_TOLERANCE;
  /** Nominal value for plateau maximal tilt when detecting. */
  static const int NOMINAL_PLATEAU_MAX_TILT;
  /** Nominal value for plateau maximal shift tolerance when detecting. */
  static const float NOMINAL_MAX_SHIFT_LENGTH;
  /** Nominal value for plateau minimal length when detecting. */
  static const float NOMINAL_PLATEAU_MIN_LENGTH;
  /** Nominal value for plateau thickness tolerance when detecting. */
  static const float NOMINAL_PLATEAU_THICKNESS_TOLERANCE;
  /** Nominal value for track slope tolerance when detecting. */
  static const float NOMINAL_SLOPE_TOLERANCE;
  /** Nominal value for plateau side shift tolerance when detecting. */
  static const float NOMINAL_SIDE_SHIFT_TOLERANCE;


  /**
   * \brief Creates an AMREL tool.
   */
  AmrelTool ();

  /**
   * \brief Deletes the extractor.
   */
  ~AmrelTool ();

  /**
   * \brief Clears the detector tile structures.
   */
  void clear ();

  /**
   * \brief Clears loaded points only.
   */
  void clearPoints ();

  /**
   * \brief Clears DTM normal vector map only.
   */
  void clearDtm ();

  /**
   * Releases shaded DTM map.
   */
  void clearShading ();

  /**
   * Releases Rorpo map.
   */
  void clearRorpo ();

  /**
   * Releases Sobel map.
   */
  void clearSobel ();

  /**
   * Releases Fbsd segments.
   */
  void clearFbsd ();

  /**
   * Releases generated seeds.
   */
  void clearSeeds ();

  /**
   * Release extracted tracks and successful seeds.
   */
  void clearAsd ();

  /**
   * \brief Returns the virtual map width (global DTM).
   */
  inline int vmWidth () const { return vm_width; }

  /**
   * \brief Returns the virtual map height (global DTM).
   */
  inline int vmHeight () const { return vm_height; }

  /**
   * Returns the tool configuration.
   */
  inline AmrelConfig *config () { return &cfg; }

  /**
   * Associates a track detector to the automatic one.
   */
  void addTrackDetector ();

  /**
   * Edits road detector features.
   */
  void checkDetector ();

  /**
   * Returns whether a Dtm map is already loaded.
   */
  inline bool isDtmLoaded () const { return (dtm_in != NULL); }

  /**
   * Loads the tile set to process.
   * Returns load success status.
   * @param dtm_on Indicates whether DTM should be loaded.
   * @param pts_on Indicates whether raw points should be loaded.
   */
  bool loadTileSet (bool dtm_on, bool pts_on);

  /**
   * Loads a cloud of points.
   * Returns load success status.
   */
  bool loadPoints ();

  /**
   * Runs the automatic road detector.
   */
  void run ();

  /**
   * Detects roads on loaded image : step 1 = Slope shading.
   */
  void processShading ();

  /**
   * Detects roads on loaded image : step 2 = RORPO image filtering.
   */
  void processRorpo ();

  /**
   * Detects roads on loaded image : step 3 = Sobel gradient map construction.
   * @param w Map width.
   * @param h Map height.
   */
  void processSobel (int w, int h);

  /**
   * Detects roads on loaded image : step 4 = FBSD straight segments detection.
   */
  void processFbsd ();

  /**
   * Detects roads on loaded image : step 5 = Seed production.
   * @param kref Lower left tile reference (-1 if pad is not used).
   */
  void processSeeds (int kref = -1);

  /**
   * Detects roads on loaded image: step 6 = road extraction from seeds.
   * Returns if succeeded (false when tiles cannot be loaded).
   */
  bool processAsd ();

  /**
   * Detects roads on loaded image : steps 1 to 5 = generating seeds.
   * Returns if succeeded (false when tiles can not be loaded).
   */
  bool processSawing ();

  /**
   * Saves DTM shaded map in steps/shade.map file.
   */
  bool saveShadingMap ();

  /**
   * Loads DTM shaded map from steps/shade.map file to run RORPO.
   */
  bool loadShadingMap ();

  /**
   * Saves Rorpo map in steps/rorpo.map file.
   */
  bool saveRorpoMap ();

  /**
   * Loads Rorpo map from steps/rorpo.map file to run Sobel and FBSD.
   */
  bool loadRorpoMap ();

  /**
   * Saves gradient map in steps/sobel.map file.
   */
  bool saveSobelMap ();

  /**
   * Loads gradient map from steps/sobel.map file to run FBSD.
   */
  bool loadSobelMap ();

  /**
   * Saves digital straight segments in steps/fbsd.dss file.
   */
  bool saveFbsdSegments ();

  /**
   * Loads digital straight segments from steps/fbsd.dss file to produce seeds.
   */
  bool loadFbsdSegments ();

  /**
   * Saves blurred-segment-based seeds in steps/seeds.pts file.
   */
  bool saveSeeds ();

  /**
   * Loads blurred-segment-based seeds from steps/seeds.pts file to run ASD.
   */
  bool loadSeeds ();

  /**
   * Edits detected seeds features.
   */
  void checkSeeds ();

  /**
   * Saves the successful seeds.
   */
  void saveSuccessfulSeeds ();

  /**
   * Displays the hill-shaded DTM in steps/hill.png file.
   */
  void saveHillImage ();

  /**
   * Displays the slope-shaded DTM in steps/shade.png file.
   */
  void saveShadingImage ();

  /**
   * Displays RORPO output in steps/rorpo.png file.
   */
  void saveRorpoImage ();

  /**
   * Displays gradient magnitude in steps/sobel.png file.
   */
  void saveSobelImage ();

  /**
   * Displays extracted blurred segments in steps/fbsd.png image.
   * @param im_w PNG image width.
   * @param im_h PNG image height.
   */
  void saveFbsdImage (int im_w, int im_h);

  /**
   * Displays blurred-segment-based seeds in steps/seeds.png file.
   */
  void saveSeedsImage ();

  /**
   * Displays extracted roads in steps/roads.png image.
   */
  void saveAsdImage ();


private:

  /** Virtual map width (global DTM). */
  int vm_width;
  /** Virtual map height (global DTM. */
  int vm_height;
  /** DTM cell size. */
  float csize;
  /** DTM/Point grid subdivision. */
  int sub_div;

  /** Tool configuration. */
  AmrelConfig cfg;
  /** Pointer to the loaded point tile set. */
  IPtTileSet *ptset;
  /** Flag indicating if tiles are already loaded in the tile set. */
  bool tile_loaded;
  /** Flag indicating if tile set buffers are already created. */
  bool buf_created;
  /** Map of detected roads. */
  unsigned short *track_map;
  /** Image to meter ratio : inverse of cell size. */
  float iratio;

  /** Digital terrain model (DTM). */
  TerrainMap *dtm_in;
  /** Shaded DTM image. */
  Image2D<unsigned char> *dtm_map;
  /** Rorpo output image. */
  Image2D<unsigned char> *rorpo_map;
  /** Successful seeds saving modality. */
  bool save_seeds;

  /** Gradient map. */
  VMap *gmap;
  /** Blurred segment detector. */
  BSDetector bsdet;
  /** Digital straight segments from FBSD. */
  std::vector<DigitalStraightSegment> dss;
  /** Produced seeds for road detection. */
  std::vector<Pt2i> *out_seeds;
  /** Successful seeds for road detection. */
  std::vector<Pt2i> *out_sucseeds;

  /** Road detector. */
  CTrackDetector *ctdet;
  /** Number of detected mountain road sections. */
  int count_of_roads;
  /** List of detected road sections. */
  std::vector<CarriageTrack *> road_sections;


};
#endif

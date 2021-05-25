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

#include <string>
#include <vector>
#include "amreltool.h"
#include "amreltimer.h"

using namespace std;


int main (int argc, char *argv[])
{
  AmrelTool autodet;

  for (int i = 1; i < argc; i++)
  {
    if (string(argv[i]).at(0) == '-')
    {
      if (string(argv[i]) == string ("--auto"))
        autodet.config()->setStep (AmrelConfig::STEP_ALL);
      else if (string(argv[i]) == string ("--sawing"))
        autodet.config()->setStep (AmrelConfig::STEP_SAWING);
      else if (string(argv[i]) == string ("--shade"))
        autodet.config()->setStep (AmrelConfig::STEP_SHADE);
      else if (string(argv[i]) == string ("--rorpo"))
        autodet.config()->setStep (AmrelConfig::STEP_RORPO);
      else if (string(argv[i]) == string ("--sobel"))
        autodet.config()->setStep (AmrelConfig::STEP_SOBEL);
      else if (string(argv[i]) == string ("--fbsd"))
        autodet.config()->setStep (AmrelConfig::STEP_FBSD);
      else if (string(argv[i]) == string ("--seeds"))
        autodet.config()->setStep (AmrelConfig::STEP_SEEDS);
      else if (string(argv[i]) == string ("--asd"))
        autodet.config()->setStep (AmrelConfig::STEP_ASD);
      else if (string(argv[i]) == string ("--eco"))
        autodet.config()->setCloudAccess (IPtTile::ECO);
      else if (string(argv[i]) == string ("--mid"))
        autodet.config()->setCloudAccess (IPtTile::MID);
      else if (string(argv[i]) == string ("--top"))
        autodet.config()->setCloudAccess (IPtTile::TOP);
      else if (string(argv[i]) == string ("--pad"))
      {
        if (i == argc - 1
            || ! autodet.config()->setPadSize (atoi (argv[++i]))) return 0;
      }
      else if (string(argv[i]) == string ("--buf"))
      {

        if (i == argc - 1
            || ! autodet.config()->setBufferSize (atoi (argv[++i]))) return 0;
      }
      else if (string(argv[i]) == string ("--hill"))
        autodet.config()->setHillMap (true);
      else if (string(argv[i]) == string ("--map"))
        autodet.config()->setOutMap (true);
      else if (string(argv[i]) == string ("--color"))
        autodet.config()->setFalseColor (true);
      else if (string(argv[i]) == string ("--dtm"))
        autodet.config()->setBackDtm (true);
      else if (string(argv[i]) == string ("--unconnected"))
        autodet.config()->setConnected (false);
      else if (string(argv[i]) == string ("--bsminlength"))
      {
        if (i != argc - 1)
          autodet.config()->setMinBSLength (atoi (argv[++i]));
      }
      else if (string(argv[i]) == string ("--bsmaxthick"))
      {
        if (i != argc - 1)
          autodet.config()->setMaxBSThickness (atoi (argv[++i]));
      }
      else if (string(argv[i]) == string ("--seedshift"))
      {
        if (i != argc - 1)
          autodet.config()->setSeedShift (atoi (argv[++i]));
      }
      else if (string(argv[i]) == string ("--seedwidth"))
      {
        if (i != argc - 1)
          autodet.config()->setSeedWidth (atoi (argv[++i]));
      }
      else if (string(argv[i]) == string ("--silent"))
        autodet.config()->setVerbose (false);
      else if (string(argv[i]) == string ("--dtmdir"))
      {
        if (++i == argc)
        {
          std::cout << "DTM files path missing" << std::endl;
          return 0;
        }
        autodet.config()->setDtmDir (argv[i]);
      }
      else if (string(argv[i]) == string ("--xyzdir"))
      {
        if (++i == argc)
        {
          std::cout << "XYZ files path missing" << std::endl;
          return 0;
        }
        autodet.config()->setXyzDir (argv[i]);
      }
      else if (string(argv[i]) == string ("--import")
               || string(argv[i]) == string ("-i"))
      {
        if (++i == argc)
        {
          std::cout << "Imported tile name missing" << std::endl;
          return 0;
        }
        autodet.config()->setImportFile (argv[i]);
      }
      else if (string(argv[i]) == string ("--tile")
               || string(argv[i]) == string ("-t"))
      {
        if (++i == argc)
        {
          std::cout << "Tile name missing" << std::endl;
          return 0;
        }
        autodet.config()->addTileName (argv[i]);
      }
      else
      {
        cout << "Unknown option " << argv[i] << endl;
        return 0;
      }
    }
    else if (! autodet.config()->setInputName (string (argv[i])))
    {
      std::cout << "Conflicting input names:" << autodet.config()->inputName ()
                << " and " << argv[i] << std::endl;
      return 0;
    }
  }

  autodet.run ();

  return EXIT_SUCCESS;
}

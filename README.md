# AMREL: Automatic Mountain Road Extraction from LiDAR data

AMREL is a software tool to automatically extract roads from large LiDAR
data sets of mountainous areas.
The road extraction is performed in two steps:

* seeds production using LiDAR derived digital terrain model (DTM) tiles,
* road detection using LiDAR raw data (ground 3D points).

Authors: Philippe Even and Phuc Ngo, LORIA/ADAGIo, Université de Lorraine.

Reference : Even, P. and Ngo, P., 2021.
[Automatic forest road extraction from LiDAR data of mountainous areas.](https://doi.org/10.1007/978-3-030-76657-3_6)
In International Joint Conference of Discrete Geometry Mathematical Morphology,
Uppsala, Sweden, May 24-17, 2021 (Springer LNCS 12708), pp. 93-106.
[hal-03144147](https://hal.archives-ouvertes.fr/hal-03144147).

## QUICK SETUP / COMPILATION GUIDE

* Requires libpng and OpenMP libraries.
* CMakeLists.txt provided for cmake.
* AMREL.pro provided for qmake.

## INPUTS

### NVM files
NVM is the internal format to encode digital terrain models.
It can be produced from standard ASC files using the following command:
```
AMREL --dtmdir path --import tile.asc --import next1.asc** ... **--import nextn.asc -t mytile
```
where **path** is the path to ASC files, **tile.asc** the suffixed name of the
selected ASC file, **nextn.asc** the name of its nth 8-neighbour tile
(optionally used to ensure normal continuity between adjacent tiles)
and **mytile** the unsuffixed name given to the internal format tile.
These files should be placed in **nvm** directory.

### TIL files
TIL is the internal format to encode arranged sets of 3D points.
It can be produced from XYZ files (a text format where each lines contains
X, Y then Z point coordinates separated by a space, that can be extracted
from LiDAR raw data using las2txt LAStools command) using the following
command:
```
AMREL --xyzdir path --import tile.txt -t mytile
```
where **path** is the path to XYZ files, **tile.txt** the suffixed name of the
selected XYZ file, and **mytile** the unsuffixed name given to the internal
format tile (should be the same than corresponding NVM file).
These files should be placed in appropriate **til** subdirectory.

### Point cloud access modes

* Mode "Top" : fast access but heavy memory consumption,
* Mode "Mid" : medium time and memory performance,
* Mode "Eco" : longer execution time but less memory required.

No difference in accuracy between these modes.

Default mode is "Top".
Other modes can be selected using **--mid** or **--eco** options when
running AMREL. Conversions are automatic.

### Example of NVM and TIL files

Gris-Mouton sector tiles of Fossard LiDAR data set are available in
[AMRELtest GitHub repository](https::github.com/evenp/AMRELtest):

1) Copy the files from **AMRELtest/Data/nvm/** directory
into **$AMREL/nvm/** directory.

2) Copy the files from **AMRELtest/Data/til/eco/** directory
into **AMREL/til/eco/** directory.

3) Copy the file *$AMRELtest/Data/tilesets/grismouton.txt*
into **AMREL/tilesets** directory.

4) Run **AMREL grismouton.txt**

The result map is available in **AMREL/steps/roads.png** image.
To test the precision of the achieved detection:

1) Copy this image in **AMRELtest/Data/detections/roads_grismouton.png**

2) In **AMRELtest/Code/** directory, run **roadgt --comp grismouton**

## TYPICAL USES

To extract roads on **mytile** single tile:
```
AMREL mytile
```
To add tile **tname** to tile set **tilesets/tsetname.txt**:
```
AMREL --add tname tsetname.txt
```
To extract roads on tile **tname**
```
AMREL tname
```
To extract roads on tile set **tilesets/tsetname.txt**
```
AMREL tsetname.txt
```
To only run first stage (slope shading) and output a result map in
**steps/shade.png**:
```
AMREL --shade --map tsetname.txt
```
To only run second stage (RORPO filter to enhance elongated shapes) and
output a result map in **steps/shade.png** (assuming the result of previous
stage is available) :
```
AMREL --rorpo --map tsetname.txt
```
To only run third stage (gradient map computation using 5x5 mask Sobel filter)
and output a result map in **steps/sobel.png** (assuming the result of previous
stage is available) :
```
AMREL --sobel --map tsetname.txt
```
To only run fourth stage (straight segment extraction using FBSD detector)
and output a result map in **steps/fbsd.png** (assuming the result of previous
stage is available) :
```
AMREL --fbsd --map tsetname.txt
```
To only run fifth stage (seeds  positionning on extracted segments)
and output a result map in **steps/seeds.png** (assuming the result of previous
stage is available) :
```
AMREL --seeds --map tsetname.txt
```
To only run sixth stage (road extraction using ASD) using extracted seeds
(**steps/roads.png** output map is systematically produced) :
```
AMREL --asd tsetname.txt
```
To build seeds from scratch on tiles listed in file **tilesets/tsetname.txt**
using groups of 5x5 tiles:
```
AMREL --sawing --pad 5 tsetname.txt
```
To extract roads form selected seeds on tiles listed in file
**tilesets/tsetname.txt** using groups of 7x7 tiles:
```
AMREL --asd --buf 7 tsetname.txt
```

## OUTPUTS

###Map of extracted roads: roads.png

A PNG format image displaying the extracted roads.
It is stored into **steps** directory.

###Seeds to edit extracted roads in ILSD tool: autodet.txt

Extracted roads can be edited using [ILSD](https://github.com/evenp/ILSD.git),
an interactive road extraction tool from raw LiDAR data:

1) Run **ILSD**

2) In **Files** menu, select **Load settings** item,
then **AMREL/steps/autodet.ini** file;
this file contains the used road extraction parameters.

3) In **Selection** menu, select **Load selection** item,
then **AMREL/steps/autodet.txt** file;
this file contains the coordinates of successful seeds which produce
the output roads.

## LIST OF COMMAND LINE ARGUMENTS

| Argument | Description |
|-----|-------------|
| "name" | Specifies a single tile or a tile set |
| --add "name" (or -a "name") | Adds tile "name" to a specified tile set |
| --dtmdir "name" | Specifies the path to imported DTM files |
| --xyzdir "name" | Specifies the path to imported point files |
| --import "name" (or -i "name") | Specifies the suffixed name of an imported file |
| --auto (or "") | Runs seeds selection from DTM map and road detection from point cloud |
| --shade | Runs only first stage: slope-shading of DTM map |
| --rorpo | Runs only second stage: RORPO filtering from shaded map |
| --sobel | Runs only third stage: SOBEL gradient map production from filtered map |
| --fbsd | Runs only fourth stage: FBSD straight segment detection from gradient map |
| --seeds | Runs only fifth stage: seeds selection from straight segments |
| --asd | Runs only sixth stage: road extraction from selected seeds |
| --sawing | Runs the five first stages: seeds selection from a DTM map |
| --eco | Uses slow access mode to ground points |
| --mid | Uses medium access mode to ground points |
| --pad "size" | Uses size x size groups of tiles for seed selection (positive odd integer value) |
| --buf "size" | Uses size x size groups of tiles for road extraction (positive odd integer value) |
| --hill | Outputs hill-shaded DTM in steps/hill.png |
| --map | Outputs results in a PNG image |
| --color | Outputs results in a colored PNG image (for each segment, seed or road section) |
| --dtm | Outputs results superimposed DTM map |
| --unconnected | Does not interpolate between valid road profiles |
| --bsminlength "length" | Sets minimal length for straight segment extraction (positive integer value) |
| --bsmaxthick "thickness" | Sets maximal thickness for straight segment extraction (positive integer value) |
| --seedshift "shift" | Sets the distance between successive seeds |
| --seedwidth "width" | Sets seed width |
| --silent | Suppresses message printing |

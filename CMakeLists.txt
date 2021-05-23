cmake_minimum_required(VERSION 3.5)

project(AMREL)

cmake_policy(SET CMP0057 NEW)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-std=c++11")

add_definitions(-O3)

# Find libpng library
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
message(STATUS "LibPNG include dir: '${PNG_INCLUDE_DIRS}' and '${LIBPNG_LIBRARIES}'")

# Find openmp library
find_package(OpenMP REQUIRED)
set(openmp_libraries ${OpenMP_CXX_FLAGS}) 

# Input
include_directories(
    ${PROJECT_SOURCE_DIR}/Amrel
    ${PROJECT_SOURCE_DIR}/ASDetector
    ${PROJECT_SOURCE_DIR}/BlurredSegment
    ${PROJECT_SOURCE_DIR}/ConvexHull
    ${PROJECT_SOURCE_DIR}/DirectionalScanner
    ${PROJECT_SOURCE_DIR}/ImageTools
    ${PROJECT_SOURCE_DIR}/PointCloud
    ${PROJECT_SOURCE_DIR}/RORPO2D
)

# Add files
set(HEADERS ASDetector/carriagetrack.h
           ASDetector/ctrackdetector.h
           ASDetector/ctracksection.h
           ASDetector/plateau.h
           ASDetector/plateaumodel.h
           Amrel/amrelconfig.h
           Amrel/amreltimer.h
           Amrel/amreltool.h
           BlurredSegment/biptlist.h
           BlurredSegment/blurredsegment.h
           BlurredSegment/bsdetector.h
           BlurredSegment/bsproto.h
           BlurredSegment/bstracker.h
           BlurredSegment/nfafilter.h
           ConvexHull/antipodal.h
           ConvexHull/chvertex.h
           ConvexHull/convexhull.h
           DirectionalScanner/adaptivescannero1.h
           DirectionalScanner/adaptivescannero2.h
           DirectionalScanner/adaptivescannero7.h
           DirectionalScanner/adaptivescannero8.h
           DirectionalScanner/directionalscanner.h
           DirectionalScanner/directionalscannero1.h
           DirectionalScanner/directionalscannero2.h
           DirectionalScanner/directionalscannero7.h
           DirectionalScanner/directionalscannero8.h
           DirectionalScanner/scannerprovider.h
           DirectionalScanner/vhscannero1.h
           DirectionalScanner/vhscannero2.h
           DirectionalScanner/vhscannero7.h
           DirectionalScanner/vhscannero8.h
           ImageTools/absrat.h
           ImageTools/digitalstraightline.h
           ImageTools/digitalstraightsegment.h
           ImageTools/edist.h
           ImageTools/pt2i.h
           ImageTools/vmap.h
           ImageTools/vr2i.h
           PointCloud/asarea.h
           PointCloud/asmath.h
           PointCloud/astrack.h
           PointCloud/ipttile.h
           PointCloud/ipttileset.h
           PointCloud/pt2f.h
           PointCloud/pt3f.h
           PointCloud/pt3i.h
           PointCloud/terrainmap.h
           PointCloud/vr2f.h
           RORPO2D/basic_operators.hpp
           RORPO2D/image_operations.hpp
           RORPO2D/image_png.hpp
           RORPO2D/image.hpp
           RORPO2D/po.hpp
           RORPO2D/rorpo.hpp
           RORPO2D/basic_operators.hpp
           RORPO2D/sort_functions.hpp
)

set(SOURCES main.cpp
           ASDetector/carriagetrack.cpp
           ASDetector/ctrackdetector.cpp
           ASDetector/ctracksection.cpp
           ASDetector/plateau.cpp
           ASDetector/plateaumodel.cpp
           Amrel/amrelconfig.cpp
           Amrel/amreltimer.cpp
           Amrel/amreltool.cpp
           BlurredSegment/biptlist.cpp
           BlurredSegment/blurredsegment.cpp
           BlurredSegment/bsdetector.cpp
           BlurredSegment/bsproto.cpp
           BlurredSegment/bstracker.cpp
           BlurredSegment/nfafilter.cpp
           ConvexHull/antipodal.cpp
           ConvexHull/chvertex.cpp
           ConvexHull/convexhull.cpp
           DirectionalScanner/adaptivescannero1.cpp
           DirectionalScanner/adaptivescannero2.cpp
           DirectionalScanner/adaptivescannero7.cpp
           DirectionalScanner/adaptivescannero8.cpp
           DirectionalScanner/directionalscanner.cpp
           DirectionalScanner/directionalscannero1.cpp
           DirectionalScanner/directionalscannero2.cpp
           DirectionalScanner/directionalscannero7.cpp
           DirectionalScanner/directionalscannero8.cpp
           DirectionalScanner/scannerprovider.cpp
           DirectionalScanner/vhscannero1.cpp
           DirectionalScanner/vhscannero2.cpp
           DirectionalScanner/vhscannero7.cpp
           DirectionalScanner/vhscannero8.cpp
           ImageTools/digitalstraightline.cpp
           ImageTools/digitalstraightsegment.cpp
           ImageTools/edist.cpp
           ImageTools/pt2i.cpp
           ImageTools/vmap.cpp
           ImageTools/vr2i.cpp
           PointCloud/asarea.cpp
           PointCloud/astrack.cpp
           PointCloud/ipttile.cpp
           PointCloud/ipttileset.cpp
           PointCloud/pt2f.cpp
           PointCloud/pt3f.cpp
           PointCloud/pt3i.cpp
           PointCloud/terrainmap.cpp
           PointCloud/vr2f.cpp
)

# Create executable and link library
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_link_libraries(${PROJECT_NAME} ${PNG_LIBRARY} ${OpenMP_LIBRARY})


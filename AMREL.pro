######################################################################
# Automatically generated by qmake (3.0) mer. nov. 7 21:11:32 2018
######################################################################

TEMPLATE = app
TARGET = AMREL
INCLUDEPATH += . \
           Amrel \
           ASDetector \
           BlurredSegment \
           DirectionalScanner \
           ImageTools \
           PointCloud \
           RORPO2D \
           ShapeFile-1.3.0
OBJECTS_DIR = obj

# Input
HEADERS += ASDetector/carriagetrack.h \
           ASDetector/ctrackdetector.h \
           ASDetector/ctracksection.h \
           ASDetector/plateau.h \
           ASDetector/plateaumodel.h \
           Amrel/amrelconfig.h \
           Amrel/amrelmap.h \
           Amrel/amreltimer.h \
           Amrel/amreltool.h \
           BlurredSegment/antipodal.h \
           BlurredSegment/biptlist.h \
           BlurredSegment/blurredsegment.h \
           BlurredSegment/bsdetector.h \
           BlurredSegment/bsproto.h \
           BlurredSegment/bstracker.h \
           BlurredSegment/chvertex.h \
           BlurredSegment/convexhull.h \
           BlurredSegment/nfafilter.h \
           DirectionalScanner/adaptivescannero1.h \
           DirectionalScanner/adaptivescannero2.h \
           DirectionalScanner/adaptivescannero7.h \
           DirectionalScanner/adaptivescannero8.h \
           DirectionalScanner/directionalscanner.h \
           DirectionalScanner/directionalscannero1.h \
           DirectionalScanner/directionalscannero2.h \
           DirectionalScanner/directionalscannero7.h \
           DirectionalScanner/directionalscannero8.h \
           DirectionalScanner/scannerprovider.h \
           DirectionalScanner/vhscannero1.h \
           DirectionalScanner/vhscannero2.h \
           DirectionalScanner/vhscannero7.h \
           DirectionalScanner/vhscannero8.h \
           ImageTools/absrat.h \
           ImageTools/digitalstraightline.h \
           ImageTools/digitalstraightsegment.h \
           ImageTools/edist.h \
           ImageTools/pt2i.h \
           ImageTools/vmap.h \
           ImageTools/vr2i.h \
           PointCloud/asarea.h \
           PointCloud/asmath.h \
           PointCloud/astrack.h \
           PointCloud/ipttile.h \
           PointCloud/ipttileset.h \
           PointCloud/pt2f.h \
           PointCloud/pt3f.h \
           PointCloud/pt3i.h \
           PointCloud/terrainmap.h \
           PointCloud/vr2f.h \
           RORPO2D/basic_operators.hpp \
           RORPO2D/image_operations.hpp \
           RORPO2D/image_png.hpp \
           RORPO2D/image.hpp \
           RORPO2D/po.hpp \
           RORPO2D/rorpo.hpp \
           RORPO2D/basic_operators.hpp \
           RORPO2D/sort_functions.hpp \
           ShapeLib-1.3.0/shapefil.h

SOURCES += main.cpp \
           ASDetector/carriagetrack.cpp \
           ASDetector/ctrackdetector.cpp \
           ASDetector/ctracksection.cpp \
           ASDetector/plateau.cpp \
           ASDetector/plateaumodel.cpp \
           Amrel/amrelconfig.cpp \
           Amrel/amrelmap.cpp \
           Amrel/amreltimer.cpp \
           Amrel/amreltool.cpp \
           BlurredSegment/antipodal.cpp \
           BlurredSegment/biptlist.cpp \
           BlurredSegment/blurredsegment.cpp \
           BlurredSegment/bsdetector.cpp \
           BlurredSegment/bsproto.cpp \
           BlurredSegment/bstracker.cpp \
           BlurredSegment/chvertex.cpp \
           BlurredSegment/convexhull.cpp \
           BlurredSegment/nfafilter.cpp \
           DirectionalScanner/adaptivescannero1.cpp \
           DirectionalScanner/adaptivescannero2.cpp \
           DirectionalScanner/adaptivescannero7.cpp \
           DirectionalScanner/adaptivescannero8.cpp \
           DirectionalScanner/directionalscanner.cpp \
           DirectionalScanner/directionalscannero1.cpp \
           DirectionalScanner/directionalscannero2.cpp \
           DirectionalScanner/directionalscannero7.cpp \
           DirectionalScanner/directionalscannero8.cpp \
           DirectionalScanner/scannerprovider.cpp \
           DirectionalScanner/vhscannero1.cpp \
           DirectionalScanner/vhscannero2.cpp \
           DirectionalScanner/vhscannero7.cpp \
           DirectionalScanner/vhscannero8.cpp \
           ImageTools/digitalstraightline.cpp \
           ImageTools/digitalstraightsegment.cpp \
           ImageTools/edist.cpp \
           ImageTools/pt2i.cpp \
           ImageTools/vmap.cpp \
           ImageTools/vr2i.cpp \
           PointCloud/asarea.cpp \
           PointCloud/astrack.cpp \
           PointCloud/ipttile.cpp \
           PointCloud/ipttileset.cpp \
           PointCloud/pt2f.cpp \
           PointCloud/pt3f.cpp \
           PointCloud/pt3i.cpp \
           PointCloud/terrainmap.cpp \
           PointCloud/vr2f.cpp \
           ShapeLib-1.3.0/dbfopen.c \
           ShapeLib-1.3.0/safileio.c \
           ShapeLib-1.3.0/shpopen.c \
           ShapeLib-1.3.0/shptree.c

LIBS += -lpng

QMAKE_CXXFLAGS+= -fopenmp -std=c++11
QMAKE_LFLAGS +=  -fopenmp
LIBS += -fopenmp

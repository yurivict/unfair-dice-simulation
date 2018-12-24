#include <iostream>
#include <fstream>
#include <iomanip>
#include <array>

#include <math.h>

#include <osg/Geometry>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osg/MatrixTransform>

#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/Channel>
#include <osgAnimation/UpdateMatrixTransform>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedRotateAxisElement>

#include <nlohmann/json.hpp>

#include "Vec3.h"
#include "Mat3.h"
#include "RotationSolver.h"
#include "RotationMatrixIntegrator.h"


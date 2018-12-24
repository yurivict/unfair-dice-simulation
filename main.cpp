#include "all-headers.h"

// see the collision detection library: /usr/ports/math/libccd

//
// Cube parameters
//

class CubeParams {
public:
  class Motion {
  public:
    Vec3  x0;
    Vec3  v0;
    Vec3  rot0;       // initial static rotation
    Vec3  w0;         // initial rotation vector
  }; // Motion
  // static params
  Float mass;
  Vec3  size;
  Vec3  CM;         // center of mass relative to the center of the cube
  Vec3  Iprincipal; // principal values of the rotational mass, relative to the center of mass
  Vec3  Irotation;  // rotation of the principle axes relative to the cube's center of mass
  // motion params
  Motion motion;
  static CubeParams readFromJson(const char *fname) {
    // read file -> json
    std::ifstream file ("cube-params.json");
    std::stringstream buffer;
    buffer << file.rdbuf();
    auto j = nlohmann::json::parse(buffer.str().c_str());
    // parse
    auto parseV3 = [&j](const char *key) {
      Vec3 v;
      int i = 0;
      for (auto &e : j[key].items())
        v(++i) = e.value().get<Float>();
      return v;
    };
    CubeParams params;
    params.mass        = j["mass"];
    params.size        = parseV3("size");
    params.CM          = parseV3("cm");
    params.Iprincipal  = parseV3("Iprincipal");
    params.Irotation   = parseV3("Irotation");
    params.motion.x0   = parseV3("x0");
    params.motion.v0   = parseV3("v0");
    params.motion.rot0 = parseV3("rot0");
    params.motion.w0   = parseV3("w0");
    return params;
  }
}; // CubeParams


const CubeParams cubeParams = CubeParams::readFromJson("cube-params.json");

// parameter
static Float dt = 0.0001; // in secs

// see https://math.stackexchange.com/questions/3043577/can-the-equation-with-quadratic-and-trigonometric-terms-be-solved for a solution discussion

typedef osg::Matrixd::value_type Float;


class Parabola {
public:
  static Vec3 evolve(Float t, const Vec3 &v, Float g) {return Vec3(v(X)*t, v(Y)*t, v(Z)*t - 0.5*g*t*t);}
};

// params

class Params {
public:
  static constexpr Float g = 0.2; // gravity const, acts towards Z-
  static const Vec3 initPosition;
}; // Params

const Vec3 Params::initPosition(1., 1., 1.);

const Float SZ = 10.; // space size

class Wall {
public:
  Vec3    pt;     // some point on the wall plane
  Vec3    normal; // normal directed "into" our space
  Wall(const Vec3 &newPt, const Vec3 &newNormal) : pt(newPt), normal(newNormal) { }
}; // Wall

//
// Overall motion solver
//
class MotionSolver {
  typedef std::pair<Vec3/*position*/,Mat3/*rotation*/> Res; // type of the value that we are returning
private: // data
  CubeParams            cubeParams;
  std::vector<Wall>     walls;
  Float                 t0;     // time of the current segment
  CubeParams::Motion    motion0; // motion parameters of the current segment
  RotationSolver<Float>*                                  rotationSolver;
  RotationMatrixIntegrator<Float, RotationSolver<Float>>* rotationIntegrator;
public:
  MotionSolver(const CubeParams &newCubeParams, const std::vector<Wall> &newWalls) : cubeParams(newCubeParams), walls(newWalls), t0(0), motion0(cubeParams.motion) {
    // initial solvers
    rotationSolver = new RotationSolver<Float>(cubeParams.Iprincipal, Vec3(0,0,0)/*M*/, motion0.w0, t0, dt);
    rotationIntegrator = new RotationMatrixIntegrator<Float, RotationSolver<Float>>(rotationSolver, motion0.rot0, t0, dt);
  }
  Res operator()(Float t) {
    const Vec3 p = motion0.x0 + Parabola::evolve(t - t0, motion0.v0, Params::g);
    const Mat3 r = (*rotationIntegrator)(t - t0);
    // detect collision with walls
    Vec3 collPt;
    int collWallIdx = -1;
    if (isCollision(p, r, &collPt, &collWallIdx))
      std::cout << "*** Collision: t=" << t << " pt=" << collPt << " wall=" << collWallIdx << std::endl;
    // output
    return Res(p, r);
  }
private:
  bool isCollision(const Vec3 &p, const Mat3 &r, Vec3 *outCorner, int *outWallIdx) const {
    const Vec3 cm = -cubeParams.CM - cubeParams.size/2;
    const Vec3 cp = -cubeParams.CM + cubeParams.size/2;
    for (Float x : {cm(1), cp(1)})
      for (Float y : {cm(2), cp(2)})
        for (Float z : {cm(3), cp(3)}) {
          //std::cout << "... corner=" << Vec3(x,y,z) << " dist-to-wall0=" << (walls[0].pt-(p+r*Vec3(x,y,z)))*walls[0].normal << std::endl;
          if (isOutside(p + r*Vec3(x,y,z), walls, outWallIdx)) {
            *outCorner = Vec3(x,y,z);
            return true;
          }
        }
    return false;
  }
  static bool isOutside(const Vec3 &p, const std::vector<Wall> &walls, int *outWallIdx) {
    *outWallIdx = 0;
    for (auto &wall : walls) {
      if (isOutside(p, wall))
        return true;
      ++*outWallIdx;
    }
    return false;
  }
  static bool isOutside(const Vec3 &p, const Wall &wall) {
    return (wall.pt-p)*wall.normal > 0;
  }
  //Float energy(const Vec3 &p, const Vec3 &v, const Vec3 &w) const {
  //  Float potentialEnergy  = p(Z)*Params::g*cubeParams.mass;
  //  Float kinericEnergy    = cubeParams.mass*v.len2()/2;
  //  Float rotationalEnergy = ;
  //  return potentialEnergy+kinericEnergy+rotationalEnergy;
  //}
}; // MotionSolver

class MyTransformCallback : public osg::NodeCallback {
public:
  MyTransformCallback(const CubeParams &newCubeParams, Float newTmBegin)
    : cubeParams(newCubeParams)
    {
      addWalls();
      // create solver
      motionSolver = new MotionSolver(cubeParams, walls);
    }
  virtual void operator() (osg::Node* node, osg::NodeVisitor* nv) {
    osg::MatrixTransform* transform = dynamic_cast<osg::MatrixTransform*>(node);
    if (nv && transform && nv->getFrameStamp()) {
      // get time
      Float timeSec = nv->getFrameStamp()->getSimulationTime();
      //std::cout << "tm=" << timeSec << " realTime=" << ::time(0) << std::endl;

      // evolve
      auto [p, r] = (*motionSolver)(timeSec);
      r = r.t();

      // matrix
      osg::Matrix tranMat = osg::Matrix::translate(p(X),p(Y),p(Z));
      osg::Matrix rotMat(
        r(1,1), r(1,2), r(1,3), 0,
        r(2,1), r(2,2), r(2,3), 0,
        r(3,1), r(3,2), r(3,3), 0,
        0,      0,      0,      1
      );

      // apply
      transform->setMatrix(rotMat*tranMat);
    }
    // must continue subgraph traversal.
    traverse(node,nv);
  }
protected:
  CubeParams cubeParams;
  // parabolic motion: see https://en.wikipedia.org/wiki/Projectile_motion#Displacement
  std::vector<Parabola> pastParabolas;    // parabolas that we have advanced past
  std::vector<Wall>     walls;
  // rotation solver: it is fundamentally a 2-step process, so we have 2 objects: solver and integrator
  MotionSolver*         motionSolver;     // solver
private:
  void addWalls() {
    walls.push_back(Wall(Vec3(0.,0.,0.), Vec3(0.,0.,1.))); // bottom (XY)
    //walls.push_back(Wall({{0.,0.,0.}}, {{1.,0.,0.}})); // left (YZ)
  }
}; // MyTransformCallback

osg::ref_ptr<osg::Geode> createAxis()
{
    osg::ref_ptr<osg::Geode> geodeAxe (new osg::Geode());
    osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

    osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());
    vertices->push_back (osg::Vec3(0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3(SZ,  0.0, 0.0));
    vertices->push_back (osg::Vec3(0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3(0.0, SZ,  0.0));
    vertices->push_back (osg::Vec3(0.0, 0.0, 0.0));
    vertices->push_back (osg::Vec3(0.0, 0.0, SZ));
    geometry->setVertexArray (vertices.get());

    osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
    colors->push_back (osg::Vec4 (1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (1.0f, 0.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 1.0f, 0.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 0.0f, 1.0f, 1.0f));
    colors->push_back (osg::Vec4 (0.0f, 0.0f, 1.0f, 1.0f));
    geometry->setColorArray (colors.get(), osg::Array::BIND_PER_VERTEX);
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));

    geodeAxe->addDrawable(geometry.get());
    geodeAxe->getOrCreateStateSet()->setMode(GL_LIGHTING, false);
    return geodeAxe;
}


int main(int argc, char* argv[]) {
    osg::ArgumentParser arguments(&argc, argv);
    osgViewer::Viewer viewer(arguments);

    viewer.setCameraManipulator(new osgGA::TrackballManipulator());

    osg::ref_ptr<osg::Group> grp = new osg::Group; // will be connected to the viewer
    osg::ref_ptr<osgAnimation::BasicAnimationManager> mng = new osgAnimation::BasicAnimationManager(); // animation manager to the scene graph to get it called during update traversals
    osg::ref_ptr<osg::Group> root = new osg::Group;
    osg::ref_ptr<osg::Geode> axe = createAxis();
    osg::ref_ptr<osg::Geode> geodeCube = new osg::Geode;
    //osg::ref_ptr<osg::Geode> geodeTime = new osg::Geode;

    { // osg::Box is created centered around its geometric center
      auto c = -cubeParams.CM;
      geodeCube->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3(c(X), c(Y), c(Z)), cubeParams.size(1), cubeParams.size(2), cubeParams.size(3))));
      // add dice-style markings on its sides
      Float markSz = 0.05*std::min(cubeParams.size(X), std::min(cubeParams.size(Y),cubeParams.size(Z)));
      auto addSphere = [&c,geodeCube](unsigned idx, int side, unsigned idx1, Float pct1, unsigned idx2, Float pct2) {
        Vec3 v = c + Vec3::one(idx, side*cubeParams.size(idx)/2) + Vec3::one(idx1, pct1*cubeParams.size(idx1)/2) + Vec3::one(idx2, pct2*cubeParams.size(idx2)/2);
        geodeCube->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(v(X), v(Y), v(Z)), 0.1)));
      };
      Float PCT = 0.4;
      addSphere(Y,-1, X,+0.0, Z,+0.0);   // 1
      addSphere(X,+1, Y,+0.0, Z,+PCT);   // 2
      addSphere(X,+1, Y,+0.0, Z,-PCT);
      addSphere(Y,+1, X,+0.0, Z,+0.0);   // 3
      addSphere(Y,+1, X,+PCT, Z,+PCT);
      addSphere(Y,+1, X,-PCT, Z,-PCT);
      addSphere(X,-1, Y,+PCT, Z,+PCT);   // 4
      addSphere(X,-1, Y,+PCT, Z,-PCT);
      addSphere(X,-1, Y,-PCT, Z,+PCT);
      addSphere(X,-1, Y,-PCT, Z,-PCT);
      addSphere(Z,+1, X,+PCT, Y,+PCT);   // 5
      addSphere(Z,+1, X,+PCT, Y,-PCT);
      addSphere(Z,+1, X,-PCT, Y,+PCT);
      addSphere(Z,+1, X,-PCT, Y,-PCT);
      addSphere(Z,+1, X,+0.0, Y,+0.0);
      addSphere(Z,-1, X,+PCT, Y,+PCT);   // 6
      addSphere(Z,-1, X,+PCT, Y,+0.0);
      addSphere(Z,-1, X,+PCT, Y,-PCT);
      addSphere(Z,-1, X,-PCT, Y,+PCT);
      addSphere(Z,-1, X,-PCT, Y,+0.0);
      addSphere(Z,-1, X,-PCT, Y,-PCT);
    }

    //geodeTime->addDrawable(new osg::ShapeDrawable(new osgWidget::Label("Abc", "")));

    //Transformation to be manipulated by the animation
    osg::ref_ptr<osg::MatrixTransform> trans = new osg::MatrixTransform();
    {
      trans->setName("AnimatedNode");
      //Dynamic object, has to be updated during update traversal
      trans->setDataVariance(osg::Object::DYNAMIC);
      // initialize MatrixTranform
      //trans->setMatrix(osg::Matrix::identity());
      trans->setMatrix(osg::Matrix::translate(osg::Vec3d(Params::initPosition[0], Params::initPosition[1], Params::initPosition[2])));
    }
    trans->setUpdateCallback(new MyTransformCallback(cubeParams, 0./*newTmBegin*/));

    // connect objects
    grp->setUpdateCallback(mng);
    grp->addChild(root);
      root->addChild(axe);
      root->addChild(trans);
        trans->addChild(geodeCube);

    //set the grp-Group with the scene and the AnimationManager as viewer's scene data
    viewer.setSceneData(grp.get());
    return viewer.run();
}


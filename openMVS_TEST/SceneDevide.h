#pragma once
#include <string>
#include <vector>
#include <map>

#include "MVS/Common.h"
#include "MVS/Scene.h"
#include "Common/Types.h"

#include <boost/geometry/geometry.hpp> 
#include <boost/geometry/geometries/register/point.hpp> 
#include <boost/geometry/geometries/register/ring.hpp> 

struct MyPoint {
	double x, y/*, z*/;
	MyPoint(double xt, double yt) { x = xt, y = yt; }
	MyPoint() {}
};
//struct MyPoint3d {
//	double x, y, z;
//	MyPoint3d(double xt, double yt, double zt) { x = xt, y = yt, z = zt; }
//	MyPoint3d() {}
//};

BOOST_GEOMETRY_REGISTER_POINT_2D(MyPoint, double, boost::geometry::cs::cartesian, x, y/*, z*/)
//BOOST_GEOMETRY_REGISTER_POINT_3D(MyPoint3d, double, boost::geometry::cs::cartesian, x, y, z)

typedef boost::geometry::model::polygon<MyPoint> Polygon_2d;
//BOOST_GEOMETRY_REGISTER_RING(Polygon_2d::ring_type)
//typedef boost::geometry::model::polygon<MyPoint3d> Polygon_3d;
//BOOST_GEOMETRY_REGISTER_RING(Polygon_3d::ring_type)

class SceneDevide
{
public:
	SceneDevide(MVS::Scene *sceneOri);
	~SceneDevide();


	bool InitialParams();
	bool SaveDevidedPointCould();
	bool SceneDevideProcess();
	bool ImageProcess();
	bool PointsCouldProcess();
	bool ImageCrop(const std::vector<Point2d>& range,
		const std::string & imagePath,
		const double & averageHeight,
		std::map<int, int>& matcher,
		MVS::Scene &scene);
	bool PointCloudCrop(const std::vector<Point2d>& range,
		std::map<int, int>& matcher,
		MVS::Scene &scene);
	static bool UniqueImageCamera(MVS::Scene &scene);
	static bool SimplicatePointCloud(const double pointPercentage, MVS::Scene &scene);
	static bool FliterRundantImage(const int numOfImageInEachDirection, MVS::Scene &scene, std::vector<int> &validImageIndex);

	typedef TPoint2<double> Point2d;
	std::vector<MVS::Scene> scenes;
	Point2d boundaryMinXY;
	Point2d boundaryMaxXY;
	unsigned int numOfScenes;
	unsigned int numOfScenesInX;
	unsigned int numOfScenesInY;
	int scaleParam;
	double averageHeight;
	double bufferRange;	// in percentage
	double sceneSizeX;
	double sceneSizeY;
	std::vector<std::vector<Point2d>> sceneRange;
	std::vector<std::map<int, int>> imageIndexMatcher;
	std::string workPath;
	const MVS::Scene *_pScene;	

	static int imageWidth;
	static int imageHeight;
};


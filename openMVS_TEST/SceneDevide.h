#pragma once
#include <string>
#include <vector>
#include <map>

#include "MVS/Common.h"
#include "MVS/Scene.h"
#include "Common/Types.h"


class SceneDevide
{
public:
	SceneDevide();
	~SceneDevide();


	bool InitialParams();
	bool SceneDevideProcess();
	static bool ImageCrop(const std::vector<Point2d> &range, const std::string &imagePath, const double &averageHeight, std::map<int, int> &matcher);


	typedef TPoint2<double> Point2d;
	std::vector<MVS::Scene> scenes;
	Point2d upLeftBoundary;
	Point2d bottomRightBoundary;
	unsigned int numOfSences;
	int scaleParam;
	double bufferRange;
	double sceneSizeX;
	double sceneSizeY;
	std::vector<std::vector<Point2d>> sceneRange;
	std::vector<std::map<int, int>> imageIndexMatcher;
	std::string workPath;
	MVS::Scene *_pScene;	


};


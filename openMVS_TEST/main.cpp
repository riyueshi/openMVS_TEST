#include "MVS/Common.h"
#include "MVS/Scene.h"

#include "SceneDevide.h"

using namespace MVS;

int main(int argc, LPCTSTR* argv)
{

	Scene scene;
	// load and estimate a dense point-cloud
	if (!scene.Load("F:\\MillerWorkPath\\openMVSWorkPath47\\scene_dense.mvs"))
		return EXIT_FAILURE;
	if (scene.pointcloud.IsEmpty()) {
		VERBOSE("error: empty initial point-cloud");
		return EXIT_FAILURE;
	}
	std::cout << scene.images.size() << std::endl;
	SceneDevide::UniqueImageCamera(scene);
	SceneDevide processer(&scene);
	processer.workPath = "F:\\MillerWorkPath\\VSProject\\WorkPath47";
	processer.numOfScenesInX = 2;
	processer.numOfScenesInY = 2;
	processer.boundaryMinXY = Point2d(-5.0, -5.0);
	processer.boundaryMaxXY = Point2d(5.0, 5.0);
	processer.InitialParams();
	std::cout << processer.scenes.at(0).images.size() << std::endl;
	//processer.SceneDevideProcess();
	processer.ImageProcess();
	std::cout << processer.scenes.at(0).images.size() << std::endl;
	for (size_t i = 0; i < 4; i++)
	{
		std::cout << processer.sceneRange.at(i).at(0) << " "<< processer.sceneRange.at(i).at(1) << std::endl;
	}
	//processer.PointsCouldProcess();
	for (size_t sceneIndex = 0; sceneIndex < processer.scenes.size(); sceneIndex++)
	{
		processer.PointCloudCrop(processer.sceneRange.at(sceneIndex), processer.imageIndexMatcher.at(sceneIndex), processer.scenes.at(sceneIndex));
	}
	processer.SaveDevidedPointCould();
	std::cout << processer.scenes.at(0).images.size() << std::endl;

	std::cout << "scene subdevide finished, press to exit" << std::endl;
	getchar();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/

#include "MVS/Common.h"
#include "MVS/Scene.h"

#include "SceneDevide.h"

using namespace MVS;

int main(int argc, LPCTSTR* argv)
{

	Scene scene;
	// load and estimate a dense point-cloud
	if (!scene.Load("F:\\MillerWorkPath\\mvsWorkPathGCP\\scene_dense.mvs"))
		return EXIT_FAILURE;
	if (scene.pointcloud.IsEmpty()) {
		VERBOSE("error: empty initial point-cloud");
		return EXIT_FAILURE;
	}

	SceneDevide::UniqueImageCamera(scene);
	SceneDevide processer(&scene);
	processer.workPath = "F:\\MillerWorkPath\\VSProject\\WorkPath";
	processer.numOfScenesInX = 2;
	processer.numOfScenesInY = 2;
	processer.boundaryMinXY = Point2d(-4.3535, -4.55049);
	processer.boundaryMaxXY = Point2d(3.8695, 3.9030);
	processer.InitialParams();
	//processer.SceneDevideProcess();
	if (!processer.ImageProcess())
	{
		std::cout << "failed to process image" << std::endl;
		getchar();
	}
	processer.PointsCouldProcess();
	processer.SaveDevidedScenes();
	//getchar();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/

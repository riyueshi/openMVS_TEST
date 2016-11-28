#include "MVS/Common.h"
#include "MVS/Scene.h"

#include "SceneDevide.h"

using namespace MVS;

int main(int argc, LPCTSTR* argv)
{
	//test
	{
		SceneDevide::imageWidth = 8176;
		SceneDevide::imageHeight = 6132;

		Scene scene;
		// load and estimate a dense point-cloud
		if (!scene.Load("F:\\MillerWorkPath\\mvsWorkPathSparse12\\sparse12_mesh.mvs"))
		{
			std::cout << "failed to load scene" << std::endl;
			getchar();
			return EXIT_FAILURE;
		}
		//std::cout << scene.mesh.faces.size() << std::endl; getchar();
		//SceneDevide::SimplicatePointCloud(0.05, scene);
		std::vector<int> imageIndexVec;
		SceneDevide::FliterRundantImage(15, scene,imageIndexVec);
		scene.Save("sparse12.mvs");
		scene.pointcloud.Save("sparsePoint.ply");
		std::cout << "simplicate point finished" << std::endl;
		getchar();
	}
	Scene scene;
	// load and estimate a dense point-cloud
	if (!scene.Load("F:\\MillerWorkPath\\openMVSWorkPath303\\scene.mvs"))
	{
		std::cout << scene.images.begin()->name << std::endl;
		std::cout << "failed to load scene" << std::endl;
		getchar();
		return EXIT_FAILURE;
	}
	if (scene.pointcloud.IsEmpty()) {
		VERBOSE("error: empty initial point-cloud");
		std::cout << "Enmpty point could" << std::endl;
		getchar();
		return EXIT_FAILURE;
	}
	std::cout << scene.images.size() << std::endl;
	SceneDevide::UniqueImageCamera(scene);
	SceneDevide processer(&scene);
	processer.workPath = "F:\\MillerWorkPath\\VSProject\\WorkPath303Sparese";
	processer.numOfScenesInX = 5;
	processer.numOfScenesInY = 5;
	processer.boundaryMinXY = Point2d(-7.0, -7.0);
	processer.boundaryMaxXY = Point2d(7.0, 7.0);
	processer.InitialParams();
	std::cout << processer.scenes.at(0).images.size() << std::endl;
	//processer.SceneDevideProcess();
	processer.ImageProcess();
	std::cout << processer.scenes.at(0).images.size() << std::endl;
	for (size_t i = 0; i < 4; i++)
	{
		std::cout << processer.sceneRange.at(i).at(0) << " "<< processer.sceneRange.at(i).at(1) << std::endl;
	}
	processer.PointsCouldProcess();
	//for (size_t sceneIndex = 0; sceneIndex < processer.scenes.size(); sceneIndex++)
	//{
	//	processer.PointCloudCrop(processer.sceneRange.at(sceneIndex), processer.imageIndexMatcher.at(sceneIndex), processer.scenes.at(sceneIndex));
	//}
	processer.SaveDevidedPointCould();
	std::cout << processer.scenes.at(0).images.size() << std::endl;

	std::cout << "scene subdevide finished, press to exit" << std::endl;
	getchar();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/

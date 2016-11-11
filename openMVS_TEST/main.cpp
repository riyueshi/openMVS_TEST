#include "MVS/Common.h"
#include "MVS/Scene.h"
#include <boost/program_options.hpp>

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


	{
		Camera camPlatform1 = scene.platforms[0].cameras[0];
		Camera camPlatform2 = scene.platforms[1].cameras[0];
		Camera camPlatform3 = scene.platforms[2].cameras[0];
		Camera camPlatform4 = scene.platforms[3].cameras[0];
		Camera camPlatform5 = scene.platforms[4].cameras[0];

		for (size_t i = 0; i < 4; i++)
		{
			scene.platforms[0].cameras.push_back(camPlatform1);
		}
		for (size_t i = 0; i < 3; i++)
		{
			scene.platforms[1].cameras.push_back(camPlatform2);
		}
		for (size_t i = 0; i < 3; i++)
		{
			scene.platforms[2].cameras.push_back(camPlatform3);
		}
		for (size_t i = 0; i < 3; i++)
		{
			scene.platforms[3].cameras.push_back(camPlatform4);
		}
		for (size_t i = 0; i < 4; i++)
		{
			scene.platforms[4].cameras.push_back(camPlatform5);
		}


		for (size_t i = 0; i < 4; i++)
		{
			scene.images[i].cameraID = i;
		}
		for (size_t i = 4,j=0; i < 7; i++,j++)
		{
			scene.images[i].cameraID = j;
		}
		for (size_t i = 7, j = 0; i < 10; i++, j++)
		{
			scene.images[i].cameraID = j;
		}
		for (size_t i = 10, j = 0; i < 13; i++, j++)
		{
			scene.images[i].cameraID = j;
		}
		for (size_t i = 13, j = 0; i <17; i++, j++)
		{
			scene.images[i].cameraID = j;
		}

	}

	SceneDevide processer(&scene);
	processer.SceneDevideProcess();
	
	// save the final mesh
	//const String baseFileName("test");
	//scene.Save(baseFileName + _T(".mvs"));
	//scene.pointcloud.Save(baseFileName + _T(".ply"));
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/

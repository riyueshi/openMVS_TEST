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
	processer.SceneDevideProcess();
	
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/

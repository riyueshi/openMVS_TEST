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

	SceneDevide processer(&scene);
	processer.SceneDevideProcess();
	
	// save the final mesh
	//const String baseFileName("test");
	//scene.Save(baseFileName + _T(".mvs"));
	//scene.pointcloud.Save(baseFileName + _T(".ply"));
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/

#include "SceneDevide.h"



SceneDevide::SceneDevide()
{
}


SceneDevide::~SceneDevide()
{
}

bool SceneDevide::InitialParams()
{
	return false;
}

bool SceneDevide::SceneDevideProcess()
{
	MVS::Scene &sceneOri = (*_pScene);
	
	//process one of the scene
	{
		//apply the original params to created scene
		scenes.at(0).platforms = sceneOri.platforms;

	}




	return false;
}

bool SceneDevide::ImageCrop(const std::vector<Point2d>& range, const std::string & imagePath, const double & averageHeight, std::map<int, int>& matcher)
{
	return false;
}

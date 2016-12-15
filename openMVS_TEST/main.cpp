#include "MVS/Common.h"
#include "MVS/Scene.h"

#include "SceneDevide.h"
#include "filesystemSimplified/file_system.hpp"

using namespace MVS;
bool RemoveImages(Scene &scene, std::vector<int> imageIndexVec)
{
	if (imageIndexVec.size() == 0)
	{
		std::cout << "image index vector is empity, press Enter to continue" << std::endl;
		getchar();
		return false;
	}
	//sort the image index (from big to small)
	sort(imageIndexVec.rbegin(), imageIndexVec.rend());

	//update the number of images
	const int image_num_privious(scene.nCalibratedImages);
	scene.nCalibratedImages -= imageIndexVec.size();

	for (size_t imageIndex = 0; imageIndex < imageIndexVec.size(); imageIndex++)
	{
		//remove the corresponding platform
		//int platformID = scene.images[imageIndexVec.at(imageIndex)].platformID;
		//scene.platforms.RemoveAt(platformID);
		//for (size_t imageIndex_ = 0; imageIndex_ < scene.images.size(); imageIndex_++)
		//{
		//	if (scene.images[imageIndex_].platformID>platformID)
		//	{
		//		scene.images[imageIndex_].platformID -= 1;
		//	}
		//}

		//remove the relative neighbor
		for (size_t imageIndex_ = 0; imageIndex_ < scene.images.size(); imageIndex_++)
		{
			for (size_t neighborIndex = 0; neighborIndex < scene.images[imageIndex_].neighbors.size(); neighborIndex++)
			{
				uint32_t &idRef = scene.images[imageIndex_].neighbors[neighborIndex].idx.ID;
				if (idRef > imageIndexVec.at(imageIndex))
				{
					idRef -= 1;
				}
				else if (idRef == imageIndexVec.at(imageIndex))
				{
					scene.images[imageIndex_].neighbors.RemoveAt(neighborIndex);
				}
			}
		}


		scene.images.RemoveAt(imageIndexVec.at(imageIndex));
		for (size_t pointIndex = 0; pointIndex < scene.pointcloud.points.size(); pointIndex++)
		{
			PointCloud::ViewArr &viewRef = scene.pointcloud.pointViews[pointIndex];
			const int viewSize = viewRef.size();
			for (size_t viewIndex = 0; viewIndex < viewSize; viewIndex++)
			{
				if (viewRef[viewSize - 1 - viewIndex] > imageIndexVec.at(imageIndex))
				{
					viewRef[viewSize - 1 - viewIndex] -= 1;
				}
				else if (viewRef[viewSize - 1 - viewIndex] == imageIndexVec.at(imageIndex))
				{
					viewRef.RemoveAt(viewSize - 1 - viewIndex);
				}
			}
		}
	}

	//remove point which visable images less than 2
	//const int pointSize(scene.pointcloud.GetSize());
	//for (size_t pointIndex = 0; pointIndex < pointSize; pointIndex++)
	//{
	//	if (scene.pointcloud.pointViews[pointSize - 1 - pointIndex].size() < 2)
	//	{
	//		scene.pointcloud.RemovePoint(pointSize - 1 - pointIndex);
	//	}
	//}
	return true;
}

bool ExportPmat(const Scene &scene,const std::string &pMatFilePath, const std::string &outPutPath)
{
	stlplus::folder_create(outPutPath + "\\CamA");
	stlplus::folder_create(outPutPath + "\\CamB");
	stlplus::folder_create(outPutPath + "\\CamC");
	stlplus::folder_create(outPutPath + "\\CamD");
	stlplus::folder_create(outPutPath + "\\CamE");

	for (int imageIndex = 0; imageIndex < scene.images.size(); imageIndex++)
	{
		std::string fileNameBase = stlplus::basename_part(scene.images[imageIndex].name);
		std::string srcFileName;
		std::string tarFileName;
		if (fileNameBase.substr(3,1)=="A")
		{
			srcFileName = pMatFilePath + "\\" + fileNameBase + ".txt";
			tarFileName = outPutPath + "\\CamA\\" + fileNameBase + ".txt";
		}
		else if (fileNameBase.substr(3, 1) == "B")
		{
			srcFileName = pMatFilePath + "\\" + fileNameBase + ".txt";
			tarFileName = outPutPath + "\\CamB\\" + fileNameBase + ".txt";
		}
		else if (fileNameBase.substr(3, 1) == "C")
		{
			srcFileName = pMatFilePath + "\\" + fileNameBase + ".txt";
			tarFileName = outPutPath + "\\CamC\\" + fileNameBase + ".txt";
		}
		else if (fileNameBase.substr(3, 1) == "D")
		{
			srcFileName = pMatFilePath + "\\" + fileNameBase + ".txt";
			tarFileName = outPutPath + "\\CamD\\" + fileNameBase + ".txt";
		}
		else if (fileNameBase.substr(3, 1) == "E")
		{
			srcFileName = pMatFilePath + "\\" + fileNameBase + ".txt";
			tarFileName = outPutPath + "\\CamE\\" + fileNameBase + ".txt";
		}
		else
		{
			std::cout << fileNameBase.substr(3, 1) << std::endl; getchar();
		}
		stlplus::file_copy(srcFileName, tarFileName);		
	}
	return true;
}

bool WritePmat(Scene &scene, const std::string &outPutPath)
{
	//stlplus::folder_create(outPutPath);
	for (int imageIndex = 0; imageIndex < scene.images.size(); imageIndex++)
	{
		auto Pmat = scene.images[imageIndex].GetCamera(scene.platforms, cv::Size(scene.images[imageIndex].width, scene.images[imageIndex].height)).P;
		//std::cout << outPutPath+"\\" + stlplus::basename_part(scene.images[imageIndex].name) + ".txt"; getchar();
		std::ofstream writer(outPutPath + "\\" + stlplus::basename_part(scene.images[imageIndex].name) + ".txt");
		writer << "CONTOUR" << std::endl;
		writer << std::setiosflags(std::ios::fixed) << std::setprecision(13);
		writer << Pmat(0, 0) << " " << Pmat(0, 1) << " " << Pmat(0, 2) << " " << Pmat(0, 3) << std::endl
			<< Pmat(1, 0) << " " << Pmat(1, 1) << " " << Pmat(1, 2) << " " << Pmat(1, 3) << std::endl
			<< Pmat(2, 0) << " " << Pmat(2, 1) << " " << Pmat(2, 2) << " " << Pmat(2, 3);
		writer.close();
	}
	return true;
}

int main(int argc, LPCTSTR* argv)
{
	//test
	{
	//	SceneDevide::imageWidth = 8176;
	//	SceneDevide::imageHeight = 6132;
		Scene scene;
		// load and estimate a dense point-cloud
		if (!scene.Load("F:\\MillerWorkPath\\mvsWorkPathAuto303\\block_12.mvs"))
		{
			std::cout << "failed to load scene" << std::endl;
			getchar();
			return EXIT_FAILURE;
		}
		//WritePmat(scene, "F:\\MillerWorkPath\\files\\Pmat303TransedScaled");
		ExportPmat(scene, "F:\\MillerWorkPath\\files\\Pmat303TransedScaled", "F:\\MillerWorkPath\\files\\pMat102");
		std::cout << "write Pmat finished" << std::endl; getchar();
	//	//std::cout << scene.mesh.faces.size() << std::endl; getchar();
	//	//SceneDevide::SimplicatePointCloud(0.05, scene);
	//	std::vector<int> imageIndexVec;
	//	SceneDevide::FliterRundantImage(9, scene,imageIndexVec);
	//	std::vector<int> imageIndexToRemove;
	//	for (int i = 0; i < scene.images.size(); i++)
	//	{
	//		if (std::find(imageIndexVec.begin(), imageIndexVec.end(), i)==imageIndexVec.end())
	//		{
	//			imageIndexToRemove.push_back(i);
	//		}
	//	}
	//	RemoveImages(scene, imageIndexToRemove);
	//	scene.Save("block12T.mvs");
	//	std::cout << "process finished" << std::endl; getchar();
	//	std::ofstream watcher("imageinfo2.txt");
	//	for (size_t i = 0; i < imageIndexVec.size(); i++)
	//	{
	//		watcher <<"valid image index: "<< imageIndexVec.at(i) << std::endl;
	//		std::string imageName = scene.images[imageIndexVec.at(i)].name;
	//		//std::cout << scene.images[imageIndexVec.at(i)].camera.R*Point3d(0,0,1) << std::endl; getchar();
	//		watcher << scene.images[imageIndexVec.at(i)].name << std::endl;
	//		cv::Mat image = cv::imread(scene.images[imageIndexVec.at(i)].name);
	//		std::cout << imageName.substr(1+imageName.find_last_of('/'), 12) << std::endl; /*getchar();*/
	//		cv::imwrite(std::string("image30\\") + imageName.substr(1 + imageName.find_last_of('/'), 12), image);
	//	}
	//	watcher.close();
	//	//scene.Save("sparse12.mvs");
	//	//scene.pointcloud.Save("sparsePoint.ply");
	//	std::cout << "simplicate point finished" << std::endl;
	//	getchar();
	}
	Scene scene;
	// load and estimate a dense point-cloud
	if (!scene.Load("F:\\MillerWorkPath\\openMVSWorkPath303\\scene_dense.mvs"))
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
	SceneDevide::SimplicatePointCloud(0.3, scene);
	std::cout << scene.images.size() << std::endl;
	SceneDevide::UniqueImageCamera(scene);
	SceneDevide processer(&scene);
	processer.workPath = "F:\\MillerWorkPath\\VSProject\\WorkPath303Dense";
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

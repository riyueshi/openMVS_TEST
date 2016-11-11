#include "SceneDevide.h"
#include "file_system.hpp"


using namespace std;
using namespace MVS;

//const int SceneDevide::imageWidth = 8176;
//const int SceneDevide::imageHeight = 6132;
const int SceneDevide::imageWidth = 4088;
const int SceneDevide::imageHeight = 3066;
//const int SceneDevide::imageWidth = 1022;
//const int SceneDevide::imageHeight = 766;

bool ShowImageInfo(const MVS::Scene &scene, std::string fileName)
{
	std::ofstream writer(fileName);
	for (auto imageIndexed = scene.images.begin(); imageIndexed != scene.images.end(); imageIndexed++)
	{
		imageIndexed->UpdateCamera(scene.platforms);
		writer << imageIndexed->name << imageIndexed->height << " " << imageIndexed->width << endl
			<< imageIndexed->camera.K << endl
			<< scene.platforms[imageIndexed->platformID].cameras[imageIndexed->cameraID].K << endl << endl;

		for (auto neightBorIndexed = imageIndexed->neighbors.begin(); neightBorIndexed != imageIndexed->neighbors.end(); neightBorIndexed++)
		{
			writer << neightBorIndexed->idx.ID << " ";
		}
		writer << endl;
	}
	return true;
}

SceneDevide::SceneDevide(MVS::Scene *sceneOri) :_pScene(sceneOri)
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
	//process one of the scene testblock
	{
		//apply the original params to created scene
		MVS::Scene scene;
		scene.platforms = (*_pScene).platforms;
		std::vector<Point2d> range;
		range.push_back(Point2d(-2.6054, -0.9664));
		range.push_back(Point2d(0.5879, 1.0688));
		const double avgHeight(0.3500);
		std::map<int, int> matcher;
		string imagePath("imageCrop\\");
		
		this->ImageCrop(range, imagePath, avgHeight, matcher, scene);
		this->PointCloudCrop(range, matcher, scene);

		cout << scene.images.begin()->name << endl;
		cout << scene.images.begin()->camera.K;
		scene.Save("test.mvs");
		scene.pointcloud.Save("test.ply");

		ShowImageInfo(scene, "sceneImageCroped.txt");
		ShowImageInfo(*_pScene, "sceneImage.txt");
		//for (size_t i = 0; i < _pScene->images[0].neighbors.size(); i++)
		//{
		//	cout << _pScene->images[_pScene->images[0].neighbors[i].idx.ID].name << endl;
		//	cout << scene.images[scene.images[0].neighbors[i].idx.ID].name << endl;
		//	getchar();
		//}
		//
		cout << "process finished" << endl;
		getchar();
	}
	return true;
}


bool SceneDevide::ImageCrop(
	const std::vector<Point2d>& range,
	const std::string & imagePath,
	const double & averageHeight,
	std::map<int, int>& matcher,
	MVS::Scene &scene)
{

	bool writeImageTag(true);
	vector<int> imageIndexToSave;

	const double areaThreshold(10000);

	vector<Vec4d> groundPointVec(4);
	{
		groundPointVec.at(0)[0] = range.at(0).x; groundPointVec.at(0)[1] = range.at(0).y; groundPointVec.at(0)[2] = averageHeight; groundPointVec.at(0)[3] = 1;
		groundPointVec.at(1)[0] = range.at(1).x; groundPointVec.at(1)[1] = range.at(0).y; groundPointVec.at(1)[2] = averageHeight; groundPointVec.at(1)[3] = 1;
		groundPointVec.at(2)[0] = range.at(1).x; groundPointVec.at(2)[1] = range.at(1).y; groundPointVec.at(2)[2] = averageHeight; groundPointVec.at(2)[3] = 1;
		groundPointVec.at(3)[0] = range.at(0).x; groundPointVec.at(3)[1] = range.at(1).y; groundPointVec.at(3)[2] = averageHeight; groundPointVec.at(3)[3] = 1;
	}

	using boost::geometry::append;
	using boost::geometry::correct;
	using boost::geometry::dsv;

	if (_pScene->images.size() == 0)
	{
		cout << "Error: no valid images in scene!" << endl;
		return false;
	}

	Polygon_2d imagePolygon;
	append(imagePolygon, MyPoint{ 0.0, 0.0 });
	append(imagePolygon, MyPoint{ double(imageWidth - 1),0.0 });
	append(imagePolygon, MyPoint{ double(imageWidth - 1),double(imageHeight - 1) });
	append(imagePolygon, MyPoint{ 0.0,double(imageHeight - 1) });
	correct(imagePolygon);

	for (size_t imageIndex = 0; imageIndex < _pScene->images.size(); imageIndex++)
	{
		vector<Vec3d> imagePointHVec(4);
		vector<MyPoint> imagePointVec;
		Image imageIndexed = _pScene->images[imageIndex]; //FIXME: this copy opeartion seam a big cost

		//update the camera and compose the project matrix

		//cout << imageIndexed.width << endl;
		//getchar();
		//imageIndexed.width = imageWidth;
		//imageIndexed.height = imageHeight;
		//imageIndexed.UpdateCamera(_pScene->platforms);
		//imageIndexed.camera.ComposeP();

		for (size_t i = 0; i < groundPointVec.size(); i++)
		{
			imagePointHVec.at(i) = imageIndexed.camera.P*groundPointVec.at(i);
			imagePointVec.push_back(MyPoint(imagePointHVec.at(i)[0] / imagePointHVec.at(i)[2], imagePointHVec.at(i)[1] / imagePointHVec.at(i)[2]));
		}

		Polygon_2d polygon;
		append(polygon, imagePointVec.at(0));
		append(polygon, imagePointVec.at(1));
		append(polygon, imagePointVec.at(2));
		append(polygon, imagePointVec.at(3));
		correct(polygon);

		std::vector<Polygon_2d> polys;
		if (boost::geometry::intersection(imagePolygon, polygon, polys))
		{
			if (polys.size() == 0)
			{
				continue;
			}

			Polygon_2d overlap = polys.at(0);
			vector<double> xVec, yVec;
			for (size_t i = 0; i < overlap.outer().size(); i++)
			{
				xVec.push_back(overlap.outer().at(i).x);
				yVec.push_back(overlap.outer().at(i).y);
			}
			int minX = *min_element(xVec.begin(), xVec.end());
			int minY = *min_element(yVec.begin(), yVec.end());
			int maxX = *max_element(xVec.begin(), xVec.end());
			int maxY = *max_element(yVec.begin(), yVec.end());

			//crop the image and update the images in the scene
			double xo = imageIndexed.camera.K(0, 2);
			double yo = imageIndexed.camera.K(1, 2);
			if ((maxX - minX) < 500 || (maxY - minY) < 500)
			{
				continue;
			}

			string imageName =string("F:/MillerWorkPath/mvsWorkPathGCP/")+ imageIndexed.name; //FIXME: image path specific
			string imageOutputName = imagePath + "/" + imageName.substr(imageName.find_last_of('/'), imageName.length() - imageName.find_last_of('/'));

			if (writeImageTag)
			{
				//cout << imageName << endl;
				//getchar();
				cv::Mat image = cv::imread(imageName);
				if (boost::geometry::area(overlap) < areaThreshold)
				{
					continue;
				}
				imageIndexToSave.push_back(imageIndex);
				matcher.insert(pair<int, int>(imageIndex, imageIndexToSave.size() - 1));
			//
			//cout << minX << endl << minY << endl << maxX << endl << maxY << endl;
			//
				cv::Mat subImage = image(cv::Rect(minX, minY, maxX - minX, maxY - minY));
				//creat output path
				if (!stlplus::folder_exists(imagePath + "\\"))
				{
					if (!stlplus::folder_create(imagePath + "\\"))
					{
						std::cerr << "\nCannot create output directory " << imagePath + "\\" << std::endl;
						return false;
					}
				}
				cv::imwrite(imageOutputName, subImage);
			}

			//cout << imageIndexed.name << " " << minX << " " << minY << endl;
			//cout << _pScene->platforms[imageIndexed.platformID].cameras[imageIndexed.cameraID].K(0, 2);

			double valueMax = maxX - minX > maxY - minY ? maxX - minX : maxY - minY;
			double deltX = (xo - minX) / valueMax - _pScene->platforms[imageIndexed.platformID].cameras[imageIndexed.cameraID].K(0, 2);
			double deltY = (yo - minY) / valueMax - _pScene->platforms[imageIndexed.platformID].cameras[imageIndexed.cameraID].K(1, 2);
			//double deltX = - minX / imageWidth ;
			//double deltY = - minY / imageWidth;


			//std::cout << imageIndexed.name << endl << imageIndexed.platformID << endl << imageIndexed.cameraID << endl; getchar();
			scene.platforms[imageIndexed.platformID].cameras[imageIndexed.cameraID].UpdatePrincipalPoint(Point2(deltX, deltY));

			//cout << scene.platforms[imageIndexed.platformID].cameras[imageIndexed.cameraID].K << endl; getchar();
			double focalOld = _pScene->platforms[imageIndexed.platformID].cameras[imageIndexed.cameraID].K(0, 0);
			double focalNew = focalOld*imageWidth / valueMax;	// FIXME: wether this expression hold for image which height is bigger than width
			scene.platforms[imageIndexed.platformID].cameras[imageIndexed.cameraID].UpdateFocalLengthAbs(focalNew);

			imageIndexed.width = maxX - minX;
			imageIndexed.height = maxY - minY;
			imageIndexed.UpdateCamera(scene.platforms);
			imageIndexed.name = imageOutputName;
			//imageIndexed.camera.ComposeP();

			//add image to the scene
			scene.images.push_back(imageIndexed);
		}
		else
		{
			continue;
		}
	}

	//update image's neighbor image
	//int imageIndex(0);
	//for (auto imageIndexed = scene.images.begin(); imageIndexed != scene.images.end(); imageIndexed++, imageIndex++)
	//{
	//	const int neighborSize = imageIndexed->neighbors.size();
	//	for (size_t neighborIndex = 0; neighborIndex < neighborSize; neighborIndex++)
	//	{
	//		auto &neighbor = imageIndexed->neighbors[neighborIndex];
	//		auto pos = matcher.find(neighbor.idx.ID);
	//		if (pos == matcher.end())
	//		{
	//			imageIndexed->neighbors.RemoveAt(neighborIndex);
	//		}
	//		else
	//		{
	//			//cout << _pScene->images[imageIndexed->neighbors[neighborIndex].idx.ID].name << endl
	//			//	<< scene.images[matcher.at(imageIndexed->neighbors[neighborIndex].idx.ID)].name << endl;
	//			neighbor.idx.ID = matcher.at(neighbor.idx.ID);
	//			//getchar();
	//		}
	//	}
	//}
	return true;
}

bool SceneDevide::PointCloudCrop(const std::vector<Point2d>& range, std::map<int, int>& matcher, MVS::Scene & scene)
{
	if (_pScene->pointcloud.GetSize()==0)
	{
		std::cout << "Error: Enmpty point cloud in scene" << endl;
		return false;
	}

	cout << endl << _pScene->pointcloud.points.size() << endl
		<< _pScene->pointcloud.pointViews.size() << endl
		<< _pScene->pointcloud.normals.size() << endl
		<< _pScene->pointcloud.colors.size() << endl
		<< _pScene->pointcloud.pointWeights.size() << endl << endl;

	if (_pScene->pointcloud.points.size() != _pScene->pointcloud.pointViews.size()||
		//_pScene->pointcloud.points.size() != _pScene->pointcloud.normals.size() ||
		_pScene->pointcloud.points.size() != _pScene->pointcloud.colors.size() ||
		_pScene->pointcloud.points.size() != _pScene->pointcloud.pointWeights.size() )
	{
		std::cout << "Error: Invalid point cloud in scene" << endl;
		return false;
	}

	//FIXME: compare which is faster iterater or [] operation
	auto pointView = _pScene->pointcloud.pointViews.begin();
	//auto pointNormal = _pScene->pointcloud.normals.begin();
	auto pointColor = _pScene->pointcloud.colors.begin();
	auto pointWeight = _pScene->pointcloud.pointWeights.begin();
	for (auto point = _pScene->pointcloud.points.begin(); point != _pScene->pointcloud.points.end(); ++point)
	{
		if (point->x > range.at(0).x&&
			point->x < range.at(1).x&&
			point->y > range.at(0).y&&
			point->y < range.at(1).y)
		{
			scene.pointcloud.points.push_back(*point);
			scene.pointcloud.pointViews.push_back(*pointView);
			//scene.pointcloud.normals.push_back(*pointNormal);
			scene.pointcloud.colors.push_back(*pointColor);
			scene.pointcloud.pointWeights.push_back(*pointWeight);
		}
		++pointView,/* ++pointNormal,*/ ++pointColor, ++pointWeight;

	}
	//for (auto pointView = scene.pointcloud.pointViews.begin(); pointView != scene.pointcloud.pointViews.end(); pointView++)
	//{
	//	const int viewSize = pointView->size();
	//	for (size_t viewIndex = 0; viewIndex < viewSize; viewIndex++)
	//	{
	//		auto pos = matcher.find(viewIndex);
	//		if (pos==matcher.end())
	//		{
	//			pointView->RemoveAt(viewIndex);
	//		}
	//		else
	//		{
	//			pointView[viewIndex] = matcher.at(viewIndex);
	//		}
	//	}
	//	//for (auto viewIndex = pointView->begin(); viewIndex != pointView->end(); viewIndex++)
	//	//{
	//	//	auto pos = matcher.find(*viewIndex);
	//	//	if (pos == matcher.end())
	//	//	{
	//	//		auto oneAfterCurrent = viewIndex++;
	//	//		viewIndex--;		//FIXME: a little stupid
	//	//		pointView->Remove(*viewIndex);
	//	//		viewIndex = oneAfterCurrent;
	//	//	}
	//	//	else
	//	//	{
	//	//		*viewIndex = matcher.at(*viewIndex);
	//	//	}
	//	//}
	//}
	return true;
}

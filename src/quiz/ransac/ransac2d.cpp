/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <cmath>
#include <unordered_set>

#include "../../render/render.h"
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	std::unordered_set<int> tempInliersResult;
	
	const int num_sample = 3;
	std::array<pcl::PointCloud<pcl::PointXYZ>::iterator, num_sample> samples{};

	// For max iterations 
	for(int iteration = 0; iteration < maxIterations; iteration++)
	{
		// Randomly sample subset and fit line
		while(tempInliersResult.size() < num_sample)
		{
			tempInliersResult.insert(rand() % cloud->size());
		}

		auto iter = tempInliersResult.begin();
		for(int i=0; i < num_sample; i++)
		{
			samples[i] = cloud->begin() + (*iter);
			iter++;
		}

		Eigen::Vector3f v1(samples[1]->x - samples[0]->x, samples[1]->y - samples[0]->y, samples[1]->z - samples[0]->z);
		Eigen::Vector3f v2(samples[2]->x - samples[0]->x, samples[2]->y - samples[0]->y, samples[2]->z - samples[0]->z);
		Eigen::Vector3f normal_vec = v1.cross(v2);

		float coefA = normal_vec[0];
		float coefB = normal_vec[1];
		float coefC = normal_vec[2];
		float coefD = -(normal_vec[0]*samples[0]->x + normal_vec[1]*samples[0]->y + normal_vec[2]*samples[0]->z);

		// Measure distance between every point and fitted line
		double distance = std::numeric_limits<double>::infinity(); 
		for(auto it_point = cloud->begin(); it_point != cloud->end(); it_point++)
		{
			distance = abs(coefA*it_point->x + coefB*it_point->y + coefC*it_point->z + coefD) / sqrt(coefA*coefA + coefB*coefB + coefC*coefC);

			// If distance is smaller than threshold count it as inlier
			if(distance < distanceTol)
			{
				tempInliersResult.insert(it_point - cloud->begin());
			}
		}

		if(tempInliersResult.size() > inliersResult.size())
		{
			inliersResult = std::move(tempInliersResult);
		}
		else 
		{
			tempInliersResult.clear();
		}
	}

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.2f);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spin();
  	}
  	
}

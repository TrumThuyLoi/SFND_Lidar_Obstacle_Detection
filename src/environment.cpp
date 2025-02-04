/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <memory>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#include "quiz/ransac/ransac2d.cpp"
#include "quiz/cluster/cluster.cpp"


std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    std::shared_ptr<Lidar> lidar = std::make_shared<Lidar>(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudData = lidar->scan();

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(cloudData, 100, 0.2f);
    renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(1, 1, 1));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        BoxQ boxQ = pointProcessor.minimalBoundingBox(cluster);
        renderBox(viewer, boxQ, clusterId);
        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    pcl::PointCloud<pcl::PointXYZI>::Ptr region_cloud = pointProcessorI.FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));

    std::unordered_set<int> plane_indices = Ransac<pcl::PointXYZI>(region_cloud, 50, 0.2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr non_plane_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<std::vector<float>> obstacle_points;

    for(int idx = 0; idx < region_cloud->size(); idx++)
	{
		if(plane_indices.count(idx))
        {
			plane_cloud->push_back(region_cloud->at(idx));
        }
        else 
        {
            auto ithPoint = region_cloud->at(idx);
            std::vector<float> point3d = {ithPoint.x, ithPoint.y, ithPoint.z};
            obstacle_points.push_back(point3d);
            non_plane_cloud->push_back(ithPoint);
        }
	}

    renderPointCloud(viewer,plane_cloud,"PlaneCloud", Color(1, 1, 1));

    KdTree<3>* kdTree = new KdTree<3>();
    for(int i=0; i < obstacle_points.size(); i++)
    {
        kdTree->insert(obstacle_points[i], i);
    }
    std::vector<pcl::PointIndices::Ptr> clusters_cloud = euclideanCluster<3>(obstacle_points, kdTree, 0.4);
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    for(int cluster_id = 0; cluster_id < clusters_cloud.size(); cluster_id++)
    {
        extract.setInputCloud(non_plane_cloud);
        extract.setIndices(clusters_cloud[cluster_id]);
        extract.setNegative(false);
        extract.filter(*cluster_cloud);

        LOG_INFO("cluster_cloud->size() = %lu", cluster_cloud->size());

        if(cluster_cloud->size() < 3)
            continue;

        renderPointCloud(viewer, cluster_cloud, "obstacleCloud" + std::to_string(cluster_id), colors[cluster_id % colors.size()]);
        
        Box box = pointProcessorI.BoundingBox(cluster_cloud);
        renderBox(viewer, box, cluster_id);
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while(viewer->wasStopped() == false)
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcl and run obstacle detection process
        inputCloudI = pointProcessorI.loadPcd(streamIterator->string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }

        viewer->spinOnce();
    }
}
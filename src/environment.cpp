/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <memory>

#include <pcl/common/pca.h>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
    // renderRays(viewer, lidar->position, cloudData);
    //renderPointCloud(viewer, cloudData, "Point Cloud", Color(1, 1, 1));

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(cloudData, 100, 0.2f);
    //renderPointCloud(viewer, segmentCloud.first, "nonPlaneCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(1, 1, 1));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId]);

        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*cluster, pcaCentroid);
        Eigen::Matrix3f covariance;
        computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        
        /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
        /// the signs are different and the box doesn't get correctly oriented in some cases.
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

        // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(cluster);
        pca.project(*cluster, *cloudPCAprojection);
        std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
        std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
        // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
        
        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
        // Get the minimum and maximum points of the transformed cloud.
        pcl::PointXYZ minPoint, maxPoint;
        pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
        const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

        // Final transform
        BoxQ boxQ = {
            eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>(),
            Eigen::Quaternionf(eigenVectorsPCA),    //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
            maxPoint.x - minPoint.x,
            maxPoint.y - minPoint.y,
            maxPoint.z - minPoint.z
        };
        // boxQ.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
        // boxQ.bboxQuaternion = Eigen::Vector3f(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI

        renderBox(viewer, boxQ, clusterId);
        ++clusterId;
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
    simpleHighway(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spin();
    } 
}
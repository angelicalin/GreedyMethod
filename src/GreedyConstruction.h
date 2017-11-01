#ifndef greedy_h
#define greedy_h

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include<pcl/io/obj_io.h>
//#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include<iostream>
#include<unordered_map>
#include <vector>

class GreedyConstruction {
public:
	GreedyConstruction();
	
	void run();

protected:
	Eigen::Vector3f normalResult;
	unsigned point_size;
	typedef pcl::PointXYZ PointType;
	pcl::PolygonMesh triangles;
	pcl::PointCloud <pcl::PointXYZ>objCloud;
	std::unordered_map<int, std::vector<int>> mapVertexToFaces;
	std::unordered_map<int, Eigen::Vector3f> mapVertexToNormal;
	std::unordered_map<int, bool> mapFaceToFlag;
	
	
	Eigen::Vector3f calculateNormal(int faceIndex, Eigen::Vector3f prevNorm);
	void recursiveFaceCalculation(int faceIndex, Eigen::Vector3f prevNormDirection);
	pcl::PolygonMesh reconstructModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};

#endif
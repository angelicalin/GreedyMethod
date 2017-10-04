#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include<pcl/io/obj_io.h>
#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include<iostream>


typedef pcl::PointXYZ PointType;

pcl::PolygonMesh reconstructModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<PointType> sor;
	sor.setInputCloud(originalCloud);
	sor.setLeafSize(1.0f, 1.0f, 1.0f);
	sor.filter(*cloud);*/

	// Normal estimation*
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(10);
	//n.setRadiusSearch(0.3);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures
	
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);
	

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	//remove(cloud);

	return triangles;

}


int
main(int argc, char** argv)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
	new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);
	//// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	
	pcl::io::loadPCDFile("milk.pcd", cloud_blob);

	//pcl::io::loadOBJFile("bunny.obj", cloud_blob);
	//	std::cout << "error loading file" << std::endl;
	//	return 0;
	//};frompclpointcloud2
	pcl::fromPCLPointCloud2(cloud_blob, *cloud);
	//* the data should be available in cloud
	// point cloud
//	pcl::PointCloud<PointType>::ConstPtr cloud;

	// Retrieved Point Cloud Callback Function
	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);

		/* Point Cloud Processing */

		cloud = ptr->makeShared();
	};

	// Kinect2Grabber
	//boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	// Register Callback Function
	//boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	//grabber->start();

	//while (!viewer->wasStopped()) {
	//	viewer->spinOnce();
	//	boost::mutex::scoped_try_lock lock(mutex);
	//	if (lock.owns_lock() && cloud) {
	//		pcl::PolygonMesh triangles;
	//		triangles = reconstructModel(cloud);
	//		if (!viewer->updatePolygonMesh(triangles, "tri")) {
	//			viewer->addPolygonMesh(triangles, "tri");
	//		}
	//		
	//	}
	//}
	
	
	pcl::PolygonMesh triangles;		
	triangles = reconstructModel(cloud);
	pcl::io::saveOBJFile("testMesh1.obj", triangles);

	
	//pcl::PointCloud<pcl::PointNormal>::Ptr outputCloud(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud <pcl::PointXYZ>objCloud;
	pcl::fromPCLPointCloud2(triangles.cloud, objCloud);
	unsigned point_size = static_cast<unsigned> (triangles.cloud.data.size() / (triangles.cloud.width * triangles.cloud.height));

	for (int i = 0; i < triangles.polygons.size(); i++) {
		pcl::Vertices currentPoly = triangles.polygons[i];

		
			pcl::PointXYZ currentPt0 = pcl::PointXYZ();
			currentPt0.x = objCloud[currentPoly.vertices[0]].x;
			currentPt0.y = objCloud[currentPoly.vertices[0]].y;
			currentPt0.z = objCloud[currentPoly.vertices[0]].z;
			pcl::PointXYZ currentPt1 = pcl::PointXYZ();
			currentPt1.x = objCloud[currentPoly.vertices[0]].x;
			currentPt1.y = objCloud[currentPoly.vertices[0]].y;
			currentPt1.z = objCloud[currentPoly.vertices[0]].z;
			pcl::PointXYZ currentPt2 = pcl::PointXYZ();
			currentPt2.x = objCloud[currentPoly.vertices[0]].x;
			currentPt2.y = objCloud[currentPoly.vertices[0]].y;
			currentPt2.z = objCloud[currentPoly.vertices[0]].z;


		
		Eigen::Vector3f vec12(currentPt1.x - currentPt0.x, currentPt1.y - currentPt0.y, currentPt1.z - currentPt0.z);
		Eigen::Vector3f vec23(currentPt2.x - currentPt1.x, currentPt2.y - currentPt1.y, currentPt2.z - currentPt1.z);
		Eigen::Vector3f vecNorm = -vec12.cross(vec23);
		vecNorm.normalize();
		for (int ii = 0; ii < 3; ii++)
		{
			
			/*outputCloud->points[index - ii].normal_x = vecNorm[0];
			outputCloud->points[index - ii].normal_y = vecNorm[1];
			outputCloud->points[index - ii].normal_z = vecNorm[2];*/
			int nxyz = 0;
			for (size_t d = 0; d < triangles.cloud.fields.size(); ++d) {
				if (triangles.cloud.fields[d].name == "normal_x" || 
					triangles.cloud.fields[d].name == "normal_y" ||
					triangles.cloud.fields[d].name == "normal_z") {
					float a = 0.0;
					memcpy(&triangles.cloud.data[(currentPoly.vertices[ii]) * point_size + triangles.cloud.fields[d].offset],&a, sizeof(float));
				
					//std::cout << "going once" << std::endl;
					if (++nxyz == 3)  break;
				}
				 
			}
		}
	}
	pcl::io::saveOBJFile("testMesh2.obj", triangles);
	return (0);
}
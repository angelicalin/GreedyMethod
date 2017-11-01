#include"GreedyConstruction.h"



GreedyConstruction::GreedyConstruction() {}

void GreedyConstruction::run()
{
	//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	//	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);
	//// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PCLPointCloud2 cloud_blob;

	//pcl::io::loadPCDFile("milk.pcd", cloud_blob);

	pcl::io::loadOBJFile("bunny.obj", cloud_blob);
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

	// Kinect2Grabber related code
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

	triangles = reconstructModel(cloud);
	pcl::fromPCLPointCloud2(triangles.cloud, objCloud);
	pcl::io::saveOBJFile("testMesh1.obj", triangles);

	//finding the center of the mesh
	float xcent = 0;
	float ycent = 0;
	float zcent = 0;
	for (int i = 0; i < triangles.cloud.width; i++) {
		xcent += objCloud[i].x;
		ycent += objCloud[i].y;
		zcent += objCloud[i].z;
	}
	xcent = xcent / triangles.cloud.width;
	ycent = ycent / triangles.cloud.width;
	zcent = zcent / triangles.cloud.width;

	//building the map that each vertex index map to a list of integer that is the face index that the vertex is on
	//map<vertex index, face index[]>,


	//initilize a centroid for each face
	Eigen::Vector3f centroid(0.0, 0.0, 0.0);
	Eigen::Vector3f centerOfMesh(xcent, ycent, zcent);
	int startingIndex;
	float currentMaxDistance = 0.0;
	Eigen::Vector3f startingFaceCentroid(0.0, 0.0, 0.0);

	//Recursively recalculating the normals

	//Problem:: It seems that the starting point cannot reach to the entire mesh because it is not connected?
	//But how to find the next starting point?
	//The normal is changed after the process but it is really off. Not much improvement from the beginning
	//Also, can the recursion be incorrect??
	//TODO:: fix the recursion and the while loop here, and see if there is a better way to do this

	//Build up the Vertex-Face map
	for (int i = 0; i < triangles.polygons.size(); i++) {
		//Only checking the faces that have not been calculated
		//if (mapFaceToFlag.find(i) == mapFaceToFlag.end()) {
		pcl::Vertices currentPoly = triangles.polygons[i];
		//currentPoly.vertices[0] will return the index of the vertex
		for (int j = 0; j < currentPoly.vertices.size(); j++) {
			std::unordered_map<int, std::vector<int>>::const_iterator findResult = mapVertexToFaces.find(currentPoly.vertices[j]);
			if (findResult == mapVertexToFaces.end()) {
				//not found
				std::vector<int> v = { i };
				mapVertexToFaces.insert(std::pair<int, std::vector<int>>(currentPoly.vertices[j], std::vector<int>{i}));
				mapVertexToNormal.insert(std::pair<int, Eigen::Vector3f>(currentPoly.vertices[j], Eigen::Vector3f(0.0, 0.0, 0.0)));
			}
			else {
				//found
				std::vector<int> s = findResult->second;
				s.push_back(i);
				mapVertexToFaces[currentPoly.vertices[j]] = s;
			}

		}

	}

	//Find the starting face
	while (mapFaceToFlag.size() != triangles.polygons.size()) {
		startingIndex = 0;
		currentMaxDistance = 0.0;
		startingFaceCentroid = Eigen::Vector3f(0.0, 0.0, 0.0);
		for (int i = 0; i < triangles.polygons.size(); i++) {
			centroid = 0 * centroid;
			if (mapFaceToFlag.find(i) == mapFaceToFlag.end()) {
				for (int j = 0; j < triangles.polygons[i].vertices.size(); j++) {
					centroid = centroid + Eigen::Vector3f(objCloud[triangles.polygons[i].vertices[j]].x, objCloud[triangles.polygons[i].vertices[j]].y, objCloud[triangles.polygons[i].vertices[j]].z);
				}
				centroid = centroid / 3.0;
				if ((centroid - centerOfMesh).squaredNorm() > currentMaxDistance) {
					startingIndex = i;
					startingFaceCentroid = centroid;
					currentMaxDistance = (centroid - centerOfMesh).squaredNorm();
				}
			}
		}
		recursiveFaceCalculation(startingIndex, startingFaceCentroid - centerOfMesh);

		std::cout << "going once" << std::endl;
	}

	point_size = static_cast<unsigned> (triangles.cloud.data.size() / (triangles.cloud.width * triangles.cloud.height));

	for (int i = 0; i < triangles.cloud.width; i++) {
		size_t d = 0;
		while (triangles.cloud.fields[d].name != "normal_x") {
			++d;
		}
		std::unordered_map<int, Eigen::Vector3f>::const_iterator findResult = mapVertexToNormal.find(i);
		if (findResult == mapVertexToNormal.end()) {
			std::cout << "calculating error, normal not found" << std::endl;
		}
		else {
			//found normal
			normalResult = findResult->second;
			normalResult.normalize();
			memcpy(&triangles.cloud.data[i*point_size + triangles.cloud.fields[d].offset], &normalResult[0], sizeof(float));
			memcpy(&triangles.cloud.data[i*point_size + triangles.cloud.fields[d + 1].offset], &normalResult[1], sizeof(float));
			memcpy(&triangles.cloud.data[i*point_size + triangles.cloud.fields[d + 2].offset], &normalResult[2], sizeof(float));
		}
	}



	pcl::io::saveOBJFile("testMesh2.obj", triangles);

}
pcl::PolygonMesh GreedyConstruction::reconstructModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

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

Eigen::Vector3f GreedyConstruction::calculateNormal(int faceIndex, Eigen::Vector3f prevNorm) {
	Eigen::Vector3f point0(objCloud[triangles.polygons[faceIndex].vertices[0]].x, objCloud[triangles.polygons[faceIndex].vertices[0]].y, objCloud[triangles.polygons[faceIndex].vertices[0]].z);
	Eigen::Vector3f point1(objCloud[triangles.polygons[faceIndex].vertices[1]].x, objCloud[triangles.polygons[faceIndex].vertices[1]].y, objCloud[triangles.polygons[faceIndex].vertices[1]].z);
	Eigen::Vector3f point2(objCloud[triangles.polygons[faceIndex].vertices[2]].x, objCloud[triangles.polygons[faceIndex].vertices[2]].y, objCloud[triangles.polygons[faceIndex].vertices[2]].z);
	Eigen::Vector3f vecNorm = (point1 - point0).cross(point2 - point1);
	float angleCosVal = vecNorm.dot(prevNorm);
	//Check if the normal is at the correct direction
	if (angleCosVal < 0) { vecNorm = (-vecNorm); }
	for (int x = 0; x < 3; x++) {
		//std::unordered_map<int, Eigen::Vector3f>::const_iterator findNormalSaved = ;
		mapVertexToNormal[triangles.polygons[faceIndex].vertices[x]] = ( mapVertexToNormal.find(triangles.polygons[faceIndex].vertices[x])->second + vecNorm);
	}
	//Take note that the face has been calculated
	mapFaceToFlag.insert(std::pair<int, bool>(faceIndex, true));
	return vecNorm;
}

void GreedyConstruction::recursiveFaceCalculation(int faceIndex, Eigen::Vector3f prevNormDirection) {
	Eigen::Vector3f vecNormResult = calculateNormal(faceIndex, prevNormDirection);
	//more than one vertex would start causing stack overflow, so only using vertices[0]

		std::unordered_map<int, std::vector<int>>::const_iterator findAdjacentFaces = mapVertexToFaces.find(triangles.polygons[faceIndex].vertices[0]);
		for (int j : findAdjacentFaces->second) {
			if (mapFaceToFlag.find(j) == mapFaceToFlag.end()) {
				//We haven't calculate the vertex yet
				recursiveFaceCalculation(j, vecNormResult);
			}
		}
}


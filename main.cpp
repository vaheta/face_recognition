#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/obj_io.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <math.h>
using namespace std;

#define DIST(x1,y1,z1,x2,y2,z2) (sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2)))

void ftt_reader (string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB &nose_tip) {
	string line;
	double temp;
	int itemp;
	char stemp[256];
	int counter = 0;
	ifstream myfile(filename.c_str());
	if (myfile.is_open()) {
		while (getline(myfile, line)) {
			if (sscanf(line.c_str(), "numFeatures=%d", &itemp) == 1) 
				cout << "numFeatures = " << itemp << endl;
			double x,y,z;
			int t2;
			string flag = " 3D ";
			string start = "(";
			string end = ")";
			if (line.find(flag) != string::npos) {
				string vec = line.substr(line.find(start)+1, line.find(end)-1-line.find(start));
				sscanf(vec.c_str(), "%lf %lf %lf", &x,&y,&z);
				pcl::PointXYZRGB point;
				point.x = x;
				point.y = y;
				point.z = z;
				cloud->points.push_back(point);
				if (counter == 16) {
					nose_tip.x = x;
					nose_tip.y = y;
					nose_tip.z = z;	
				}
				counter = counter + 1;
				if (counter >= 60) {
					break;
				}
			}
		}
		myfile.close();
	} else {
		printf("Couldn't open %s.\n", filename.c_str());
	}
}

void croppc (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointXYZRGB nose_tip, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out) {
	for (int i=0; i<cloud_in->size(); i++) {
		if (DIST(cloud_in->points[i].x, cloud_in->points[i].y, cloud_in->points[i].z, nose_tip.x, nose_tip.y, nose_tip.z) <= 100)
			cloud_out->points.push_back(cloud_in->points[i]);
	}
}

void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose) {
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(10, 10, 10);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

int main(int argc, char** argv) {
	// load 2 PCs
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_0 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2_0 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2_tr (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_1_sp (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_2_sp (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB nose_tip_1;
	pcl::PointXYZRGB nose_tip_2;
	string filename_1 = string(argv[1]) + string(".obj");
	string filename_1_sp = string(argv[1]) + ".ftt";
	string filename_2 = string(argv[2]) + ".obj";
	string filename_2_sp = string(argv[2]) + ".ftt";
	pcl::io::loadOBJFile (filename_1, *cloud_1_0);
	pcl::io::loadOBJFile (filename_2, *cloud_2_0);
	ftt_reader(filename_1_sp, cloud_1_sp, nose_tip_1);
	ftt_reader(filename_2_sp, cloud_2_sp, nose_tip_2);
	
	croppc(cloud_1_0, nose_tip_1, cloud_1);
	croppc(cloud_2_0, nose_tip_2, cloud_2);

	// visualize them

	for (int i=0; i<cloud_1->size(); i++) {
		cloud_1->points[i].r = 255;
		cloud_1->points[i].g = 0;
		cloud_1->points[i].b = 0;
	}
	for (int i=0; i<cloud_2->size(); i++) {
		cloud_2->points[i].r = 255;
		cloud_2->points[i].g = 0;
		cloud_2->points[i].b = 0;
	}
	for (int i=0; i<cloud_1_sp->size(); i++) {
		cloud_1_sp->points[i].r = 255;
		cloud_1_sp->points[i].g = 0;
		cloud_1_sp->points[i].b = 0;
	}
	for (int i=0; i<cloud_2_sp->size(); i++) {
		cloud_2_sp->points[i].r = 0;
		cloud_2_sp->points[i].g = 0;
		cloud_2_sp->points[i].b = 255;
	}

	// transform clouds so that nose tip is at 0.
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	transform_1(0,3) = -nose_tip_1.x;
	transform_1(1,3) = -nose_tip_1.y;
	transform_1(2,3) = -nose_tip_1.z;
	pcl::transformPointCloud(*cloud_1, *cloud_1, transform_1);
	pcl::transformPointCloud(*cloud_1_sp, *cloud_1_sp, transform_1);

	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
	transform_2(0,3) = -nose_tip_2.x;
	transform_2(1,3) = -nose_tip_2.y;
	transform_2(2,3) = -nose_tip_2.z;
	pcl::transformPointCloud(*cloud_2, *cloud_2, transform_2);
	pcl::transformPointCloud(*cloud_2_sp, *cloud_2_sp, transform_2);

	// align them using SVD
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB> TESVD;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGB,pcl::PointXYZRGB>::Matrix4 transformation;
	TESVD.estimateRigidTransformation (*cloud_2_sp,*cloud_1_sp,transformation);
	transformation(0,3) = 0;
	transformation(1,3) = 0;
	transformation(2,3) = 0;

	pcl::transformPointCloud (*cloud_2, *cloud_2, transformation);
	pcl::transformPointCloud (*cloud_2_sp, *cloud_2_sp, transformation);

	// visualize the result
	// pcl::visualization::PCLVisualizer viewer;
	// viewer.setBackgroundColor(1, 1, 1);
	// while (!viewer.wasStopped ()){
	// 	viewer.removeAllPointClouds();

	// 	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_1_sp);
	// 	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_1_sp, rgb, "pc");
	// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc");

	// 	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(cloud_2_sp);
	// 	viewer.addPointCloud<pcl::PointXYZRGB>(cloud_2_sp, rgb1, "target");
	// 	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target");
	// 	viewer.spinOnce(30);
	// }

	// kdtrees
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud (cloud_1);
	double weight = 0;
	for (int i=0; i<cloud_2->size(); i++) {
		int K=1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		if ( kdtree.nearestKSearch (cloud_2->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ) {
			int color = pointNKNSquaredDistance[0] * 255 / 20;
			weight = weight + pointNKNSquaredDistance[0]/double(cloud_2->size());
			if (color >255)
				color = 255;
			cloud_2->points[i].r = color;
			cloud_2->points[i].g = 0;
			cloud_2->points[i].b = 0;
		}
	}

	cout << "Weight = " << weight << endl;
	

	// pcl::visualization::PCLVisualizer viewer1;
	// viewer1.setBackgroundColor(1, 1, 1);
	// viewer1.addCoordinateSystem (50.0);
	// while (!viewer1.wasStopped ()){
	// 	viewer1.removeAllPointClouds();
	// 	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_2);
	// 	viewer1.addPointCloud<pcl::PointXYZRGB>(cloud_2, rgb, "pc");
	// 	viewer1.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pc");
	// 	viewer1.spinOnce(30);
	// }

	return 0;
}
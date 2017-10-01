#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
// PCL specific includes
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <iostream>
#include <boost/thread/thread.hpp>

ros::Publisher pub_pts,pub_obj;
int iter=0;

typedef pcl::PointXYZ PointT;

struct Scalar
{
  int r;
  int g;
  int b;
};

double x_min=0.0;
double x_max=30.0;
double y_min=-10.0; 
double y_max=10.0;
double z_min=-3.0;
double z_max=0.0;
double val_vox= 0.05f;
double DistanceThreshold= 0.2;
double val_dist = 0.5;
double val_radius = 0.3;
int minNeighbor = 10;
int val_minpt = 20;
bool doFILTER = true;

void publishCloudsData(pcl::PointCloud<PointT>::Ptr point_cloud_ptr, pcl::PointCloud<pcl::PointXYZI> &coloured_point_cloud, int idx, visualization_msgs::MarkerArray &markers)
{ 
  //assign cloud idx
  for (int i=0;i<point_cloud_ptr->size();i++)
  {
    pcl::PointXYZ pt = point_cloud_ptr->points[i];
    pcl::PointXYZI pt2;
    pt2.x = pt.x, pt2.y = pt.y, pt2.z = pt.z;
    pt2.intensity = float(idx);

    coloured_point_cloud.push_back(pt2);
  }

  // calculate min max pt
  visualization_msgs::Marker marker;

  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*point_cloud_ptr, min_pt, max_pt);
  geometry_msgs::Point max_point,min_point;
  max_point.x = max_pt.x; max_point.y = max_pt.y; max_point.z = max_pt.z;
  min_point.x = min_pt.x; min_point.y = min_pt.y; min_point.z = min_pt.z;
  marker.points.push_back(max_point);
  marker.points.push_back(min_point);

  double center_x=(min_pt.x+max_pt.x)/2.0;
  double center_y=(min_pt.y+max_pt.y)/2.0;
  double dist = sqrt(center_x*center_x+center_y*center_y);
  
  std::string obj_dist="";
  std::stringstream ss;
  ss << dist;
  obj_dist=ss.str();
  
  
  marker.header.frame_id = "/Sensor";
  marker.header.stamp = ros::Time::now();
  marker.ns = "object";
  marker.id = idx;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center_x;
  marker.pose.position.y = center_y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0f;
  marker.scale.y = 1.0f;
  marker.scale.z = 1.0f;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.5);
  marker.text = obj_dist;
  
  markers.markers.push_back(marker);
}
void cloud_cb (const sensor_msgs::PointCloud2 input)
{
	iter++;
	std::string cloudname="";
	std::stringstream ss;
	ss << iter;
	cloudname += "cloud_" + ss.str() + ".pcd";
    //ROS_INFO("%s", cloudname.c_str());

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // creates a shared pointer
	pcl::fromROSMsg(input, *cloud);

	// All the objects needed
	pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;

	pcl::PCDWriter writer;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_noise(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_xy(new pcl::PointCloud<PointT>);

	// -------------------------------------------------
	// -----Create coloured point cloud for viewer -----
	// -------------------------------------------------
	pcl::PointCloud<pcl::PointXYZI> coloured_point_cloud;  

	// -------------------------------------------------
	// Create the filtering object
	// -------------------------------------------------
	pcl::VoxelGrid<pcl::PointXYZ> vox;	

	if (doFILTER)
	{

		// passthrough filter to remove spurious NaNs
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("x"); // z
		pass.setFilterLimits(x_min, x_max);
		pass.filter(*cloud_voxel);

		pass.setInputCloud(cloud_voxel);
		pass.setFilterFieldName("y"); // z
		pass.setFilterLimits(y_min, y_max);
		pass.filter(*cloud_voxel);

		pass.setInputCloud(cloud_voxel);
		pass.setFilterFieldName("z"); // z
		pass.setFilterLimits(z_min, z_max);
		pass.filter(*cloud_voxel);

		//voxel filtering
		vox.setInputCloud(cloud_voxel);
		vox.setLeafSize(val_vox, val_vox, val_vox);
		vox.filter(*cloud_filtered);
		
	}
	else
	{
		pcl::copyPointCloud(*cloud, *cloud_filtered);
		//cloud_filtered.swap(cloud_voxel);
	}
	pcl::copyPointCloud(*cloud_filtered, *cloud_voxel);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// -------------------------------------------------
	// Extract plane points
	// -------------------------------------------------

	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(DistanceThreshold);

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	return ;
	}


	// Extract the plane points
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud_filtered2);

	// remove noise points
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
	outrem.setInputCloud(cloud_filtered2);
	outrem.setRadiusSearch(val_radius);
	outrem.setMinNeighborsInRadius(minNeighbor);
	outrem.filter(*cloud_noise);

	// -------------------------------------------------
	// Calculate minimum bounding boxes
	// -------------------------------------------------
	cloud_xy->points.resize(cloud_noise->points.size());
	for (size_t i = 0; i < cloud_noise->points.size(); i++)
	{
		cloud_xy->points[i].x = cloud_noise->points[i].x;
		cloud_xy->points[i].y = cloud_noise->points[i].y;
		cloud_xy->points[i].z = 0;
	}

	// Creating the KdTree object for the search method of the extraction
	tree->setInputCloud(cloud_xy);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(val_dist); // 2cm
	ec.setMinClusterSize(val_minpt);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_xy);
	ec.extract(cluster_indices);


    visualization_msgs::MarkerArray markers;
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloud_xy->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		j++;
        publishCloudsData(cloud_cluster, coloured_point_cloud, j, markers);
	}


	// Convert To ROS data type   
	sensor_msgs::PointCloud2 output;  
	pcl::PCLPointCloud2 cloud_p;
	pcl::toPCLPointCloud2(coloured_point_cloud, cloud_p); 

	pcl_conversions::fromPCL(cloud_p, output);
	output.header.frame_id = "Sensor";    
    pub_pts.publish(output);

    // Publish markers
    pub_obj.publish(markers);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "clustering");
  ros::NodeHandle nh;

  // Set param  
  nh.getParam("/object_detection/x_min", x_min);
  nh.getParam("/object_detection/x_max", x_max);
  nh.getParam("/object_detection/y_min", y_min);
  nh.getParam("/object_detection/y_max", y_max);
  nh.getParam("/object_detection/z_min", z_min);
  nh.getParam("/object_detection/z_max", z_max);
  nh.getParam("/object_detection/val_vox", val_vox);
  nh.getParam("/object_detection/DistanceThreshold", DistanceThreshold);
  nh.getParam("/object_detection/val_dist", val_dist);
  nh.getParam("/object_detection/val_radius", val_radius);
  nh.getParam("/object_detection/minNeighbor", minNeighbor);
  nh.getParam("/object_detection/val_minpt", val_minpt);
  nh.getParam("/object_detection/doFILTER", doFILTER);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("Sensor/points", 1, cloud_cb);

  pub_pts = nh.advertise<sensor_msgs::PointCloud2> ("Sensor/clustered_points", 100);
  pub_obj = nh.advertise<visualization_msgs::MarkerArray>("Sensor/visualization_objects", 100);
  
  ROS_INFO("Clustering Start");  
  // Spin
  ros::spin ();
}

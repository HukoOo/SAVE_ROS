#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
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

ros::Publisher pub_pts,pub_curve;
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


void publishCloudsData(pcl::PointCloud<PointT>::Ptr point_cloud_ptr, pcl::PointCloud<pcl::PointXYZI> &coloured_point_cloud, int idx)
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

}

void cloud_cb (const sensor_msgs::PointCloud2 input)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>); // creates a shared pointer
	pcl::fromROSMsg(input, *cloud);

	// All the objects needed
	pcl::PassThrough<PointT> pass;
	pcl::ExtractIndices<PointT> extract;
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_voxel(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_outliers(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_noise(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_xy(new pcl::PointCloud<PointT>);

	// -------------------------------------------------
	// -----Create coloured point cloud for viewer -----
	// -------------------------------------------------
	pcl::PointCloud<pcl::PointXYZI> coloured_point_cloud;  

	// Creating the KdTree object for the search method of the extraction
	visualization_msgs::MarkerArray markers;
	if(cloud->points.size()>0)
	{
		tree->setInputCloud(cloud);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(0.1); 
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(150);
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);
		ec.extract(cluster_indices);


		int j = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				cloud_cluster->points.push_back(cloud->points[*pit]);
			cloud_cluster->width = cloud_cluster->points.size();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			// calculate min max pt        
            pcl::PointXYZ max_point,min_point;
			pcl::getMinMax3D(*cloud_cluster, min_point, max_point);
			
			double max_y=1.5, min_y=-1.5;
			geometry_msgs::Point min_pt, max_pt;
			max_pt.x = max_point.x; max_pt.y = max_point.y;
			min_pt.x = min_point.x; min_pt.y = min_point.y; 
			geometry_msgs::Point pt_r[2],pt_l[2];
			pt_l[0] = max_pt;
			pt_l[1].x = min_pt.x; pt_l[1].y = max_pt.y;
			pt_r[0] = min_pt;
			pt_r[1].x = max_pt.x; pt_r[1].y = min_pt.y;	

			double center_x=(min_pt.x+max_pt.x)/2.0;
			double center_y=(min_pt.y+max_pt.y)/2.0;

			if((center_y <= max_y && pt_l[0].y >= min_y &&
				center_x <= 20 && center_x >0))
			 {
				 
				j++;
				publishCloudsData(cloud_cluster, coloured_point_cloud, j);
			 
				 
				 visualization_msgs::Marker marker;         
				 marker.points.push_back(max_pt);
				 marker.header.frame_id = "/Sensor";
				 marker.header.stamp = ros::Time::now();
				 marker.ns = "center_pt";
				 marker.id = j;
				 marker.type = visualization_msgs::Marker::SPHERE;
				 marker.action = visualization_msgs::Marker::ADD;
				 marker.pose.position.x = center_x;
				 marker.pose.position.y = center_y;
				 marker.pose.position.z = 0;
				 marker.pose.orientation.x = 0.0;
				 marker.pose.orientation.y = 0.0;
				 marker.pose.orientation.z = 0.0;
				 marker.pose.orientation.w = 1.0;
				 marker.scale.x = 0.3f;
				 marker.scale.y = 0.3f;
				 marker.scale.z = 0.3f;
				 marker.color.r = 1.0f;
				 marker.color.g = 1.0f;
				 marker.color.b = 0.0f;
				 marker.color.a = 1.0;
				 marker.lifetime = ros::Duration(0.2);
				 
				 markers.markers.push_back(marker);
			 }
		}
	}

	// Convert To ROS data type   
	sensor_msgs::PointCloud2 output;  
	pcl::PCLPointCloud2 cloud_p;

	pcl::toPCLPointCloud2(coloured_point_cloud, cloud_p); 
	pcl_conversions::fromPCL(cloud_p, output);
	output.header.frame_id = "Sensor";    
    pub_curve.publish(output);


	// Publish markers
    pub_pts.publish(markers);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "clustering");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("Sensor/road_points", 1, cloud_cb);

  pub_curve= nh.advertise<sensor_msgs::PointCloud2> ("Sensor/clustered_curve", 100);
  pub_pts = nh.advertise<visualization_msgs::MarkerArray> ("Sensor/center_points", 100);
  
  ROS_INFO("Curve Clustering Start");  
  // Spin
  ros::spin ();
}

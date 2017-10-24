#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <boost/thread/thread.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int pt_count=0;
int left_count=0;
int right_count=0;

void cloud_cb (const sensor_msgs::PointCloud2 input)
{
  pt_count++;
  std::string cloudname="";
  std::stringstream ss;
  ss << pt_count;
  cloudname += "cloud_" + ss.str() + ".pcd";
  ROS_INFO("%s", cloudname.c_str());

  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZ>); // creates a shared pointer
  pcl::fromROSMsg(input, *pclCloud);
  //pcl::io::savePCDFileBinaryCompressed(cloudname, *pclCloud);
}

void image_left_cb (const sensor_msgs::ImageConstPtr& msg)
{
  left_count++;
  std::string image_name="";
  std::stringstream ss;
  ss << left_count;
  image_name += "left_" + ss.str() + ".bmp";
  ROS_INFO("%s", image_name.c_str());

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //cv::imwrite(image_name, cv_ptr->image);

}
void image_right_cb (const sensor_msgs::ImageConstPtr& msg)
{
  right_count++;
  std::string image_name="";
  std::stringstream ss;
  ss << right_count;
  image_name += "right_" + ss.str() + ".bmp";
  ROS_INFO("%s", image_name.c_str());

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imwrite(image_name, cv_ptr->image);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "writer");
  ros::NodeHandle nh;

  ros::Subscriber pt_sub = nh.subscribe ("Sensor/points", 1, cloud_cb);
  ros::Subscriber cv_sub1 = nh.subscribe ("stereo/left/image_raw", 1, image_left_cb);
  ros::Subscriber cv_sub2 = nh.subscribe ("stereo/right/image_raw", 1, image_right_cb);

   ROS_INFO("Writing Start");  
  // Spin
  ros::spin ();

}

#include <ros/ros.h>
#include <tf/transform_listener.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <teb_local_planner/SKKU_LaneArray.h>
#include <teb_local_planner/SKKU_Lane.h>

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

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#define PI 3.14159265
ros::Publisher filtered_obj,lane_pub,arrow_pub;

//std::ofstream fs1;
//std::ofstream fs2;

double lane_pts[2][3] {     // x=3,15,50 meter
    {1, 1, 1},              //left
    {-1, -1, -1}            //right
  };

class BoundBox
{
public:
    BoundBox(double _x, double _y, double _width, double _height):x(0),y(0),width(0),height(0)
    {
       x=_x;
       y=_y;
       width=_width;
       height=_height;
    }
public:
    double x;
    double y;
    double width;
    double height;
};

double roi_x = 1.0;
double roi_y = 2.0;

BoundBox roi_l(-roi_x,0,roi_x*2,roi_y),roi_r(-roi_x, -roi_y, roi_x*2, roi_y);

bool valueInRange(int value, int min, int max)
{ return (value >= min) && (value <= max); }

bool rectOverlap(BoundBox A, BoundBox B)
{
    bool xOverlap = valueInRange(A.x, B.x, B.x + B.width) ||
                    valueInRange(B.x, A.x, A.x + A.width);

    bool yOverlap = valueInRange(A.y, B.y, B.y + B.height) ||
                    valueInRange(B.y, A.y, A.y + A.height);

    return xOverlap && yOverlap;
}

void marker_cb(const visualization_msgs::MarkerArray _markers)
{
  visualization_msgs::MarkerArray markers_obj;

  teb_local_planner::SKKU_Lane wall_interval;

  int num_input_markers = _markers.markers.size();
  int num_filtered_markers=0;
  for(int i=0;i<num_input_markers;i++)
  {
    ///////////////// Detect front object distance /////////////////////////////////
    visualization_msgs::Marker marker_obj;

    geometry_msgs::Point min_pt, max_pt;
    // calculate two line equations
    double m[2]={0,0};
    double b[2]={0,0};
    double y;
    m[0] = (lane_pts[0][0]-lane_pts[0][1])/(3.0-50.0);
    m[1] = (lane_pts[1][0]-lane_pts[1][1])/(3.0-50.0);
    b[0] = lane_pts[0][0]-m[0]*3.0;
    b[1] = lane_pts[1][0]-m[1]*3.0;


    // extract the objects between the lanes

    max_pt = _markers.markers[i].points[0];
    min_pt = _markers.markers[i].points[1];
    geometry_msgs::Point pt_r[2],pt_l[2];
    pt_l[0] = max_pt;
    pt_l[1].x = min_pt.x; pt_l[1].y = max_pt.y;
    pt_r[0] = min_pt;
    pt_r[1].x = max_pt.x; pt_r[1].y = min_pt.y;

    if((pt_l[0].y <= m[0]*pt_l[0].x+b[0] && pt_l[0].y >= m[1]*pt_l[0].x+b[1]) ||
       (pt_l[1].y <= m[0]*pt_l[1].x+b[0] && pt_l[1].y >= m[1]*pt_l[1].x+b[1]) ||
       (pt_r[0].y <= m[0]*pt_r[0].x+b[0] && pt_r[0].y >= m[1]*pt_r[0].x+b[1]) ||
       (pt_r[1].y <= m[0]*pt_r[1].x+b[0] && pt_r[1].y >= m[1]*pt_r[1].x+b[1]) ||
       ((pt_l[0].y > m[0]*pt_l[0].x+b[0])&&(pt_l[1].y > m[0]*pt_l[1].x+b[0])&&
       (pt_r[0].y < m[1]*pt_r[0].x+b[1])&&(pt_r[1].y < m[1]*pt_r[1].x+b[1])))
        {
            
            double center_x=(min_pt.x+max_pt.x)/2.0;
            double center_y=(min_pt.y+max_pt.y)/2.0;
            double dist = sqrt(center_x*center_x+center_y*center_y);
            
            std::string obj_dist="";
            std::stringstream ss;
            ss << dist;
            obj_dist=ss.str();            
            marker_obj.points.push_back(max_pt);
            marker_obj.points.push_back(min_pt);
            marker_obj.header.frame_id = "/Sensor";
            marker_obj.header.stamp = ros::Time::now();
            marker_obj.ns = "object";
            marker_obj.id = i;
            marker_obj.type = visualization_msgs::Marker::POINTS;
            marker_obj.action = visualization_msgs::Marker::ADD;
            marker_obj.pose.position.x = 0;
            marker_obj.pose.position.y = 0;
            marker_obj.pose.position.z = 0;
            marker_obj.pose.orientation.x = 0.0;
            marker_obj.pose.orientation.y = 0.0;
            marker_obj.pose.orientation.z = 0.0;
            marker_obj.pose.orientation.w = 1.0;
            marker_obj.scale.x = 0.3f;
            marker_obj.scale.y = 0.3f;
            marker_obj.scale.z = 0.3f;
            marker_obj.color.r = 1.0f;
            marker_obj.color.g = 1.0f;
            marker_obj.color.b = 0.0f;
            marker_obj.color.a = 1.0;
            marker_obj.lifetime = ros::Duration(0.2);
            marker_obj.text = obj_dist;
            
            markers_obj.markers.push_back(marker_obj);
        }
    
  }
  filtered_obj.publish(markers_obj);
}
void lane_cb(const teb_local_planner::SKKU_LaneArray lanes)
{
    visualization_msgs::MarkerArray markers_lane;
    int num_lanes = lanes.lane.size();
    
    for(int i=0;i<3;i++)
    {
      lane_pts[0][i] = -lanes.lane[i].right;  
      lane_pts[1][i] = -lanes.lane[i].left;
    }
	
	// no detected lanes
	if(lane_pts[0][0]==1000 || lane_pts[0][0]>999)
		for(int i=0;i<3;i++)
		{
		  lane_pts[0][i] = 1;  
		  lane_pts[1][i] = -1;
		}
	
    ///////////////// Detect front object distance /////////////////////////////////
    visualization_msgs::Marker marker_lane_left,marker_lane_right,marker_obj;
    
    geometry_msgs::Point min_pt, max_pt;
    // calculate two line equations
    double m[2]={0,0};
    double b[2]={0,0};
    double y;
    m[0] = (lane_pts[0][0]-lane_pts[0][1])/(3.0-50.0);
    m[1] = (lane_pts[1][0]-lane_pts[1][1])/(3.0-50.0);
    b[0] = lane_pts[0][0]-m[0]*3.0;
    b[1] = lane_pts[1][0]-m[1]*3.0;


    // set markers
    marker_lane_left.header.frame_id = marker_lane_right.header.frame_id = "/Sensor";
    marker_lane_left.header.stamp = marker_lane_right.header.stamp = ros::Time::now();
    marker_lane_left.type = marker_lane_right.type = visualization_msgs::Marker::LINE_STRIP;
    marker_lane_left.action = marker_lane_right.action =visualization_msgs::Marker::ADD;
    marker_lane_left.scale.x = marker_lane_right.scale.x = 0.01f;
    marker_lane_left.scale.y = marker_lane_right.scale.y = 0.01f;
    marker_lane_left.scale.z = marker_lane_right.scale.z = 0.01f;
    marker_lane_left.color.r = marker_lane_right.color.r = 0.0f;
    marker_lane_left.color.g = marker_lane_right.color.g = 1.0f;
    marker_lane_left.color.b = marker_lane_right.color.b = 0.0f;
    marker_lane_left.color.a = marker_lane_right.color.a = 1.0;
    marker_lane_left.lifetime = marker_lane_right.lifetime = ros::Duration(0.2);

    for(int x=0;x<50;x++)
    {
        marker_lane_left.ns = "lane_left";
        marker_lane_right.ns = "lane_right";
        geometry_msgs::Point pt;
        pt.x=x;
        pt.y = m[0]*x+b[0];
        marker_lane_left.points.push_back(pt);
        pt.y = m[1]*x+b[1];
        marker_lane_right.points.push_back(pt);
    }    
    markers_lane.markers.push_back(marker_lane_left);
    markers_lane.markers.push_back(marker_lane_right);
    lane_pub.publish(markers_lane);
    
}
void cloud_cb (const sensor_msgs::PointCloud2 input)
{
    bool isObj=false;
    bool isFirst=true;
    ros::Time curr_time = ros::Time::now();
    double pt_size=0.0;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>); // creates a shared pointer
    pcl::fromROSMsg(input, *cloud);

    // extract front object
    visualization_msgs::MarkerArray markers_lane,markers_obj, markers_arrow;

    // grouping cloud by itensity
    int intensity=0;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
    pcl::PointCloud<pcl::PointXYZ> obj_cloud;
    for(int i=0;i<cloud->points.size();i++)
    {
        if(intensity<cloud->points[i].intensity)
        {
            if(intensity!=0)
                clouds.push_back(obj_cloud);
            intensity = cloud->points[i].intensity;
            obj_cloud.clear();
        }
        else{
        }
        pcl::PointXYZI pt = cloud->points[i];
        pcl::PointXYZ pt2;
        pt2.x = pt.x; pt2.y=pt.y; pt2.z = pt.z;
        obj_cloud.push_back(pt2);
    }
    clouds.push_back(obj_cloud);
    
    for(int i=0;i<clouds.size();i++)
    {
        if(clouds[i].size()>0)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>(clouds[i])); 
            // calculate min max pt        
            pcl::PointXYZ max_point,min_point;
            pcl::getMinMax3D(*cloud_ptr, min_point, max_point);
            geometry_msgs::Point min_pt, max_pt;
            max_pt.x = max_point.x; max_pt.y = max_point.y; max_pt.z = max_point.z;
            min_pt.x = min_point.x; min_pt.y = min_point.y; min_pt.z = min_point.z;
        

            ///////////////// Detect front object distance /////////////////////////////////
            visualization_msgs::Marker marker_lane_left,marker_lane_right,marker_obj;
        
            // calculate two line equations
            double m[2]={0,0};
            double b[2]={0,0};
            double y;
            m[0] = (lane_pts[0][0]-lane_pts[0][1])/(3.0-50.0);
            m[1] = (lane_pts[1][0]-lane_pts[1][1])/(3.0-50.0);
            b[0] = lane_pts[0][0]-m[0]*3.0;
            b[1] = lane_pts[1][0]-m[1]*3.0;
        
            // set markers
            marker_lane_left.header.frame_id = marker_lane_right.header.frame_id = "/Sensor";
            marker_lane_left.header.stamp = marker_lane_right.header.stamp = ros::Time::now();
            marker_lane_left.type = marker_lane_right.type = visualization_msgs::Marker::LINE_STRIP;
            marker_lane_left.action = marker_lane_right.action =visualization_msgs::Marker::ADD;
            marker_lane_left.scale.x = marker_lane_right.scale.x = 0.01f;
            marker_lane_left.scale.y = marker_lane_right.scale.y = 0.01f;
            marker_lane_left.scale.z = marker_lane_right.scale.z = 0.01f;
            marker_lane_left.color.r = marker_lane_right.color.r = 0.0f;
            marker_lane_left.color.g = marker_lane_right.color.g = 1.0f;
            marker_lane_left.color.b = marker_lane_right.color.b = 0.0f;
            marker_lane_left.color.a = marker_lane_right.color.a = 1.0;
            marker_lane_left.lifetime = marker_lane_right.lifetime = ros::Duration(0.2);
        
            for(int x=0;x<50;x++)
            {
                marker_lane_left.ns = "lane_left";
                marker_lane_right.ns = "lane_right";
                geometry_msgs::Point pt;
                pt.x=x;
                pt.y = m[0]*x+b[0];
                marker_lane_left.points.push_back(pt);
                pt.y = m[1]*x+b[1];
                marker_lane_right.points.push_back(pt);
            }    
            markers_lane.markers.push_back(marker_lane_left);
            markers_lane.markers.push_back(marker_lane_right);
        
            // extract the objects between the lanes
            geometry_msgs::Point pt_r[2],pt_l[2];
            pt_l[0] = max_pt;
            pt_l[1].x = min_pt.x; pt_l[1].y = max_pt.y;
            pt_r[0] = min_pt;
            pt_r[1].x = max_pt.x; pt_r[1].y = min_pt.y;
        
                if((pt_l[0].y <= m[0]*pt_l[0].x+b[0] && pt_l[0].y >= m[1]*pt_l[0].x+b[1]) ||
                (pt_l[1].y <= m[0]*pt_l[1].x+b[0] && pt_l[1].y >= m[1]*pt_l[1].x+b[1]) ||
                (pt_r[0].y <= m[0]*pt_r[0].x+b[0] && pt_r[0].y >= m[1]*pt_r[0].x+b[1]) ||
                (pt_r[1].y <= m[0]*pt_r[1].x+b[0] && pt_r[1].y >= m[1]*pt_r[1].x+b[1]) ||
                ((pt_l[0].y > m[0]*pt_l[0].x+b[0])&&(pt_l[1].y > m[0]*pt_l[1].x+b[0])&&
                (pt_r[0].y < m[1]*pt_r[0].x+b[1])&&(pt_r[1].y < m[1]*pt_r[1].x+b[1])))
                {
                    
                    double center_x=(min_pt.x+max_pt.x)/2.0;
                    double center_y=(min_pt.y+max_pt.y)/2.0;
                    double dist = sqrt(center_x*center_x+center_y*center_y);
                    double angle=0.0;

                    
                    ///////////////// Detect line & Calculate gradient  /////////////////////////////////
                    // RANSAC
                    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
                    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                    // Create the segmentation object
                    pcl::SACSegmentation<pcl::PointXYZ> seg;
                    // Optional
                    seg.setOptimizeCoefficients (true);
                    // Mandatory
                    seg.setModelType (pcl::SACMODEL_LINE);
                    seg.setMethodType (pcl::SAC_RANSAC);
                    seg.setDistanceThreshold (0.01);
                    seg.setInputCloud (cloud_ptr);
                    seg.segment (*inliers, *coefficients);

                    if (inliers->indices.size () == 0)
                    {
                    PCL_ERROR ("Could not estimate a LINE model for the given dataset.");
                    }
                    tf::Vector3 axis_vector(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
                    tf::Vector3 up_vector(1.0, 0.0, 0.0);
                    tf::Vector3 right_vector = axis_vector.cross(up_vector);//
                    right_vector.normalized();//
                    tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));//
                    q.normalize();//
                    geometry_msgs::Quaternion line_orientation;
                    tf::quaternionTFToMsg(q, line_orientation);
                
                    
                    //angle=2*asin(line_orientation.y)*180.0/PI;//formulae for quarternions.
                    tf::Matrix3x3 mat_(q);
                    double roll, pitch, yaw;
                    mat_.getRPY(roll, pitch, yaw);
                    angle=yaw*180.0/PI;

                    visualization_msgs::Marker marker_arrow;
                    marker_arrow.ns = "object_pose";
                    marker_arrow.header.frame_id = "Sensor";
                    marker_arrow.header.stamp = ros::Time::now();
                    marker_arrow.type = visualization_msgs::Marker::ARROW;
                    marker_arrow.action = visualization_msgs::Marker::ADD;
                    marker_arrow.pose.position.x = center_x; //coefficients->values[0];
                    marker_arrow.pose.position.y = center_y; //coefficients->values[1];
                    marker_arrow.pose.position.z = 0.0;                     //coefficients->values[2];
                    marker_arrow.pose.orientation.x = line_orientation.x;   //coefficients->values[3];
                    marker_arrow.pose.orientation.y = line_orientation.y;   //coefficients->values[4];
                    marker_arrow.pose.orientation.z = line_orientation.z;   //coefficients->values[5];
                    marker_arrow.pose.orientation.w = line_orientation.w;   //0.0;
                    marker_arrow.id = i;
                    marker_arrow.scale.x = 1.5;
                    marker_arrow.scale.y = 0.1;
                    marker_arrow.scale.z = 0.1;
                    // Set the color -- be sure to set alpha to something non-zero!
                    marker_arrow.color.r = 1.0f;
                    marker_arrow.color.g = 0.0f;
                    marker_arrow.color.b = 0.0f;
                    marker_arrow.color.a = 1.0f;
                    marker_arrow.lifetime = ros::Duration(0.2);
                    // Publish the marker
                    markers_arrow.markers.push_back(marker_arrow);
                    

                    std::string obj_dist="";
                    std::stringstream ss;
                    ss << angle;
                    obj_dist=ss.str();            
                    marker_obj.points.push_back(max_pt);
                    marker_obj.points.push_back(min_pt);
                    marker_obj.header.frame_id = "/Sensor";
                    marker_obj.header.stamp = ros::Time::now();
                    marker_obj.ns = "object";
                    marker_obj.id = i;
                    marker_obj.type = visualization_msgs::Marker::POINTS;
                    marker_obj.action = visualization_msgs::Marker::ADD;
                    marker_obj.pose.position.x = 0;
                    marker_obj.pose.position.y = 0;
                    marker_obj.pose.position.z = 0;
                    marker_obj.pose.orientation.x = 0.0;
                    marker_obj.pose.orientation.y = 0.0;
                    marker_obj.pose.orientation.z = 0.0;
                    marker_obj.pose.orientation.w = 1.0;
                    marker_obj.scale.x = 0.3f;
                    marker_obj.scale.y = 0.3f;
                    marker_obj.scale.z = 0.3f;
                    marker_obj.color.r = 1.0f;
                    marker_obj.color.g = 1.0f;
                    marker_obj.color.b = 0.0f;
                    marker_obj.color.a = 1.0;
                    marker_obj.lifetime = ros::Duration(0.2);
                    marker_obj.text = obj_dist;
                    
                    markers_obj.markers.push_back(marker_obj);

                    
                    /*
                    if(isFirst)
                    {
                        fs1 << curr_time;
                        fs1 <<", ";
                        fs2 << curr_time;
                        fs2 <<", ";
                        isFirst=false;
                    }
                    fs2 << max_pt.x;
                    fs2 <<", ";
                    fs2 << max_pt.y;
                    fs2 <<", ";
                    fs2 << min_pt.x;
                    fs2 <<", ";
                    fs2 << min_pt.y;
                    fs2 <<", ";
                    fs2 << angle;
                    fs2 <<", ";

                    
                    pt_size+=cloud_ptr->points.size();
                    isObj=true;
                    */

            }
            else
            {
                //ROS_INFO("%d cloud is empty",i);
            }
        }

    }
    /*
    if(isObj)
    {    
          fs1 << pt_size;
        fs1 <<"\n";
        fs2 <<"\n";
    }
    else{
    }
    */

  lane_pub.publish(markers_lane);
  filtered_obj.publish(markers_obj);
  arrow_pub.publish(markers_arrow);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obj_node");

  ros::NodeHandle nh;
  //fs1.open ("/home/jjm/raw.txt");
  //fs2.open ("/home/jjm/pt.txt");

  //ros::Subscriber marker_sub = nh.subscribe ("Sensor/visualization_objects", 1, marker_cb);
  ros::Subscriber sub = nh.subscribe ("Sensor/clustered_points", 1, cloud_cb);
  ros::Subscriber lane_sub = nh.subscribe ("lane_array", 1, lane_cb);

  filtered_obj = nh.advertise<visualization_msgs::MarkerArray>("Sensor/filtered_objects", 100);
  lane_pub = nh.advertise<visualization_msgs::MarkerArray>("lane_markers", 100);
  arrow_pub = nh.advertise<visualization_msgs::MarkerArray>("Sensor/front_object_pose", 100);

  ros::spin();

  //fs1.close();
  //fs2.close();

  return 0;
}

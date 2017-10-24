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

#include <cmath>
#include <math.h>

#define PI 3.14159265

ros::Publisher wall_pub;

std::vector<double> lineFitting(pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud)
{
    std::vector<double> coeff;
    double angle=0;

    if(!_cloud->points.empty())
    {
        // RANSAC
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_LINE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.1);
        seg.setInputCloud (_cloud);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0)
        {
            return coeff;
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

        coeff.push_back(coefficients->values[0]);
        coeff.push_back(coefficients->values[1]);
        coeff.push_back(coefficients->values[2]);
        coeff.push_back(coefficients->values[3]);
        coeff.push_back(coefficients->values[4]);
        coeff.push_back(coefficients->values[5]);
        coeff.push_back(angle);
        return coeff;
    }
    else
    {
        return coeff;
    }

}
void cloud_cb (const sensor_msgs::PointCloud2 input)
{
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>); // creates a shared pointer
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_left (new pcl::PointCloud<pcl::PointXYZI>); // creates a shared pointer
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_right (new pcl::PointCloud<pcl::PointXYZI>); // creates a shared pointer
    pcl::fromROSMsg(input, *cloud);

    visualization_msgs::MarkerArray markers_arrow;
    if(!cloud->points.empty())
    {
        // clear the arrow
        
        pcl::PassThrough<pcl::PointXYZI> pass;

        // passthrough filter to remove spurious NaNs
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y"); // z
        pass.setFilterLimits(0, 10);
        pass.filter(*cloud_left);

        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y"); // z
        pass.setFilterLimits(-10, 0);
        pass.filter(*cloud_right);

        ///////////////// Detect line & Calculate gradient  /////////////////////////////////
        std::vector<double> left_coeff,right_coeff;
        left_coeff = lineFitting(cloud_left);
        right_coeff = lineFitting(cloud_right);

    if(left_coeff.size()>0)
    {
        double dist=0.0, angle=0.0;
        double a,b,c;
        double x,y;
        a = left_coeff[4]/left_coeff[3];
        b = -1;
        c = left_coeff[1]-left_coeff[0]*left_coeff[4]/left_coeff[3];
        x = (b*(b*0-a*0)-a*c)/(a*a+b*b);
        y = (a*(-b*0+a*0)-b*c)/(a*a+b*b);
        dist = sqrt(x*x+y*y);
        angle = left_coeff[6];

        //
            visualization_msgs::Marker marker_arrow;
            marker_arrow.ns = "left_line";
            marker_arrow.header.frame_id = "Sensor";
            marker_arrow.header.stamp = ros::Time::now();
            marker_arrow.type = visualization_msgs::Marker::ARROW;
            marker_arrow.action = visualization_msgs::Marker::ADD;
            marker_arrow.scale.x = 0.2;
            marker_arrow.scale.y = 0.5;
            marker_arrow.scale.z = 0.2;
            // Set the color -- be sure to set alpha to something non-zero!
            marker_arrow.color.r = 0.16f;
            marker_arrow.color.g = 0.16f;
            marker_arrow.color.b = 0.16f;
            marker_arrow.color.a = 1.0f;
            marker_arrow.lifetime = ros::Duration();
            //

            //check horizon or vertical
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            geometry_msgs::Point pt1, pt2;
            if(abs(angle)>90-10 && abs(angle)<90+10)  //horizon
            {
                if(angle>0)
                    angle=angle-90;
                else
                    angle=angle+90;
                pcl::PointXYZ norm_vec;
                norm_vec.x=1.0*cos(angle*PI/180);
                norm_vec.y=1.0*sin(angle*PI/180);

                for(int i=0;i<cloud_left->points.size();i++)
                {
                    double interval=0.0;
                    interval = abs(a*cloud_left->points[i].x+b*cloud_left->points[i].y+c)/sqrt(a*a+b*b);
                    if(interval<0.05)
                    {
                        tmp_cloud->points.push_back(pcl::PointXYZ(cloud_left->points[i].x,cloud_left->points[i].y,cloud_left->points[i].z));
                        
                    }
                }
                pcl::PointXYZ min_pt,max_pt;
                pcl::getMinMax3D(*tmp_cloud,min_pt,max_pt);
                if(min_pt.x<15.0 && min_pt.y>1.0)
                {                
                    marker_arrow.color.g = 1.0f;
                    // re-calculate 
                    a = -norm_vec.y/norm_vec.x;
                    b = 1;
                    c = a*min_pt.x+b*min_pt.y;
                    x = (b*(b*0-a*0)-a*c)/(a*a+b*b);
                    y = -(a*(-b*0+a*0)-b*c)/(a*a+b*b);
                    
                    pt1.x = x; pt1.y = y; pt1.z = 0.0;
                    pt2.x = pt1.x+norm_vec.x; pt2.y = pt1.y+norm_vec.y; pt2.z = 0.0;

                    marker_arrow.points.push_back(pt1);
                    marker_arrow.points.push_back(pt2);
                    
                    std::string obj_incl="";
                    std::stringstream ss;
                    ss << angle;
                    obj_incl=ss.str();  
                    marker_arrow.text = obj_incl;
                    // Publish the marker
                    if(pt1.y>=0)
                        markers_arrow.markers.push_back(marker_arrow); 
                }
            }     
            else
            {        

                pt1.x = x; pt1.y = y; pt1.z = 0.0;
                pt2.x = pt1.x+left_coeff[3]; pt2.y = pt1.y+left_coeff[4]; pt2.z = pt1.z+left_coeff[5];
                marker_arrow.points.push_back(pt1);
                marker_arrow.points.push_back(pt2);

                std::string obj_incl="";
                std::stringstream ss;
                ss << left_coeff[6];
                obj_incl=ss.str();  
                marker_arrow.text = obj_incl;
                // Publish the marker
                if(pt1.y>=0)
                    markers_arrow.markers.push_back(marker_arrow);
            } 
    }
    if(right_coeff.size()>0)
    {
            double dist=0.0,angle=0.0;
            double a,b,c;
            double x,y;
            a = right_coeff[4]/right_coeff[3];
            b = -1;
            c = right_coeff[1]-right_coeff[0]*right_coeff[4]/right_coeff[3];
            x = (b*(b*0-a*0)-a*c)/(a*a+b*b);
            y = (a*(-b*0+a*0)-b*c)/(a*a+b*b);
            dist = sqrt(x*x+y*y);
            angle = right_coeff[6];

            //

            visualization_msgs::Marker marker_arrow;
            marker_arrow.ns = "right_line";
            marker_arrow.header.frame_id = "Sensor";
            marker_arrow.header.stamp = ros::Time::now();
            marker_arrow.type = visualization_msgs::Marker::ARROW;
            marker_arrow.action = visualization_msgs::Marker::ADD;
            marker_arrow.scale.x = 0.2;
            marker_arrow.scale.y = 0.5;
            marker_arrow.scale.z = 0.2;
            // Set the color -- be sure to set alpha to something non-zero!
            marker_arrow.color.r = 0.0f;
            marker_arrow.color.g = 0.0f;
            marker_arrow.color.b = 0.0f;
            marker_arrow.color.a = 1.0f;
            marker_arrow.lifetime = ros::Duration();

            //check horizon or vertical
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            geometry_msgs::Point pt1, pt2;
            
            if(abs(angle)>90-10 && abs(angle)<90+10)  //horizon
            {
                if(angle>0)
                    angle=angle-90;
                else
                    angle=angle+90;
                pcl::PointXYZ norm_vec;
                norm_vec.x=1.0*cos(angle*PI/180);
                norm_vec.y=1.0*sin(angle*PI/180);

                for(int i=0;i<cloud_right->points.size();i++)
                {
                    double interval=0.0;
                    interval = abs(a*cloud_right->points[i].x+b*cloud_right->points[i].y+c)/sqrt(a*a+b*b);
                    if(interval<0.05)
                    {
                        tmp_cloud->points.push_back(pcl::PointXYZ(cloud_right->points[i].x,cloud_right->points[i].y,cloud_right->points[i].z));
                        
                    }
                }
                pcl::PointXYZ min_pt,max_pt;
                pcl::getMinMax3D(*tmp_cloud,min_pt,max_pt);
                
                if(max_pt.x<15.0 && max_pt.y<-1.0)
                {  
                    marker_arrow.color.r = 1.0f;
                    // re-calculate 
                    a = -norm_vec.y/norm_vec.x;
                    b = 1;
                    c = a*max_pt.x+b*max_pt.y;
                    x = (b*(b*0-a*0)-a*c)/(a*a+b*b);
                    y = -(a*(-b*0+a*0)-b*c)/(a*a+b*b);
                    
                    pt1.x = x; pt1.y = y; pt1.z = 0.0;
                    pt2.x = pt1.x+norm_vec.x; pt2.y = pt1.y+norm_vec.y; pt2.z = 0.0;

                    marker_arrow.points.push_back(pt1);
                    marker_arrow.points.push_back(pt2);
                    
                    std::string obj_incl="";
                    std::stringstream ss;
                    ss << angle;
                    obj_incl=ss.str();  
                    marker_arrow.text = obj_incl;
                    // Publish the marker
                    if(pt1.y<=0)
                        markers_arrow.markers.push_back(marker_arrow);
                }
            }     
            else
            {
                
                pt1.x = x; pt1.y = y; pt1.z = 0.0;
                pt2.x = pt1.x+right_coeff[3]; pt2.y = pt1.y+right_coeff[4]; pt2.z = pt1.z+right_coeff[5];
                marker_arrow.points.push_back(pt1);
                marker_arrow.points.push_back(pt2);
        
                std::string obj_incl="";
                std::stringstream ss;
                ss << right_coeff[6];
                obj_incl=ss.str();  
                marker_arrow.text = obj_incl;
                // Publish the marker
                if(pt1.y<=0)
                        markers_arrow.markers.push_back(marker_arrow);
            }

            
    }
    }
  wall_pub.publish(markers_arrow);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "side_obj_node");

  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe ("Sensor/clustered_points", 1, cloud_cb);

  wall_pub = nh.advertise<visualization_msgs::MarkerArray>("Sensor/side_object_pose", 100);

  ros::spin();

  return 0;
}

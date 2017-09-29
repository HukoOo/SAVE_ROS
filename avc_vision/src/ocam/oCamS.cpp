#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <ocams/camConfig.h>
#include <boost/thread.hpp>

#include "withrobot_camera.hpp"

class Camera
{
    Withrobot::Camera* camera;
    Withrobot::camera_format camFormat;

private:
    int width_;
    int height_;
    std::string devPath_;

    int resolution_;
    double frame_rate_;
    int exposure_, gain_, wb_blue_, wb_red_;
    bool autoexposure_;

public:
    /**
     * @brief      { camera driver }
     *
     * @param[in]  resolution  The resolution
     * @param[in]  frame_rate  The frame rate
     */
    Camera(int resolution, double frame_rate, std::string port) 
    {
        enum_dev_list();

        if (resolution == 0) { width_ = 1280; height_ = 960;}
        if (resolution == 1) { width_ = 1280; height_ = 720;}
        if (resolution == 2) { width_ = 640; height_  = 480;}
        if (resolution == 3) { width_ = 640; height_  = 360;}
        
        devPath_=port;
        camera= new Withrobot::Camera(devPath_.c_str());
        camera->set_format(width_, height_, Withrobot::fourcc_to_pixformat('Y','U','Y','V'), 1, (unsigned int)frame_rate);

        camera->get_current_format(camFormat);
        camFormat.print();
        camera->start();
 
    }

    ~Camera() {

            camera->stop();
            delete camera;
    }

    void enum_dev_list()
    {
        /* enumerate device(UVC compatible devices) list */
        std::vector<Withrobot::usb_device_info> dev_list;
        int dev_num = Withrobot::get_usb_device_info_list(dev_list);

        if (dev_num < 1) {
            dev_list.clear();

            return;
        }

        for (unsigned int i=0; i < dev_list.size(); i++) 
        {
            std::cout <<i <<" "<< dev_list[i].product << " " << dev_list[i].dev_node << std::endl;
            if (dev_list[i].product == "oCam-1CGN-U")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
        }
    }

    void uvc_control( int exposure, int gain, int blue, int red, bool ae)
    {
        /* Exposure Setting */
        camera->set_control("Exposure (Absolute)", exposure);

        /* Gain Setting */
        camera->set_control("Gain", gain);

        /* White Balance Setting */
        camera->set_control("White Balance Blue Component", blue);
        camera->set_control("White Balance Red Component", red);

        /* Auto Exposure Setting */
        if (ae)
            camera->set_control("Exposure, Auto", 0x3);
        else
            camera->set_control("Exposure, Auto", 0x1);
    }

    /**
     * @brief      Gets the images.
     *
     * @param     image   The image
     *
     * @return     The images.
     */
    bool getImages(cv::Mat &image, uint32_t &time_stamp)
    {

        cv::Mat Img(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
        cv::Mat image_rgb;
        uint32_t ts;
       if (camera->get_frame(Img.data, camFormat.image_size, 1) != -1)
        {
        	cv::cvtColor(Img, image_rgb, CV_BayerGR2RGB);
           // time stamp
           memcpy(&ts, image_rgb.data, sizeof(ts));
            time_stamp = ts;
            image = image_rgb;
            return true;
        } else {
            return false;
        }        
    }
};

class oCamROS {

private:
    int resolution_;
    double frame_rate_;
    int exposure_, gain_, wb_blue_, wb_red_;
    bool autoexposure_;
    bool show_image_;
    bool config_changed_;
    ros::NodeHandle nh;
    std::string frame_id_,image_topic_,port_,info_topic_,info_xml_, camera_name_,time_topic_;
    Camera *cam_left,*cam_right;
    /**
     * @brief      { publish camera info }
     *
     * @param[in]  pub_cam_info  The pub camera information
     * @param[in]  cam_info_msg  The camera information message
     * @param[in]  now           The now
     */
    void publishCamInfo(const ros::Publisher& pub_cam_info, sensor_msgs::CameraInfo& cam_info_msg, ros::Time now) {
        cam_info_msg.header.stamp = now;
        pub_cam_info.publish(cam_info_msg);
    }

    /**
     * @brief      { publish image }
     *
     * @param[in]  img           The image
     * @param      img_pub       The image pub
     * @param[in]  img_frame_id  The image frame identifier
     * @param[in]  t             { parameter_description }
     */
    void publishImage(cv::Mat img, image_transport::Publisher &img_pub, std::string img_frame_id, ros::Time t) {
        cv_bridge::CvImage cv_image;
        cv_image.image = img;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.header.frame_id = img_frame_id;
        cv_image.header.stamp = t;
        img_pub.publish(cv_image.toImageMsg());
    }

    void device_poll() 
    {
        //Reconfigure confidence
        dynamic_reconfigure::Server<ocams::camConfig> server;
        dynamic_reconfigure::Server<ocams::camConfig>::CallbackType f;
        f = boost::bind(&oCamROS::callback, this ,_1, _2);
        server.setCallback(f);

        // setup publisher stuff
        image_transport::ImageTransport it(nh);
        image_transport::Publisher left_image_pub = it.advertise("stereo/left/image_raw", 1);
        image_transport::Publisher right_image_pub = it.advertise("stereo/right/image_raw", 1);

        ros::Publisher cam_time_stamp_pub = nh.advertise<sensor_msgs::TimeReference>(time_topic_,1);
        ros::Publisher left_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("stereo/left/camera_info", 1);
        ros::Publisher right_cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("stereo/right/camera_info", 1);

        sensor_msgs::TimeReference time_stamp_msg;
        sensor_msgs::CameraInfo left_info, right_info;

        ROS_INFO("Loading from ROS calibration files");

        // get config from the left, right.yaml in config
        camera_info_manager::CameraInfoManager info_manager(nh);

        info_manager.setCameraName("left");
        info_manager.loadCameraInfo( "package://avc_vision/config/640x480/left.yaml");
        left_info = info_manager.getCameraInfo();

        info_manager.setCameraName("right");
        info_manager.loadCameraInfo( "package://avc_vision/config/640x480/right.yaml");
        right_info = info_manager.getCameraInfo();

        left_info.header.frame_id = "left_frame";
        right_info.header.frame_id = "right_frame";

        ROS_INFO("Got camera calibration files");

        // loop to publish images;
        cv::Mat left_image, right_image;
        uint32_t time_stamp, sec, nsec;
        ros::Rate r(frame_rate_);

        while (ros::ok())
        {
            ros::Time now = ros::Time::now();

            if (!cam_left->getImages(left_image, time_stamp))
            {
                usleep(1000);
                continue;
            } else {
                ROS_INFO_ONCE("Success, found camera");
            }
            if (!cam_right->getImages(right_image, time_stamp))
            {
                usleep(1000);
                continue;
            } else {
                ROS_INFO_ONCE("Success, found camera");
            }

            /* time stamp publish */
            sec = (uint32_t)time_stamp/1000;
            nsec = (uint32_t)(time_stamp - sec*1000) * 1e6;
            ros::Time measurement_time(sec, nsec);
            ros::Time time_ref(0, 0);
            time_stamp_msg.header.stamp = measurement_time;
            time_stamp_msg.header.frame_id = "left_frame";
            time_stamp_msg.time_ref = time_ref;
            cam_time_stamp_pub.publish(time_stamp_msg);


            if (left_image_pub.getNumSubscribers() > 0) {
                publishImage(left_image, left_image_pub, "left_frame", now);
            }
            if (right_image_pub.getNumSubscribers() > 0) {
                publishImage(right_image, right_image_pub, "right_frame", now);
            }
            if (left_cam_info_pub.getNumSubscribers() > 0) {
                publishCamInfo(left_cam_info_pub, left_info, now);
            }
            if (right_cam_info_pub.getNumSubscribers() > 0) {
                publishCamInfo(right_cam_info_pub, right_info, now);
            }

            if (show_image_) 
            {
                cv::imshow("left", left_image);
                cv::imshow("right", right_image);
                cv::waitKey(1);
            }

            r.sleep();
        }
    }

    void callback(ocams::camConfig &config, uint32_t level) 
    {
//        ROS_INFO("exposure:%d, gain:%d, blue:%d, red:%d, ae:%d", config.exposure, config.gain, config.wb_blue, config.wb_red, config.auto_exposure);
        cam_left->uvc_control(config.exposure, config.gain, config.wb_blue, config.wb_red, config.auto_exposure);
        cam_right->uvc_control(config.exposure, config.gain, config.wb_blue, config.wb_red, config.auto_exposure);
    }


public:
    /**
     * @brief      { function_description }
     *
     * @param[in]  resolution  The resolution
     * @param[in]  frame_rate  The frame rate
     */
    oCamROS() 
    {
        ros::NodeHandle priv_nh("~");

        /* default parameters */
        resolution_ = 2;
        frame_rate_ = 60.0;
        exposure_ = 100;
        gain_ = 150;
        wb_blue_ = 100;
        wb_red_ = 100;
        autoexposure_= false;
        frame_id_ = "camera";
        image_topic_ = "ocam/image_raw";
        info_topic_ = "ocam/cam_info";
        info_xml_="package://ocams/config/left.yaml";
        camera_name_="ocam";
        show_image_ = false;
        port_="/dev/video0";

        /* get parameters 
        priv_nh.getParam("resolution", resolution_);
        priv_nh.getParam("frame_rate", frame_rate_);
        priv_nh.getParam("exposure", exposure_);
        priv_nh.getParam("gain", gain_);
        priv_nh.getParam("wb_blue", wb_blue_);
        priv_nh.getParam("wb_red", wb_red_);
        priv_nh.getParam("frame_id", frame_id_);
        priv_nh.getParam("show_image", show_image_);
        priv_nh.getParam("auto_exposure", autoexposure_);
        priv_nh.getParam("port", port_);
        priv_nh.getParam("image_topic", image_topic_);
        priv_nh.getParam("info_topic", info_topic_);
        priv_nh.getParam("info_xml", info_xml_);
        priv_nh.getParam("camera_name", camera_name_);
        */

        /* initialize the camera */
        cam_left = new Camera(resolution_, frame_rate_, "/dev/video1");
        cam_right = new Camera(resolution_, frame_rate_, "/dev/video2");
        cam_left->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        cam_right->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        ROS_INFO("Initialized the camera");

        // thread
        boost::shared_ptr<boost::thread> device_poll_thread;
        device_poll_thread = boost::shared_ptr<boost::thread>(new boost::thread(&oCamROS::device_poll, this));
    }

    ~oCamROS() 
    {
        delete cam_left;
        delete cam_right;
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "ocam");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    oCamROS ocams_ros;

    ros::spin();
}

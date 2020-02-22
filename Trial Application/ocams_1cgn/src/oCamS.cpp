//version 3.7.0
//0.0.0 trial code on linux to test opencv.
//1.0.0 original lib adaptation. had lots of errors.
//1.1.0 fixed opencv support error. Error lied in Eigen3. Now fixed.
//1.2.0 fixed camera data error. Can receive data from camera, although it doesnt look like a proper picture.
//1.2.1 fixed blended image error.
//1.2.2 successfully displayed 2 images: left and right.
//2.0.0 calibrated. baseline 0.12m fyi.
//2.1.0 found disparity map, but had trouble displaying.
//2.2.0 found that the process sometimes dies suddenly.
//2.2.1 tried to fix error to no avail. emailed Withrobot.Inc for solution, waited... ... ... ... ... ...
//3.0.0 got updated library. process no longer suddenly stops. however, has "core dump" error.
//3.1.0 fixed core dump error. it was due to image format incompatibility. Takeaway: always use CV::Mat instead of custom data types!
//3.2.0 trimmed the code from 2000+ lines down to around 1000 lines by removing all obsolete functions and commented code blocks completely.
//3.3.0 added onboard sensor integration
//3.4.0 fused unnecessary nodes and reduced data publication amount. Downgraded some functions to support bad microprocessor.
//3.5.0 shrinked the code to around 700 lines. introduced a new core dumped error if I use ctrl+C to terminate the program.
//3.6.0 added translation matrix to account for camera image shift. Greately improved accuracy of Stereovision
//3.7.0 added post-processing for a much better disparity map
//3.8.0 added a complete depth map function
// -- By Li Bozhao
// Not Yet Optimized. Estimated final version be 4.x.x.
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>
#include <ocams_1cgn/camConfig.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <boost/thread.hpp>
#include <image_geometry/stereo_camera_model.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include "withrobot_camera.hpp"
#include "myahrs_plus.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using namespace cv::ximgproc;
using namespace std;
using namespace WithrobotIMU;
//global Mat for debugging
cv::Mat visualizeDepth;
//Camera wrapper for setting up the camera. Utilises Withrobot Api.
class StereoCamera
{
    Withrobot::Camera* camera;
    Withrobot::camera_format camFormat;
private:
    std::string devPath_;
public:
    int width_;
    int height_;
    //constructor. 
    StereoCamera(int resolution, double frame_rate): camera(NULL)
    {
        enum_dev_list();
        camera = new Withrobot::Camera(devPath_.c_str());
        if (resolution == 0) { width_ = 1280; height_ = 960;}
        if (resolution == 1) { width_ = 1280; height_ = 720;}
        if (resolution == 2) { width_ = 640; height_  = 480;}   //this is the chosen resolution by default thanks to shitty microprocessor
        if (resolution == 3) { width_ = 640; height_  = 360;}
        if (resolution == 4) { width_ = 320; height_  = 240;}
        camera->set_format(width_, height_, Withrobot::fourcc_to_pixformat('Y','U','Y','V'), 1, (unsigned int)frame_rate);
        //get camera format (image size and frame rate)
        camera->get_current_format(camFormat);
        camFormat.print();
        // Withrobot camera start
        camera->start();
    }
    ~StereoCamera()
    {
        camera->stop();
        delete camera;
    }
    void enum_dev_list()
    {
        /* enumerate device(UVC compatible devices) list */
        std::vector<Withrobot::usb_device_info> dev_list;
        int dev_num = Withrobot::get_usb_device_info_list(dev_list);
        if (dev_num < 1) //usb device not found, stop immediately
        {
            dev_list.clear();
            return;
        }
        for (unsigned int i=0; i < dev_list.size(); i++)
        {
            if (dev_list[i].product == "oCamS-1CGN-U")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
            else if (dev_list[i].product == "oCamS-1CGN-U-F")
            {
                devPath_ = dev_list[i].dev_node;
                return;
            }
        }
    }
    //implement camera settings
    void uvc_control(int exposure, int gain, int blue, int red, bool ae)
    {
        // Exposure Setting 
        camera->set_control("Exposure (Absolute)", exposure);
        // Gain Setting 
        camera->set_control("Gain", gain);
        // White Balance Setting 
        camera->set_control("White Balance Blue Component", blue);
        camera->set_control("White Balance Red Component", red);
        // Auto Exposure Setting 
        if (ae)
            camera->set_control("Exposure, Auto", 0x3);
        else
            camera->set_control("Exposure, Auto", 0x1);
    }
    //get image from sensor. ALWAYS USE CV::Mat INSTEAD OF CUSTOM DATA TYPES!
    bool getImages(cv::Mat &left_image, cv::Mat &right_image, uint32_t &time_stamp) {
        cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC2);
        cv::Mat dstImg[2];
        uint32_t ts;
        if (camera->get_frame(srcImg.data, camFormat.image_size, 1) != -1)
        {
            // get time stamp
            memcpy(&ts, srcImg.data, sizeof(ts));
            //split source to left and right. each colored with 3 channels
            cv::split(srcImg, dstImg);
            time_stamp = ts;
            right_image = dstImg[0];
            left_image = dstImg[1];
            return true;
        } else {
            return false;
        }
    }
};
//onboard sensors wrapper. Utilizing myahrs_plus.cpp from Withrobot.
class MyAhrsDriverForROS : public iMyAhrsPlus
{
private:
    ros::NodeHandle nh;
    tf::TransformBroadcaster broadcaster_;
    Platform::Mutex lock_;
    //SensorData sensor_data_;
    std::string parent_frame_id_;
    std::string frame_id_;
    tf::TransformBroadcaster br;
    ros::Publisher pubMotiondata = nh.advertise<sensor_msgs::Imu>("motion/accelww", 1);
    ros::Publisher pubMotionTimestamp = nh.advertise<sensor_msgs::TimeReference>("motion/timestamp",1);
    ros::Publisher pubMotionMagnetic = nh.advertise<sensor_msgs::MagneticField>("motion/magnetic", 1);
    double linear_acceleration_stddev_;
    double angular_velocity_stddev_;
    double magnetic_field_stddev_;
    double orientation_stddev_;
    //receives data from onboard sensors when triggered
    void OnSensorData(int sensor_id, SensorData data)
    {
        //threading lock protection for reading the data.
        LockGuard _l(lock_);
        sensor_data_ = data;
        publicize_data();
        //publish for other nodes to perform SLAM or odometry
        pubMotiondata.publish(imu_data_msg);
        pubMotionTimestamp.publish(time_stamp_msg);
        pubMotionMagnetic.publish(imu_magnetic_msg);
        br.sendTransform(stamped_transform);
    }
    //visualizing a change of attribute
    void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value)
    {
        printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
    }
public:
    SensorData sensor_data_;                        //do not publish it, it is for processing
    sensor_msgs::Imu imu_data_msg;                  //publish it
    sensor_msgs::TimeReference time_stamp_msg;      //publish it
    sensor_msgs::MagneticField imu_magnetic_msg;    //publish it
    tf::StampedTransform stamped_transform;         //publish it
    //constructor
    MyAhrsDriverForROS(std::string port="", int baud_rate=115200): iMyAhrsPlus(port, baud_rate){  ros::Rate loop_rate(10); }
    //destructor
    ~MyAhrsDriverForROS(){}
    //initialize sensor wrapper
    bool initialize(std::string mode="")
    {
        bool ok = false;
        // DO NOT REMOVE WHILE(0) ALTHOUGH IT SEEMS USELESS! IT IS FOR ERROR HANDLING
        do
        {
            if(start() == false) break;
            //set and print IMU mode
            if(cmd_data_format(mode.c_str()) == false) break;
            printf("IMU initialized: %s\r\n", mode.c_str());
            ok = true;
        } while(0);
        return ok;
    }
    //gets data from sensor by ref
    inline void get_data(SensorData& data)
    {
        LockGuard _l(lock_);
        data = sensor_data_;
    }
    //gets data from sensor by val
    inline SensorData get_data()
    {
        LockGuard _l(lock_);
        return sensor_data_;
    }
    void publicize_data()
    {
        uint32_t time_stamp, sec, nsec;      
        //following constants provided by the manufacturer:
        double linear_acceleration_cov = 0.05;
        double angular_velocity_cov    = 0.025;
        double magnetic_field_cov      = -1;
        double orientation_cov         = 0.1;
        double orientation_denom       = 16384.;
        double acceleration_denom      = 100.;
        double angular_velocity_denom  = 900.;
        double magnetic_field_denom    = 16.;
        //conversion of data:
        imu_data_msg.linear_acceleration_covariance[0] =
                imu_data_msg.linear_acceleration_covariance[4] =
                imu_data_msg.linear_acceleration_covariance[8] = linear_acceleration_cov;
        imu_data_msg.angular_velocity_covariance[0] =
                imu_data_msg.angular_velocity_covariance[4] =
                imu_data_msg.angular_velocity_covariance[8] = angular_velocity_cov;
        imu_data_msg.orientation_covariance[0] =
                imu_data_msg.orientation_covariance[4] =
                imu_data_msg.orientation_covariance[8] = orientation_cov;
        imu_magnetic_msg.magnetic_field_covariance[0] =
                imu_magnetic_msg.magnetic_field_covariance[4] =
                imu_magnetic_msg.magnetic_field_covariance[8] = magnetic_field_cov;
        ros::Time now = ros::Time::now();
        // time stamp 
        time_stamp = sensor_data_.time_stamp;
        sec = (uint32_t)time_stamp/1000;
        nsec = (uint32_t)(time_stamp - sec*1000) * 1e6;
        ros::Time measurement_time(sec, nsec);
        ros::Time time_ref(0, 0);
        time_stamp_msg.header.stamp = measurement_time;
        time_stamp_msg.header.frame_id = frame_id_;
        time_stamp_msg.time_ref = time_ref;
        //time_stamp_pub_.publish(time_stamp_msg);
        now = measurement_time;
        imu_data_msg.header.stamp     =
                imu_magnetic_msg.header.stamp = now;
        imu_data_msg.header.frame_id = frame_id_;
        //conversion constants provided by Withrobot.
        // orientation
        imu_data_msg.orientation.x = float(sensor_data_.quaternion.x) / orientation_denom;
        imu_data_msg.orientation.y = float(sensor_data_.quaternion.y) / orientation_denom;
        imu_data_msg.orientation.z = float(sensor_data_.quaternion.z) / orientation_denom;
        imu_data_msg.orientation.w = float(sensor_data_.quaternion.w) / orientation_denom;
        // original data used the g unit, convert to m/s^2
        imu_data_msg.linear_acceleration.x     = float(sensor_data_.imu.ax) / acceleration_denom;
        imu_data_msg.linear_acceleration.y     = float(sensor_data_.imu.ay) / acceleration_denom;
        imu_data_msg.linear_acceleration.z     = float(sensor_data_.imu.az) / acceleration_denom;
        // original data used the degree/s unit, convert to radian/s
        imu_data_msg.angular_velocity.x     = float(sensor_data_.imu.gx) / angular_velocity_denom;
        imu_data_msg.angular_velocity.y     = float(sensor_data_.imu.gy) / angular_velocity_denom;
        imu_data_msg.angular_velocity.z     = float(sensor_data_.imu.gz) / angular_velocity_denom;
        // original data used the uTesla unit, convert to Tesla
        imu_magnetic_msg.magnetic_field.x = float(sensor_data_.imu.mx) / magnetic_field_denom;
        imu_magnetic_msg.magnetic_field.y = float(sensor_data_.imu.my) / magnetic_field_denom;
        imu_magnetic_msg.magnetic_field.z = float(sensor_data_.imu.mz) / magnetic_field_denom;
        //stamped_transform
        stamped_transform = tf::StampedTransform(tf::Transform(tf::Quaternion(imu_data_msg.orientation.x, imu_data_msg.orientation.y, 
            imu_data_msg.orientation.z, imu_data_msg.orientation.w),tf::Vector3(0.0, 0.0, 0.0)), now, parent_frame_id_, frame_id_);
    } 
};

//main class.
class oCamStereoROS {
    enum MatchingAlg{ STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_3WAY=3, STEREO_HH4=4 };
public:
    sensor_msgs::TimeReference time_stamp_msg;
    //constructor
    oCamStereoROS() 
    {   //create a node
        ros::NodeHandle priv_nh("~");
        // default parameters
        //TO DO: chanhge these according to the SHITTY hornet microprocessor
        resolution_ = 2;        //640*480
        frame_rate_ = 10.0;     //10.0 fps
        exposure_ = 100;
        gain_ = 150;
        wb_blue_ = 200; //important
        wb_red_ = 160; //important
        autoexposure_= true; //important, got hardware accel
        left_frame_id_ = "left_camera";
        right_frame_id_ = "right_camera";
		//modify this to true for debugging only:
        show_image_ = true;
        /* get parameters */
        priv_nh.getParam("resolution", resolution_);
        priv_nh.getParam("frame_rate", frame_rate_);
        priv_nh.getParam("exposure", exposure_);
        priv_nh.getParam("gain", gain_);
        priv_nh.getParam("wb_blue", wb_blue_);
        priv_nh.getParam("wb_red", wb_red_);
        priv_nh.getParam("left_frame_id", left_frame_id_);
        priv_nh.getParam("right_frame_id", right_frame_id_);
        priv_nh.getParam("show_image", show_image_);
        priv_nh.getParam("auto_exposure", autoexposure_);
        /* initialize the camera */
        ocams = new StereoCamera(resolution_, frame_rate_);
        ocams->uvc_control(exposure_, gain_, wb_blue_, wb_red_, autoexposure_);
        ROS_INFO("Initialized the camera");
        // thread for polling
        boost::shared_ptr<boost::thread> device_poll_thread;
        device_poll_thread = boost::shared_ptr<boost::thread>(new boost::thread(&oCamStereoROS::device_poll, this));
    }
    //destructor
    ~oCamStereoROS() {
        delete ocams;
        delete IMU;
    }
private:
    struct StereoParams
    {
        MatchingAlg mode;
        int preFilterType;
        int preFilterSize;
        int preFilterCap;
        int SADWindowSize;
        int minDisparity;
        int numDisparities;
        int textureThreshold;
        int uniquenessRatio;
        int speckleRange;
        int speckleWindowSize;
        int disp12MaxDiff;
        int p1;
        int p2;
    };
    StereoParams state;
    Withrobot::Camera* camera_ros;
    int resolution_;
    double frame_rate_;
    int exposure_, gain_, wb_blue_, wb_red_;
    bool autoexposure_;
    bool show_image_;
    bool config_changed_;

    cv::Mat dispbgr = cv::Mat::zeros(320, 240, CV_8UC1);
    ros::NodeHandle nh;
    std::string left_frame_id_, right_frame_id_;
    StereoCamera* ocams;
    MyAhrsDriverForROS* IMU;
    inline bool isValidPoint(const cv::Vec3f& pt)
    {
        // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
        // and zero disparities (point mapped to infinity).
        return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
    }
    //rectifies images
    void Rectification(cv::Mat& src_l, cv::Mat& src_r, cv::Mat& _map11, cv::Mat& _map12, cv::Mat& _map21, cv::Mat& _map22, cv::Mat& dst_l, cv::Mat& dst_r)
    {
        cv::Mat left = src_l;
        cv::Mat right = src_r;
        cv::Mat map11=_map11;
        cv::Mat map12=_map12;
        cv::Mat map21=_map21;
        cv::Mat map22=_map22;
        cv::Mat img1r, img2r;
        cv::remap(left, img1r, map11, map12, cv::INTER_LINEAR);
        cv::remap(right, img2r, map21, map22, cv::INTER_LINEAR);
        dst_l = img1r;
        dst_r = img2r;
    }
    //add processing by cv here.
    //find stereomatch
    void stereoMatch(cv::Mat left_mono, cv::Mat right_mono, cv::Mat& disparity, MatchingAlg alg = STEREO_BM)
    {
        cv::Mat left = left_mono;
        cv::Mat right = right_mono;
        cv::Mat dispL,dispR;
        Ptr<DisparityWLSFilter> wls_filter;
        if( alg == STEREO_BM )
        {
            cv::Ptr<cv::StereoBM> bmlr = cv::StereoBM::create();
            wls_filter = createDisparityWLSFilter(bmlr);
            //for fastest execution, disable all of these settings. result is fairly decent
            /*
            // pre-filter
            bmlr->setPreFilterType(state.preFilterType);
            bmlr->setPreFilterSize(state.preFilterSize);
            bmlr->setPreFilterCap(state.preFilterCap);
            bmlr->setTextureThreshold(state.textureThreshold);
            bmlr->setUniquenessRatio(state.uniquenessRatio);
            bmlr->setMinDisparity(state.minDisparity);
            //bmlr->setNumDisparities(state.numDisparities); //this line causes corrupted pixels
            //bmlr->setBlockSize(state.SADWindowSize); //this line causes the shifted image ERROR. OPENCV BUG!
            bmlr->setSpeckleWindowSize(state.speckleWindowSize);
            bmlr->setSpeckleRange(state.speckleRange);
            bmlr->setDisp12MaxDiff(state.disp12MaxDiff);
            */
            cv::Ptr<cv::StereoMatcher>bmrl = createRightMatcher(bmlr);
            // setting up effective area after pre-treatment of rod-like object detection (to do)
            //            bm->setROI1(roi1);
            //            bm->setROI2(roi2);
            bmlr->compute(left, right, dispL);
            bmrl->compute(right, left, dispR);
        }
        else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY || alg == STEREO_HH4 )
        {
            cv::Ptr<cv::StereoSGBM> sgbmlr = cv::StereoSGBM::create();
            wls_filter = createDisparityWLSFilter(sgbmlr);
            sgbmlr->setMinDisparity(state.minDisparity);
            sgbmlr->setNumDisparities(state.numDisparities);
            sgbmlr->setBlockSize(state.SADWindowSize);
            sgbmlr->setP1(state.p1);
            sgbmlr->setP2(state.p2);
            sgbmlr->setDisp12MaxDiff(state.disp12MaxDiff);
            sgbmlr->setPreFilterCap(state.preFilterCap);
            sgbmlr->setUniquenessRatio(state.uniquenessRatio);
            sgbmlr->setSpeckleWindowSize(state.speckleWindowSize);
            sgbmlr->setSpeckleRange(state.speckleRange);
            if(alg==STEREO_HH)
                sgbmlr->setMode(cv::StereoSGBM::MODE_HH);
            else if(alg==STEREO_SGBM)
                sgbmlr->setMode(cv::StereoSGBM::MODE_SGBM);
            else if(alg==STEREO_3WAY)
                sgbmlr->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
            else if(alg==STEREO_HH4)
                sgbmlr->setMode(cv::StereoSGBM::MODE_HH4);
            cv::Ptr<StereoMatcher> sgbmrl = createRightMatcher(sgbmlr);
            sgbmlr->compute(left, right, dispL);
            sgbmrl->compute(right, left, dispR);
        }
        //wls post-filtering
        double wls_lambda=16000.0;//8000:large->sticks to edges
        double wls_sigma=0.4; //0.8~2.0:large->leak edges;small->texture-sensitive
        wls_filter->setLambda(wls_lambda);
        wls_filter->setSigmaColor(wls_sigma);
        wls_filter->filter(dispL,left,disparity,dispR);
        //disparity=dispL;
        //wls_filter->filter(dispL,left,disparity,dispR,wls_filter->getROI(),right);
        //result is in CV_16S. get real disparity using disp.convertTo(disp, CV_32F, 1.0/16); from CSDN.
    }
    //wait for and processes data
    void device_poll() {
        //Reconfigure confidence
        dynamic_reconfigure::Server<ocams_1cgn::camConfig> server;
        dynamic_reconfigure::Server<ocams_1cgn::camConfig>::CallbackType f;
        f = boost::bind(&oCamStereoROS::callback, this ,_1, _2);
        server.setCallback(f);
        // setup publisher stuff for calibration
        image_transport::ImageTransport it(nh);
        image_transport::Publisher left_image_pub = it.advertise("stereo/left/image_raw", 1);
        image_transport::Publisher right_image_pub = it.advertise("stereo/right/image_raw", 1);

        sensor_msgs::CameraInfo left_info, right_info;
        ROS_INFO("Loading from ROS calibration files");
        // get config from the left, right.yaml in config
        camera_info_manager::CameraInfoManager info_manager(nh);
        info_manager.setCameraName("left");
        info_manager.loadCameraInfo( "package://ocams_1cgn/config/left.yaml");
        left_info = info_manager.getCameraInfo();
        info_manager.setCameraName("right");
        info_manager.loadCameraInfo( "package://ocams_1cgn/config/right.yaml");
        right_info = info_manager.getCameraInfo();
        left_info.header.frame_id = left_frame_id_;
        right_info.header.frame_id = right_frame_id_;
        //processing calibration
        float scale = 1.f;
        cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
        cv::Mat map11, map12, map21, map22;
        cv::Size img_size = cv::Size(ocams->width_, ocams->height_);
        // reading intrinsic parameters
        cv::Mat M1, D1, M2, D2;
        M1 = cv::Mat(3, 3, CV_64F, &left_info.K);
        M2 = cv::Mat(3, 3, CV_64F, &right_info.K);
        D1 = cv::Mat(1, 5, CV_64F, &left_info.D.at(0));
        D2 = cv::Mat(1, 5, CV_64F, &right_info.D.at(0));
        M1 *= scale;
        M2 *= scale;
        cv::Mat R, T, R1, P1, R2, P2;
        R1 = cv::Mat(3, 3, CV_64F, &left_info.R);
        P1 = cv::Mat(3, 4, CV_64F, &left_info.P);
        R2 = cv::Mat(3, 3, CV_64F, &right_info.R);
        P2 = cv::Mat(3, 4, CV_64F, &right_info.P);
        cv::initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_32FC1, map11, map12);
        cv::initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_32FC1, map21, map22);
        double Tx = 0.12;   //baseline
        double focal = P2.at<double>(0,0);
        Q.at<double>(0,0) = Q.at<double>(1,1) = 1.0;
        Q.at<double>(0,3) = -P2.at<double>(0,2);
        Q.at<double>(1,3) = -P2.at<double>(1,2);
        Q.at<double>(2,3) = P2.at<double>(0,0);
        Q.at<double>(3,2) = 1.0 / Tx;
        //std::cout << Q << std::endl;
        ROS_INFO("Got camera calibration files");
        /******************************************************************************************************************/
        //TO DO: more pre processing (edges or rod detection and recontruct rectangles)
        cv::Mat left_raw, right_raw,left_rgb, right_rgb;
        cv::Mat left_monoR,right_monoR;     
        cv::Mat left_monoG,right_monoG;     //use G channels with R channel for red/green/orange poles.
        cv::Mat left_rect_monoR, right_rect_monoR;
        cv::Mat left_rect_monoG, right_rect_monoG;
        cv::Mat map11_=map11.clone();
        cv::Mat map12_=map12.clone();
        cv::Mat map21_=map21.clone();
        cv::Mat map22_=map22.clone();
        cv::Mat disp16R, disp16G;
        uint32_t time_stamp, sec, nsec;
        cv_bridge::CvImage cv_image;
        ros::Publisher pubDepth = nh.advertise<sensor_msgs::Image>("stereo/depth", 1);
        ros::Publisher pubTimestamp = nh.advertise<sensor_msgs::TimeReference>("stereo/depth_time",1);
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::Time now = ros::Time::now();
            if (!ocams->getImages(left_raw, right_raw, time_stamp)) {
                usleep(10);
                continue;
            } else {
                ROS_INFO_ONCE("Success, found camera");
            }
            //Rectification
            cv::cvtColor(left_raw, left_rgb, COLOR_BayerGR2RGB);
            //cv::cvtColor(right_raw, right_mono, COLOR_BayerGR2GRAY);
			cv::cvtColor(right_raw, right_rgb, COLOR_BayerGR2RGB);
            cv::Mat channelsL[3];
            cv::Mat channelsR[3];
            cv::split(left_rgb,channelsL);
            left_monoR = channelsL[2];      //red channel is at 2 (BGR).
            left_monoG = channelsL[1];
            cv::split(right_rgb,channelsR);
            right_monoR = channelsR[2];
            right_monoG = channelsR[1];
            Rectification(left_monoR, right_monoR, map11, map12, map21, map22, left_rect_monoR, right_rect_monoR);
            Rectification(left_monoG, right_monoG, map11_, map12_, map21_, map22_, left_rect_monoG, right_rect_monoG);
            // enum mode {STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_3WAY=3, STEREO_HH4=4}
            //account for translation:
            float verticaltranslationdata[10] = { 1., 0., 0., 0., 1., -13.45729308 };  //I got this data by trial and error :( thank god it works
            cv::Mat verticalWarpMatrix = cv::Mat(2, 3, CV_32F, verticaltranslationdata);
            warpAffine(right_rect_monoR, right_rect_monoR, verticalWarpMatrix, right_rect_monoR.size(), INTER_LINEAR + WARP_INVERSE_MAP);
            warpAffine(right_rect_monoG, right_rect_monoG, verticalWarpMatrix, right_rect_monoG.size(), INTER_LINEAR + WARP_INVERSE_MAP);
            //stereomatch
            normalize(left_rect_monoR,left_rect_monoR,255.0,0.0,NORM_MINMAX);
            normalize(left_rect_monoG,left_rect_monoG,255.0,0.0,NORM_MINMAX);
            normalize(right_rect_monoR,right_rect_monoR,255.0,0.0,NORM_MINMAX);
            normalize(right_rect_monoG,right_rect_monoG,255.0,0.0,NORM_MINMAX);
            stereoMatch(left_rect_monoR, right_rect_monoR, disp16R, state.mode);
            stereoMatch(left_rect_monoG, right_rect_monoG, disp16G, state.mode);
            //set time stamp
            sec = (uint32_t)time_stamp/1000;
            nsec = (uint32_t)(time_stamp - sec*1000) * 1e6;
            ros::Time measurement_time(sec, nsec);
            ros::Time time_ref(0, 0);
            time_stamp_msg.header.stamp = measurement_time;
            time_stamp_msg.header.frame_id = left_frame_id_;
            time_stamp_msg.time_ref = time_ref;
            now = measurement_time;
            ROS_INFO_ONCE("Stereo Matched");
            
            cv::Mat combined16D(disp16R.rows,disp16R.cols,CV_16SC1);
            cv::Mat combined32D(disp16R.rows,disp16R.cols,CV_32FC1);
            for (int i = 0; i < combined16D.rows; i++)
            {
                for (int j = 0; j < combined16D.cols; j++)
                {
                    short valueR = disp16R.at<short>(i,j);
                    short valueG = disp16G.at<short>(i,j);
                    int tmpvalue = (valueR + valueG)/2;
                    combined16D.at<short>(i,j) = (short)tmpvalue;
                }
            }
            combined16D.convertTo(combined32D,CV_32FC1,1./16.);//true disparity
            ROS_INFO_ONCE("Combined Matrices");
            //cv::Mat_<cv::Vec3f> pointsD;  //point cloud generation: currently not used
            //cv::reprojectImageTo3D(combined32D, pointsD, Q, true, CV_32F);
            //depth map.
            cv::Mat depthMap(combined32D.rows,combined32D.cols,CV_32FC1);
            //float depthdenom = 48.;
            for (int i = 0; i < combined32D.rows; i++)
            {
                for (int j = 0; j < combined32D.cols; j++)
                {
                    float dispvalue = combined32D.at<float>(i,j);
                    if (dispvalue > 0.0f && dispvalue < 256.0f)
                    {   // depth = baseline * focal / disparity
                        float depth = Tx * focal / dispvalue;
                        depthMap.at<float>(i,j) = depth;
                    }
                    else
                    {
                        depthMap.at<float>(i,j)=0.0f;
                    }
                }
            }
            cv::Mat coloredDepth;
            depthMap.convertTo(coloredDepth,CV_8UC1);
            float scalefactor = 255.0f/15.0f; //detection range 15 meters
            for (int i = 0; i < depthMap.rows; i++)
            {
                for (int j = 0; j < depthMap.cols; j++)
                {
                    float depthvalue = depthMap.at<float>(i,j) * scalefactor;
                    if (depthvalue > 0.0f && depthvalue < 256.0f)
                    {   
                        coloredDepth.at<uchar>(i,j) = (uchar)depthvalue;
                    }
                    else
                    {
                        depthMap.at<uchar>(i,j)=0;
                    }
                }
            }       
            cv::applyColorMap(coloredDepth,coloredDepth,COLORMAP_TWILIGHT_SHIFTED);
            //publish depth (detection range 1 to 15 meters, beyond which is very off)
            cv_image.image = depthMap;
            cv_image.encoding = "32FC1";
            sensor_msgs::Image ros_image;
            cv_image.toImageMsg(ros_image);
            pubDepth.publish(ros_image);
            pubTimestamp.publish(time_stamp_msg);
            ROS_INFO_ONCE("Ready for Display");
            //only publish for calibration. If not subscribed, nothing is published.
            if (left_image_pub.getNumSubscribers() > 0) {
                publishImage(left_rgb, left_image_pub, "left_frame", now, sensor_msgs::image_encodings::BGR8);
            }
            if (right_image_pub.getNumSubscribers() > 0) {
                publishImage(right_rgb, right_image_pub, "right_frame", now, sensor_msgs::image_encodings::BGR8);
            }
            if (show_image_) {
                cv::namedWindow("colored depth");
                cv::setMouseCallback("colored depth",onMouse);
                visualizeDepth=depthMap;//print actual depth value when clicking jetdepth
                cv::imshow("colored depth",coloredDepth);
                cv::waitKey(1);
            }
        }
    }
    //publishes images, takes in Mat, publishes ros img
    void publishImage(cv::Mat img, image_transport::Publisher &pub_img, std::string img_frame_id, ros::Time t, std::string encoding_id)
    {
        pub_img.publish(imageToROSmsg(img, encoding_id, img_frame_id, t));
    }
    //conversion between Mat and ros msg.
    sensor_msgs::ImagePtr imageToROSmsg(cv::Mat img, const std::string encodingType, std::string frameId, ros::Time t)
    {
        sensor_msgs::ImagePtr ptr = boost::make_shared<sensor_msgs::Image>();
        sensor_msgs::Image& imgMessage = *ptr;
        imgMessage.header.stamp = t;
        imgMessage.header.frame_id = frameId;
        imgMessage.height = img.rows;
        imgMessage.width = img.cols;
        imgMessage.encoding = encodingType;
        int num = 1; //for endianness detection
        imgMessage.is_bigendian = !(*(char *) &num == 1);
        imgMessage.step = img.cols * img.elemSize();
        size_t size = imgMessage.step * img.rows;
        imgMessage.data.resize(size);
        //copy data to ros msg
        if (img.isContinuous())
            memcpy((char*) (&imgMessage.data[0]), img.data, size);
        else {
            uchar* opencvData = img.data;
            uchar* rosData = (uchar*) (&imgMessage.data[0]);
            for (unsigned int i = 0; i < img.rows; i++) {
                memcpy(rosData, opencvData, imgMessage.step);
                rosData += imgMessage.step;
                opencvData += img.step;
            }
        }
        return ptr;
    }
    //handles mouse up for debugging. shows value in terminal.
    static void onMouse(int event, int x, int y, int, void*)
    {
        if(event == EVENT_LBUTTONUP)
        {
            cout<<"Value = "<<visualizeDepth.at<float>(y,x)<<endl;
        }
    }
    void callback(ocams_1cgn::camConfig &config, uint32_t level) {
        ocams->uvc_control(config.exposure, config.gain, config.wb_blue, config.wb_red, config.auto_exposure);
        //set configurations
        state.mode = (MatchingAlg)config.stereo_algorithm;
        state.preFilterType = config.prefilter_type;
        if(config.prefilter_size % 2 == 0)
            config.prefilter_size--;
        state.preFilterSize = config.prefilter_size;
        state.preFilterCap = config.prefilter_cap;
        if( (config.stereo_algorithm == 0) && (config.correlation_window_size<5) )
        {
            config.correlation_window_size = 5;
        }
        if(config.correlation_window_size % 2 == 0)
        {
            config.correlation_window_size--;
        }
        state.SADWindowSize = config.correlation_window_size;
        state.minDisparity = config.min_disparity;
        if(config.disparity_range % 16 != 0)
            config.disparity_range = config.disparity_range/16*16;
        state.numDisparities = config.disparity_range;
        state.textureThreshold = config.texture_threshold;
        state.uniquenessRatio = config.uniqueness_ratio;
        state.speckleRange = config.speckle_range;
        state.speckleWindowSize = config.speckle_size;
        state.disp12MaxDiff = config.disp12MaxDiff;
        state.p1 = config.P1;
        state.p2 = config.P2;
    }
};
// program entry.
int main (int argc, char **argv)
{
    ros::init(argc, argv, "ocams_1cgn");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    std::string port = std::string("/dev/ttyACM0");
    int baud_rate    = 115200;
    std::string imu_mode = std::string("AMGQUA");
    ros::param::get("~port", port);
    ros::param::get("~imu_mode", imu_mode);
    ros::param::get("~baud_rate", baud_rate);
    MyAhrsDriverForROS sensor(port, baud_rate);
    if(sensor.initialize(imu_mode) == false)//error of false initialization and sudden death of process.
    {
        ROS_ERROR("%s\r\n", "If you see this error, please execute:\r\n \
        $ sudo vi /etc/udev/rules.d/99-ttyacms.rules\r\n \
        ATTRS{idVendor}==\"04b4\" ATTRS{idProduct}==\"00f9\", MODE=\"0666\", ENV{ID_MM_DEVICE_IGNORE}=\"1\"\r\n \
        ATTRS{idVendor}==\"04b4\" ATTRS{idProduct}==\"00f8\", MODE=\"0666\", ENV{ID_MM_DEVICE_IGNORE}=\"1\"\r\n \
        $ sudo udevadm control -R\r\n");
    }
    oCamStereoROS ocams_ros;
    ros::spin();
    return 0;
}
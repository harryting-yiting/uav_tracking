#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Image.h>

#include <tf2/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <iostream>
#include <string>

extern "C"{
    #include "apriltag.h"
    #include "apriltag_pose.h"
    #include "tag36h11.h"
    #include "tag25h9.h"
    #include "tag16h5.h"
    #include "tagCircle21h7.h"
    #include "tagCircle49h12.h"
    #include "tagCustom48h12.h"
    #include "tagStandard41h12.h"
    #include "tagStandard52h13.h"
    #include "common/getopt.h"
}



cv_bridge::CvImagePtr cv_ptr_;
cv::Mat cv_mat_;
bool init_cam_ = false;
bool init_caminfo_ = false;
bool stop_detec_flag_ = false;
// conver sensor msgs into cv_ptr_
void getCapCallback(const sensor_msgs::ImageConstPtr & image_msg);

// get camera info
void getCaminfoCallback(const sensor_msgs::CameraInfo & caminfo_msg);

void outputArrAsMat(const double *data, int rows, int cols);

void setTfTransfrom(const double *translation, const double *rotation, tf2::Transform &tf_transform);

void getCapCallback(const sensor_msgs::ImageConstPtr & image_msg)
{
    cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    cv_mat_ = cv_ptr_->image;
    init_cam_ = true;
}

void getCaminfoCallback(const sensor_msgs::CameraInfo & caminfo_msg)
{
    init_caminfo_ = true;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "apriltag_node");
    ros::NodeHandle nh;
    
    ros::Subscriber cam_sub_ = nh.subscribe
            ("/iris_fpv_cam/usb_cam/image_raw", 1, &getCapCallback);
    ros::Subscriber caminfo_sub = nh.subscribe
            ("/iris_fpv_cam/usb_cam/camera_info", 1, &getCaminfoCallback);
    
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose>
            ("april_tag/pose",1);

    // publish rate
    ros::Rate rate(10);

    static tf2_ros::TransformBroadcaster br;
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 1, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tagStandard41h12", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    cv::Mat frame, gray;
    cv::namedWindow("test image", cv::WINDOW_AUTOSIZE);
    while(ros::ok())
    {
        ros::spinOnce();
        if(init_cam_ && init_caminfo_)
        {
            cv::cvtColor(cv_mat_, gray, cv::COLOR_BGR2GRAY);

            // Make an image_u8_t header for the Mat data
            image_u8_t im = { .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
            };
            
            zarray_t *detections = apriltag_detector_detect(td, &im);
            std::cout << zarray_size(detections) << " tags detected" << std::endl;

            // Draw detection outlines
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                line(cv_mat_, cv::Point(det->p[0][0], det->p[0][1]),
                        cv::Point(det->p[1][0], det->p[1][1]),
                        cv::Scalar(0, 0xff, 0), 2);
                line(cv_mat_, cv::Point(det->p[0][0], det->p[0][1]),
                        cv::Point(det->p[3][0], det->p[3][1]),
                        cv::Scalar(0, 0, 0xff), 2);
                line(cv_mat_, cv::Point(det->p[1][0], det->p[1][1]),
                        cv::Point(det->p[2][0], det->p[2][1]),
                        cv::Scalar(0xff, 0, 0), 2);
                line(cv_mat_, cv::Point(det->p[2][0], det->p[2][1]),
                        cv::Point(det->p[3][0], det->p[3][1]),
                        cv::Scalar(0xff, 0, 0), 2);

                std::stringstream ss;
                ss << det->id;
                cv::String text = ss.str();
                int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                cv::Size textsize = getTextSize(text, fontface, fontscale, 2,
                                                &baseline);
                putText(frame, text, cv::Point(det->c[0]-textsize.width/2,
                                        det->c[1]+textsize.height/2),
                        fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
                
                // for pose estimation
                apriltag_detection_info_t detection_info;
                detection_info.det = det;
                detection_info.tagsize = 0.2;  //m
                detection_info.fx = 277.191356;
                detection_info.fy = 277.191356;
                detection_info.cx = 320.5;
                detection_info.cy = 240.5;

                apriltag_pose_t new_pose;
                
                estimate_pose_for_tag_homography(& detection_info, &new_pose);

                std::cout << "[T] " << std::endl;
                outputArrAsMat(new_pose.t->data, new_pose.t->nrows, new_pose.t->ncols);
                std::cout << "[R] " << std::endl;
                outputArrAsMat(new_pose.R->data, new_pose.R->nrows, new_pose.R->ncols); 
                
                tf2::Transform tf2_tranform;
                setTfTransfrom(new_pose.t->data, new_pose.R->data, tf2_tranform);
                tf2::Stamped<tf2::Transform> tf2_stamped_transform(tf2_tranform, ros::Time::now(), "fpv_cam");

                geometry_msgs::TransformStamped t_stampedmsg;
                tf2::convert(tf2_stamped_transform, t_stampedmsg);
                t_stampedmsg.child_frame_id = "tag" + std::to_string(i);
                br.sendTransform(t_stampedmsg);
            }   

            apriltag_detections_destroy(detections);
            cv::imshow("test image", cv_mat_);
            if (cv::waitKey(30) == 1)
                break;
        }
        rate.sleep();

    }
    return 0;
}

void outputArrAsMat(const double *data, int rows, int cols)
{
    for(int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            std::cout << data[i*rows + j] << " , ";
        }

        std::cout << std::endl;
    }
}

void setTfTransfrom(const double *translation, const double *rotation, tf2::Transform &tf_transform)
{
    // set tranform
    tf2::Matrix3x3 tf_rotation(rotation[0], rotation[1], rotation[2] 
                                , rotation[3], rotation[4], rotation[5]
                                , rotation[6], rotation[7], rotation[8]);
    tf2::Vector3 tf_translation(translation[0], translation[1], translation[2]);


    tf_transform.setOrigin(tf_translation);
    tf_transform.setBasis(tf_rotation);
}
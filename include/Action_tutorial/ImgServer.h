#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <mutex>
#include <queue>
#include <thread>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/stitching.hpp"
#include "sensor_msgs/Image.h"

// Action Server Related
#include <actionlib/server/simple_action_server.h>
#include <Action_tutorial/imageAction.h>

bool success;

class ImgServer
{
    public:
        ImgServer(ros::NodeHandle &n);
        void execute(const Action_tutorial::imageGoalConstPtr &goal);
        cv::Mat sensorMsg2cvMat(const sensor_msgs::Image img_msg);
        sensor_msgs::Image cvMat2sensorMsg(cv::Mat image, std_msgs::Header header);
        void makePanoImg(cv::Mat image1, cv::Mat image2, cv::Mat &pano_img, int index);

    private:
        ros::NodeHandle nh;
        actionlib::SimpleActionServer<Action_tutorial::imageAction> server;
        Action_tutorial::imageFeedback feedback;
        Action_tutorial::imageResult result;
        
        cv::Mat cv_img1, cv_img2;
        cv::Mat cv_pano_img;
};
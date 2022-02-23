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

// Action Client Related 
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <Action_tutorial/imageAction.h>


int sequence = 0;

class ImgClient
{
    public:  
        ImgClient(ros::NodeHandle &n);
        ~ImgClient();
        void img1_callback(const sensor_msgs::ImageConstPtr &img1);
        void img2_callback(const sensor_msgs::ImageConstPtr &img2);
        sensor_msgs::Image ImgPtr2Img(const sensor_msgs::ImageConstPtr &ptrimg);
        void syncImg();
        
    private:
        std::mutex m_buf;
        std::queue<sensor_msgs::ImageConstPtr> image1_buf;
        std::queue<sensor_msgs::ImageConstPtr> image2_buf;
        std::thread sync;

        ros::NodeHandle nh;
        ros::Publisher pub_pano;
        ros::Subscriber sub_img1;
        ros::Subscriber sub_img2;
        actionlib::SimpleActionClient<Action_tutorial::imageAction> client;
        Action_tutorial::imageGoal goal;
        Action_tutorial::imageResultConstPtr result;
};
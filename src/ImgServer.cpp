#include "Action_tutorial/ImgServer.h"

ImgServer::ImgServer(ros::NodeHandle &n):nh(n), server(nh, "imgAction", boost::bind(&ImgServer::execute, this, _1), false)
{
    server.start();
}

void ImgServer::execute(const Action_tutorial::imageGoalConstPtr &goal)
{
    ros::Rate r(1);
    success = false;

    int cur_index = goal -> index;
    sensor_msgs::Image image1 = goal -> img1;
    sensor_msgs::Image image2 = goal -> img2;

    ROS_WARN("IMG1 width: %d", image1.width);
    ROS_WARN("IMG2 width: %d", image2.width);

    cv_img1 = sensorMsg2cvMat(image1);
    cv_img2 = sensorMsg2cvMat(image2);

    ROS_WARN("IMG1 Size: %d", cv_img1.cols);
    ROS_WARN("IMG2 Size: %d", cv_img2.cols);

    makePanoImg(cv_img1, cv_img2, cv_pano_img, cur_index);
    
    if(success)
    {
        sensor_msgs::Image pano = cvMat2sensorMsg(cv_pano_img, image1.header);
        result.pano_index = cur_index;
        result.panoImg = pano;
        ROS_WARN("Success Panorama Image !!");

        server.setSucceeded(result);
    }
}

cv::Mat ImgServer::sensorMsg2cvMat(const sensor_msgs::Image img_msg)
{
    cv_bridge::CvImageConstPtr ptr;

    sensor_msgs::Image cvt_img;
    cvt_img.header = img_msg.header;
    cvt_img.height = img_msg.height;
    cvt_img.width = img_msg.width;
    cvt_img.is_bigendian = img_msg.is_bigendian;
    cvt_img.step = img_msg.step;
    cvt_img.data = img_msg.data;
    cvt_img.encoding = "mono8";

    ptr = cv_bridge::toCvCopy(cvt_img, sensor_msgs::image_encodings::BGRA8);

    cv::Mat img = ptr->image.clone();
	cv::cvtColor(img, img, cv::COLOR_BGRA2BGR);

    return img;
}

sensor_msgs::Image ImgServer::cvMat2sensorMsg(cv::Mat image, std_msgs::Header header)
{
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img;

	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
	img_bridge.toImageMsg(img);

	return img;
}

void ImgServer::makePanoImg(cv::Mat image1, cv::Mat image2, cv::Mat &pano_img, int index)
{
    // Define mode for stitching as panorama
	cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;

	// Check image read correctly
	if(image1.cols == 0 && image2.cols == 0)
	{
		ROS_WARN("Empty Image1 and Image2");
		return;
	}
	else if(image1.cols == 0)
	{
		ROS_WARN("Empty Image1");
		return;
	}
	else if(image2.cols == 0)
	{
		ROS_WARN("Empty Image2");
		return;
	} 
		
	std::vector<cv::Mat> imgs;
	imgs.push_back(image1);
	imgs.push_back(image2);

	cv::Mat panorama;

	cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode, false);
    cv::Stitcher::Status status = stitcher->stitch(imgs, panorama);

	if (status != cv::Stitcher::OK)
    {
        // Check if images could not be stiched
        // status is OK if images are stiched successfully
        return;
    }

    std::string save_pano_dir = "~/Action_tutorial/image_result/pano_" + std::to_string(index) + "_.png";
    cv::imwrite(save_pano_dir, panorama);

	printf("Successfully make panorama image !!\n");
	pano_img = panorama;
    success = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "server");
    ros::NodeHandle n;

    ImgServer ImgServer(n);
    ros::spin();

    return 0;
}

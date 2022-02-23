#include "Action_tutorial/ImgClient.h"

#include <iomanip>

ImgClient::ImgClient(ros::NodeHandle &n)
: nh(n), 
  client("imgAction", true) 
//   pub_pano(nh.advertise<sensor_msgs::Image>("PanoramaImage", 100)),
//   sub_img1(nh.subscribe("/img1/camera/infra1/image_rect_raw", 1000, &ImgClient::img1_callback, this)),
//   sub_img2(nh.subscribe("/img2/camera/infra1/image_rect_raw", 1000, &ImgClient::img2_callback, this))
{
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer(); //will wait for infinite time

    pub_pano = nh.advertise<sensor_msgs::Image>("PanoramaImage", 100);

    sub_img1 = nh.subscribe<sensor_msgs::Image>("/img1/camera/infra1/image_rect_raw", 1000, &ImgClient::img1_callback, this);
    sub_img2 = nh.subscribe<sensor_msgs::Image>("/img2/camera/infra1/image_rect_raw", 1000, &ImgClient::img2_callback, this);
    
    // std::thread sync{syncImg};
    
    sync = std::thread{&ImgClient::syncImg, this};

}

ImgClient::~ImgClient()
{
    sync.detach();
}

sensor_msgs::Image ImgClient::ImgPtr2Img(const sensor_msgs::ImageConstPtr &ptrimg)
{
    sensor_msgs::Image img;
    img.header = ptrimg->header;
    img.height = ptrimg->height;
    img.width = ptrimg->width;
    img.is_bigendian = ptrimg->is_bigendian;
    img.step = ptrimg->step;
    img.data = ptrimg->data;
    img.encoding = "mono8";

    return img;    
}

void ImgClient::img1_callback(const sensor_msgs::ImageConstPtr &img1)
{
    m_buf.lock();
	image1_buf.push(img1);
	m_buf.unlock();
}

void ImgClient::img2_callback(const sensor_msgs::ImageConstPtr &img2)
{
    m_buf.lock();
	image2_buf.push(img2);
	m_buf.unlock();
}

void ImgClient::syncImg()
{
    while(1)
	{
        sequence++;

        Action_tutorial::imageGoal goal;
        sensor_msgs::Image img1, img2;

        // ROS_WARN("current sequence: %i", sequence);
        // ROS_WARN("size of image buf1: %i", image1_buf.size());
        // ROS_WARN("size of image buf2: %i", image2_buf.size());

		m_buf.lock();
		while(!image1_buf.empty() && !image2_buf.empty())
		{
			double time1 = image1_buf.front() -> header.stamp.toSec();
			double time2 = image2_buf.front() -> header.stamp.toSec();

            // std::cout << std::setprecision(18);
            // std::cout << "time of image1_buf front: " << time1 << "time of image2_buf front: " << time2 << std::endl;

			if(time1 < time2 - 0.03)
			{
				image1_buf.pop();
				// printf("throw img1\n");
			}
			else if(time1 > time2 + 0.03)
			{
				image2_buf.pop();
				// printf("throw img2\n");
			}
			else
			{
                // ROS_WARN("size of image buf1 Final: %i", image1_buf.size());
                // ROS_WARN("size of image buf2 Final: %i", image2_buf.size());

                img1 = ImgPtr2Img(image1_buf.front());
                img2 = ImgPtr2Img(image2_buf.front());

                while(!image1_buf.empty())
                {
                    image1_buf.pop();
                }
                while(!image2_buf.empty())
                {
                    image2_buf.pop();                    
                }

				//printf("find img0 and img1\n");
			}
		}
		m_buf.unlock();


        // std::cout << "time of image1_buf front: " << img1.header.stamp.toSec() << std::endl;
        // std::cout << "time of image2_buf front: " << img2.header.stamp.toSec() << std::endl;

        goal.index = sequence;
        goal.img1 = img1;
        goal.img2 = img2;
        client.sendGoal(goal);

        bool finished_before_timeout = client.waitForResult(ros::Duration(5));

        if(finished_before_timeout)
        {
            result = client.getResult();
            ROS_WARN("Get Pano Img Index: %i", result -> pano_index);
            pub_pano.publish(result -> panoImg);
        }
        else
            ROS_INFO("Action did not finish before the time out.");

        // std::chrono::milliseconds dura(3);
        // std::this_thread::sleep_for(dura);
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "client");
	ros::NodeHandle n;

    ImgClient client(n);
    ros::spin();

    return 0;
}
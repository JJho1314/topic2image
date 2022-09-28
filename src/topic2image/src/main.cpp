#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include "autoware_msgs/PointsImage.h"
#include "autoware_msgs/ProjectionMatrix.h"
//#include "autoware_msgs/CameraExtrinsic.h"
#include <image_transport/image_transport.h>
//时间同步
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <points_image.hpp>
#include <opencv2/opencv.hpp>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

static cv::Mat cameraExtrinsicMat;
static cv::Mat cameraMat;
static cv::Mat distCoeff;
static cv::Size imageSize;

static image_transport::Publisher pub;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> testSyncPolicy;

static void projection_callback(const autoware_msgs::ProjectionMatrix &msg)
{
    cameraExtrinsicMat = cv::Mat(4, 4, CV_64F);
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            cameraExtrinsicMat.at<double>(row, col) = msg.projection_matrix[row * 4 + col];
        }
    }
    resetMatrix();
}

static void intrinsic_callback(const sensor_msgs::CameraInfo &msg)
{
    imageSize.height = msg.height;
    imageSize.width = msg.width;

    cameraMat = cv::Mat(3, 3, CV_64F);
    for (int row = 0; row < 3; row++)
    {
        for (int col = 0; col < 3; col++)
        {
            cameraMat.at<double>(row, col) = msg.K[row * 3 + col];
        }
    }

    distCoeff = cv::Mat(1, 5, CV_64F);
    for (int col = 0; col < 5; col++)
    {
        distCoeff.at<double>(col) = msg.D[col];
    }
    resetMatrix();
}

cv::Mat Array2Mat(std::vector<float> data, int row, int col)
{
    unsigned char(*array)[col] = (unsigned char(*)[col])malloc(sizeof(unsigned char) * row * col);
    // unsigned char **array = new unsigned char[row][col];
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            array[i][j] = (unsigned char)(data[i * col + j] * 255);
        }
    }
    std::vector<cv::Mat> channels;
    cv::Mat img(row, col, CV_8UC1, (unsigned char *)array);
    return img;
}

void callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::PointCloud2ConstPtr &lidar_msg)
{
    if (cameraExtrinsicMat.empty() || cameraMat.empty() || distCoeff.empty() || imageSize.height == 0 ||
        imageSize.width == 0)
    {
        ROS_INFO("[points2image]Looks like camera_info or projection_matrix are not being published.. Please check that "
                 "both are running..");
        return;
    }
    std::cout << "image_height: " << imageSize.height << std::endl;
    std::cout << "image_width: " << imageSize.width << std::endl;
    std::cout << "cameraExtrinsicMat: " << cameraExtrinsicMat << std::endl;
    std::cout << "cameraMat: " << cameraMat << std::endl;
    cv::Mat recievedImg;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::BGR8);
    recievedImg = cv_ptr->image.clone();

    sensor_msgs::ImagePtr pub_msg = pointcloud2_to_image(recievedImg, lidar_msg, cameraExtrinsicMat, cameraMat, distCoeff, imageSize);
    // cv::Mat dest = Array2Mat(pub_msg.intensity,pub_msg.image_height,pub_msg.image_width);
    // cv::imwrite("dest.png",dest);
    // ROS_INFO("Convert a image!");
    pub.publish(pub_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "points2image");
    ros::NodeHandle nh;

    ros::NodeHandle private_nh("~");
    image_transport::ImageTransport it(nh);

    std::string camera_info_topic_str;
    std::string projection_matrix_topic;
    std::string pub_topic_str = "/points2image";

    private_nh.param<std::string>("projection_matrix_topic", projection_matrix_topic, "/projection_matrix");
    private_nh.param<std::string>("camera_info_topic", camera_info_topic_str, "/camera_info");

    ros::Publisher camera_pub;
    ros::Publisher lidar_pub;

    std::string name_space_str = ros::this_node::getNamespace();

    if (name_space_str != "/")
    {
        if (name_space_str.substr(0, 2) == "//")
        {
            /* if name space obtained by ros::this::node::getNamespace()
               starts with "//", delete one of them */
            name_space_str.erase(name_space_str.begin());
        }
        pub_topic_str = name_space_str + pub_topic_str;
        projection_matrix_topic = name_space_str + projection_matrix_topic;
        camera_info_topic_str = name_space_str + camera_info_topic_str;
    }

    std::string points_topic;
    if (private_nh.getParam("points_node", points_topic))
    {
        ROS_INFO("[points2image]Setting points node to %s", points_topic.c_str());
    }
    else
    {
        ROS_INFO("[points2image]No points node received, defaulting to points_cluster, you can use _points_node:=YOUR_TOPIC");
        points_topic = "/points_cluster";
    }

    ROS_INFO("[points2image]Publishing to... %s", pub_topic_str.c_str());
    pub = it.advertise(pub_topic_str, 100);

    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub; //雷达订阅
    message_filters::Subscriber<sensor_msgs::Image> camera_sub;      //相机订阅

    ros::Publisher img_pub;
    ros::Publisher pointcloud_pub;

    //订阅话题
    //发布的话题
    // img_pub = n.advertise<sensor_msgs::Image>("image", 1000);
    // pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("point", 1000);

    //订阅的话题
    message_filters::Subscriber<sensor_msgs::Image> img_sub(nh, "/gmsl_cam/image_raw", 1);     // topic1 输入 "/gmsl_cam/image_raw"
    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub(nh, points_topic, 1); // topic2 输入

    message_filters::Synchronizer<testSyncPolicy> sync(testSyncPolicy(10), img_sub, pointcloud_sub); // 同步
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // ros::Subscriber sub = n.subscribe(points_topic, 1, callback);

    ROS_INFO("[points2image]Subscribing to... %s", projection_matrix_topic.c_str());
    ros::Subscriber projection = nh.subscribe(projection_matrix_topic, 1, projection_callback);
    ROS_INFO("[points2image]Subscribing to... %s", camera_info_topic_str.c_str());
    ros::Subscriber intrinsic = nh.subscribe(camera_info_topic_str, 1, intrinsic_callback);

    while (ros::ok())
    {
        //以实时发布的方式发布

        ros::spinOnce();
    }

    return 0;
}

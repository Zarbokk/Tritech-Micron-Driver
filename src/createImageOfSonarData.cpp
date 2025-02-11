#include "rclcpp/rclcpp.hpp"
//#include "ping360_sonar/SonarEcho.h"
#include <micron_driver_ros/msg/scan_line.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.h"
#include <cv_bridge/cv_bridge.h>

double rotationOfSonarOnRobot = 0;//maybe in rad
cv::Mat sonarImage;
rclcpp::Node::SharedPtr g_node = nullptr;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisherSonarImage;
double lastAngle = 0;
std::vector<double> linspace(double start_in, double end_in, int num_in) {
    if (num_in < 0) {
        std::cout << "number of linspace negative" << std::endl;
        exit(-1);
    }
    std::vector<double> linspaced;

    double start = start_in;
    double end = end_in;
    auto num = (double) num_in;

    if (num == 0) { return linspaced; }
    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);//stepSize

    for (int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
    // are exactly the same as the input
    return linspaced;
}

void imageDataGenerationCallback(const micron_driver_ros::msg::ScanLine::SharedPtr msg){
//    std::cout << "msg: " << std::endl;
//    std::cout << msg->angle << std::endl;
//    std::cout << msg->number_of_samples << std::endl;
//    std::cout << msg->range << std::endl;
//    std::cout << msg->step_size << std::endl;



//    msg->number_of_samples
//    msg->range;
    //sonarImage.at<uchar>(3,3) = 255;

    double linear_factor = ((double)msg->bins.size()) / ((double)sonarImage.size[0] / 2.0);
    double stepSize = abs(msg->angle -lastAngle);//1;
    if(stepSize>200){
        std::cout << stepSize << std::endl;
        stepSize = abs(stepSize-360.0);
    }
    lastAngle = msg->angle;

    for(int i = 1 ; i<sonarImage.size[0]/2;i++){
        double color=0;
        if (i<sonarImage.size[0]){
            color = msg->bins.at((int)(i * linear_factor - 1)).intensity;
        }
//        double stepSize = (msg->angle-lastAngle);//1;
        std::vector<double> linspaceVector = linspace(-stepSize/2,stepSize/2,10);
        for(const auto& value: linspaceVector) {
//            std::cout << "start" << std::endl;
//            std::cout << msg->angle << std::endl;
//            std::cout << rotationOfSonarOnRobot << std::endl;
//            std::cout << value << std::endl;

            //minus because of the coordinate change from z to top to z to bottom
            double theta = 2 * M_PI * (msg->angle + value + rotationOfSonarOnRobot) / 360.0;// @TODO probably wrong, needs to be in rad. so double theta = msg->angle + value + rotationOfSonarOnRobot
            double x = i * cos(theta);
            double y = i * sin(theta);
            sonarImage.at<uchar>((int)(((double)sonarImage.size[0] / 2.0) - x)-1,(int)(((double)sonarImage.size[0] / 2.0) + y)-1) = color*0.9;
//            std::cout << theta << std::endl;
        }
    }
    cv::rectangle(sonarImage,cv::Point(0,0),cv::Point(65,65),0,cv::FILLED);
    std::string tmp = std::to_string((int)(msg->bins.back().distance));
    cv::putText(sonarImage,tmp,cv::Point(2,40),cv::FONT_ITALIC,1,255);


//    cv::imshow("After",sonarImage);
//    cv::waitKey(1);


    sensor_msgs::msg::Image::SharedPtr imageMessage = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", sonarImage).toImageMsg();
    publisherSonarImage->publish(*imageMessage);
    lastAngle = msg->angle;
}

int main(int argc, char *argv[])
{
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

    rclcpp::init(argc, argv);
    g_node= rclcpp::Node::make_shared("sonar_vis_node");


//    rclcpp::start();
//    rclcpp::NodeHandle n_;
    //has to be squared
    int sizeMat = 500;
    sonarImage = cv::Mat(sizeMat, sizeMat, CV_8UC1, cv::Scalar(0));

//    rclcpp::Subscriber subscriberDataSonar = n_.subscribe("sonar/intensity",1000,imageDataGenerationCallback);
    auto subscription =
            g_node->create_subscription<micron_driver_ros::msg::ScanLine>("/tritech_sonar/scan_lines", qos, imageDataGenerationCallback);
//    node->create_subscription<ping360_sonar_msgs::msg::SonarEcho>("scan_echo", qos,std::bind(&imageDataGenerationCallback,std::placeholders::_1));
    publisherSonarImage = g_node->create_publisher<sensor_msgs::msg::Image>("micron/image2", 10);

    rclcpp::spin(g_node);

    return 1;
}
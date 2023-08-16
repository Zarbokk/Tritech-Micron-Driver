//
// Created by tim-linux on 04.01.22.
//

#include "ros/ros.h"
//#include "ping360_sonar/SonarEcho.h"
//#include "commonbluerovmsg/SonarEcho2.h"
#include <micron_driver_ros/ScanLine.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>

double rotationOfSonarOnRobot = 200;
cv::Mat sonarImage;
ros::Publisher publisher;

double lastAngle = 1;
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

void imageDataGenerationCallback(const micron_driver_ros::ScanLine::ConstPtr &msg){
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
    lastAngle = msg->angle;

    for(int i = 1 ; i<sonarImage.size[0]/2;i++){
        double color=0;
        if (i<sonarImage.size[0]){

            color = msg->bins.at((int)(i * linear_factor - 1)).intensity;
        }



        std::vector<double> linspaceVector = linspace(-stepSize/2,stepSize/2,10);
        for(const auto& value: linspaceVector) {
            //minus because of the coordinate change from z to top to z to bottom
            double theta = 2 * M_PI * (msg->angle + value + rotationOfSonarOnRobot) / 360.0;
            double x = i * cos(theta);
            double y = i * sin(theta);
            sonarImage.at<uchar>((int)(((double)sonarImage.size[0] / 2.0) - x)-1,(int)(((double)sonarImage.size[0] / 2.0) + y)-1) = color*1.2;
        }
    }
    cv::rectangle(sonarImage,cv::Point(0,0),cv::Point(65,65),0,cv::FILLED);
    std::string tmp = std::to_string(msg->bins.back().distance);
    cv::putText(sonarImage,tmp,cv::Point(2,40),cv::FONT_ITALIC,1,255);


//    cv::imshow("After",sonarImage);
//    cv::waitKey(1);


    sensor_msgs::ImagePtr imageMessage = cv_bridge::CvImage(std_msgs::Header(), "mono8", sonarImage).toImageMsg();

    publisher.publish(imageMessage);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "conversionofsonardatatoimageMicron");
    ros::start();
    ros::NodeHandle n_;
    //has to be squared
    int sizeMat = 500;
    sonarImage = cv::Mat(sizeMat, sizeMat, CV_8UC1, cv::Scalar(0));

    std::cout << "/1" << std::endl;
    publisher = n_.advertise<sensor_msgs::Image>("/micron_driver/tritech_sonar/image", 10);
    std::cout << "2" << std::endl;

    ros::Subscriber subscriberDataSonar = n_.subscribe("/micron_driver/tritech_sonar/scan_line",1000,imageDataGenerationCallback);
//    ros::Subscriber subscriberDataSonar = n_.subscribe("ping360_node/sonar/data",1000,imageDataGenerationCallback);
    std::cout << "3" << std::endl;

    ros::spin();
    return 1;
}

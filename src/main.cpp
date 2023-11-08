
//rclcpp includes 
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

//Messages includes
#include "std_msgs/msg/string.hpp"
#include <micron_driver_ros/msg/scan_line.hpp>
//Service includes 
//#include <micron_driver_ros/msg/sonar_reconfig.h>

//C++ includes 
#include <sstream>
#include "stdint.h"

//Code includes
#include <micron_driver_ros/Serial.h>
#include <micron_driver_ros/tritech_micron_driver.h>
#include <micron_driver_ros/math.h>

//typedef micron_driver_ros::msg::ScanLine micron_driver_ros::msg::IntensityBin;
//typedef micron_driver_ros::msg::IntensityBin _IntensityBinMsgType;
//typedef float float;
//typedef float _AngleType;
//typedef std::vector<uint8_t> _IntensityBinsRawType;

class TritechMicron : public rclcpp::Node
{
public: 

	//Constructor
	TritechMicron() : Node("micron_publisher") {
        sleep(2);
        this->declare_parameter<std::string>("/micron_driver/frame_id_", "micron");
        this->declare_parameter<std::string>("/micron_driver/port_", "/dev/ttyUSB0");
        this->declare_parameter<int>("/micron_driver/num_bins_", 200);
        this->declare_parameter<double>("/micron_driver/range_", 5);

        this->declare_parameter<double>("/micron_driver/velocity_of_sound_", 1500);
        this->declare_parameter<int>("/micron_driver/angle_step_size_", 32);
        this->declare_parameter<int>("/micron_driver/leftLimit_", 1);
        this->declare_parameter<int>("/micron_driver/rightLimit_", 6399);

        this->declare_parameter<bool>("/micron_driver/use_debug_mode", false);
        this->declare_parameter<bool>("/micron_driver/simulate_", false);


        frame_id_ = this->get_parameter("/micron_driver/frame_id_").as_string();
        port_ = this->get_parameter("/micron_driver/port_").as_string();
        num_bins_ = this->get_parameter("/micron_driver/num_bins_").as_int();
        range_ = this->get_parameter("/micron_driver/range_").as_double();
        velocity_of_sound_ = this->get_parameter("/micron_driver/velocity_of_sound_").as_double();
        angle_step_size_ = this->get_parameter("/micron_driver/angle_step_size_").as_int();
        leftLimit_ = this->get_parameter("/micron_driver/leftLimit_").as_int();
        rightLimit_ = this->get_parameter("/micron_driver/rightLimit_").as_int();

        use_debug_mode = this->get_parameter("/micron_driver/use_debug_mode").as_bool();
        simulate_ = this->get_parameter("/micron_driver/simulate_").as_bool();


        this->scan_line_pub_ = this->create_publisher<micron_driver_ros::msg::ScanLine>("tritech_sonar/scan_lines", 4);
        driver_ = new TritechMicronDriver( this->num_bins_, this->range_, this->velocity_of_sound_, this->angle_step_size_, this->leftLimit_, this->rightLimit_, this->use_debug_mode);
        driver_->registerScanLineCallback( std::bind( &TritechMicron::publish,this,std::placeholders::_1,  std::placeholders::_2,std::placeholders::_3 ) );
        uint8_t angle_step_size_byte = std::max(1, std::min(255, angle_step_size_));

        if ( !driver_->connect( port_.c_str()) )
        {
            RCLCPP_INFO(this->get_logger(), "Could not connect to device;" );
        }

        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&TritechMicron::parametersCallback, this, std::placeholders::_1));
	}

	//Destructor
	~TritechMicron()
	{

		if ( driver_ )
		{
            RCLCPP_INFO(this->get_logger(), "Disconnecting Sonar!" );
			driver_->disconnect();
			delete driver_;
		}
	}

	rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
   {
       rcl_interfaces::msg::SetParametersResult result;
       result.successful = false;
       result.reason = "";

       for(const auto &param : parameters)
       {
           if(param.get_name() == "/micron_driver/frame_id_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
               {
                   frame_id_ = param.as_string();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/port_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
               {
                   port_ = param.as_string();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/num_bins_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
               {
                   num_bins_ = param.as_int();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/range_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
               {
                   range_ = param.as_double();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/velocity_of_sound_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
               {
                   velocity_of_sound_ = param.as_double();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/angle_step_size_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
               {
                   angle_step_size_ = param.as_int();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/leftLimit_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
               {
                   leftLimit_ = param.as_int();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/rightLimit_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
               {
                   rightLimit_ = param.as_int();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/use_debug_mode")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
               {
                   use_debug_mode = param.as_bool();
                   result.successful = true;
               }
           }
           if(param.get_name() == "/micron_driver/simulate_")
           {
               if(param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
               {
                   simulate_ = param.as_bool();
                   result.successful = true;
               }
           }
       }
       if (result.successful == true){
            driver_->reconfigure(num_bins_, range_, velocity_of_sound_, angle_step_size_, leftLimit_, rightLimit_);
       }

       return result;
   }
//	bool reconfig (micron_driver_ros::sonar_reconfig::Request &req,micron_driver_ros::sonar_reconfig::Response &resp )
//	{
//	 driver_->reconfigure(req.nbins,req.range, req.vos, req.angle_step_size, req.leftLimit, req.rightLimit);
//
//	return true;
//	}

	/* This function is a callback associated with the reception of a Scanline Message from the Sonar. 
	   It takes the Sonar scanline, formats it as a Scanline message and publishes it on the corresponding 
	   topic. */
	void publish( float scan_angle,float bin_distance_step, std::vector<uint8_t>  intensity_bins )
		{
//            std::cout << "publishing message" << std::endl;
	
//			micron_driver_ros::msg::IntensityBin::Ptr scan_line_msg( new micron_driver_ros::msg::IntensityBin );
            micron_driver_ros::msg::ScanLine scan_line_msg = micron_driver_ros::msg::ScanLine();

            scan_line_msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
			scan_line_msg.header.frame_id = frame_id_;
			scan_line_msg.angle = scan_angle;
			scan_line_msg.bin_distance_step = bin_distance_step;

			scan_line_msg.bins.reserve( intensity_bins.size() );

			for ( int i = 0; i < intensity_bins.size(); ++i )
			{
                micron_driver_ros::msg::IntensityBin bin;
				bin.distance = bin_distance_step * ( i + 1 );
				bin.intensity = intensity_bins[i];
				scan_line_msg.bins.push_back( bin );
			}
//            this->scan_line_pub_->publish(sca)
			this->scan_line_pub_->publish( scan_line_msg );

		}

		//Create publisher for the sonar scanlines
    rclcpp::Publisher<micron_driver_ros::msg::ScanLine>::SharedPtr scan_line_pub_;

	//Sonar parameters: Will be read from parameter server.
	std::string frame_id_ ;
	std::string port_ ;
	int num_bins_ ;
	double range_;
	double velocity_of_sound_;
	int angle_step_size_;
	int leftLimit_;
	int rightLimit_;
	bool use_debug_mode;
	bool simulate_;
	TritechMicronDriver * driver_;

	//Sonar simulation parameters
	int simulate_num_bins_ ;
	double simulate_bin_distance_step_;
	double simulate_distance;
	int simulate_intensity;
	double simulate_intensity_variance;
	bool simulate_use_manual_angle  ;
	double simulate_manual_angle  ;
	double simulate_scan_angle_velocity;
	float scan_angle;

	//Parameter reconfigure callback handle
	OnSetParametersCallbackHandle::SharedPtr callback_handle_;

}; //End of class



int main(int argc, char **argv)
{


    rclcpp::init(argc, argv);
    std::cout << "starting: " << std::endl;
    rclcpp::spin(std::make_shared<TritechMicron>());
    std::cout << "after " << std::endl;
    rclcpp::shutdown();
    return 0;
	
}






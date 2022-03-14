#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>


#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <linux/if_packet.h>
#include <thread>
#include <chrono>
#include <linux/reboot.h>
#include <sys/reboot.h>


#include <rclcpp/rclcpp.hpp>
//#include <rclcpp/console.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/empty.hpp>

#define Phoenix_No_WPI // remove WPI dependencies
#include <ctre/Phoenix.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/unmanaged/Unmanaged.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>
#include <ctre/phoenix/cci/Diagnostics_CCI.h>

#include "messages/msg/talon_out.hpp"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

/** @file
 * @brief Node controlling one Talon motor 
 * 
 * This node receives information published by the logic node,
 * then transforms the data received into movement by the motor
 * controlled by the Talon instance.  The topics that the node
 * subscribes to are as follows:
 * \li \b speed_topic
 * \li \b STOP
 * \li \b GO
 * 
 * The \b speed_topic topic is either \b drive_left_speed
 * or \b drive_right_speed as defined in the parameters set in
 * the launch file.  To read more about the logic node or the
 * launch file
 * \see logic_node.cpp
 * \see launch.py
 * 
 * The topics being published are as follows:
 * \li \b info_topic
 * 
 * This string has the general form talon_{motorNumber}_info and
 * is defined by the user in the launch file.  To read more about
 * the launch file,
 * \see launch.py
 * 
 * */


rclcpp::Node::SharedPtr nodeHandle;
//bool GO=false;
bool GO = true;

/** @brief STOP Callback
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of STOP.  This function
 * sets a boolean value GO to false, which prevents the
 * robot from moving.
 * @param empty
 * @return void
 * */
void stopCallback(std_msgs::msg::Empty::SharedPtr empty){
	RCLCPP_INFO(nodeHandle->get_logger(),"STOP");
	GO=false;
} 

/** @brief GO Callback
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of GO.  This function
 * sets a boolean value GO to true, which allows the
 * robot to drive.
 * @param empty
 * @return void
 * */
void goCallback(std_msgs::msg::Empty::SharedPtr empty){
	RCLCPP_INFO(nodeHandle->get_logger(),"GO");
	GO=true;
}

bool useVelocity=false;
int velocityMultiplier=0;
int testSpeed=0;
TalonSRX* talonSRX;

/** @brief Speed Callback Function
 * 
 * Callback function triggered when the node receives
 * a topic with the topic name of drive_left_speed or
 * drive_right_speed.  This function takes the data
 * from the topic and sets the motor to the speed
 * specified.
 * @param speed
 * @return void
 * */
void speedCallback(const std_msgs::msg::Float32::SharedPtr speed){
//	RCLCPP_INFO(nodeHandle->get_logger(),"---------->>> %f ", speed->data);
	//std::cout << "---------->>>  " << speed->data << std::endl;

	if(useVelocity){
        	talonSRX->Set(ControlMode::Velocity, int(speed->data*velocityMultiplier));
		//talonSRX->Set(ControlMode::Velocity, testSpeed);
	}else{
        	talonSRX->Set(ControlMode::PercentOutput, speed->data);
	}
}


int main(int argc,char** argv){
	rclcpp::init(argc,argv);
	nodeHandle = rclcpp::Node::make_shared("linear");

	RCLCPP_INFO(nodeHandle->get_logger(),"Starting talon");
	//int success;

	//int motorNumber=0;
	//success=nodeHandleP.getParam("motor_number", motorNumber);
	nodeHandle->declare_parameter<int>("motor_number",1);
	rclcpp::Parameter motorNumberParameter = nodeHandle->get_parameter("motor_number");
	int motorNumber = motorNumberParameter.as_int();
	std::cout << "motor_number: " << motorNumber << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"motorNumber: %d", motorNumber);
	
	nodeHandle->declare_parameter<int>("diagnostics_port",1);
	rclcpp::Parameter portNumberParameter = nodeHandle->get_parameter("diagnostics_port");
	int portNumber = portNumberParameter.as_int();
	std::cout << "diagnostics_port: " << portNumber <<std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(), "diagnosticsPort: %d", portNumber);
	//c_SetPhoenixDiagnosticsStartTime(-1); //Disables the Phoenix Diagnostics server, but does not allow the Talons to run
	c_Phoenix_Diagnostics_Create1(portNumber);  //Creates a Phoenix Diagnostics server with the port specified

	//std::string speedTopic;
	//success=nodeHandleP.getParam("speed_topic", speedTopic);
	nodeHandle->declare_parameter<std::string>("speed_topic","unset");
    rclcpp::Parameter speedTopicParameter = nodeHandle->get_parameter("speed_topic");
    std::string speedTopic = speedTopicParameter.as_string();
	std::cout << "speed_topic: " << speedTopic << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"speedTopic: %s",speedTopic.c_str());

	//bool invertMotor=false;
	//success=nodeHandleP.getParam("invert_motor", invertMotor);
	nodeHandle->declare_parameter<bool>("invert_motor",false);
    rclcpp::Parameter invertMotorParameter = nodeHandle->get_parameter("invert_motor");
    bool invertMotor = invertMotorParameter.as_bool();
	std::cout << "invert_motor: " << invertMotor << std::endl;
	RCLCPP_INFO(nodeHandle->get_logger(),"invertMotor: %d",invertMotor);

    ctre::phoenix::platform::can::SetCANInterface("can0");
	RCLCPP_INFO(nodeHandle->get_logger(),"Opened CAN interface");

	int kTimeoutMs=30;
	int kPIDLoopIdx=0;
	//int kSlotIdx=0;
	talonSRX=new TalonSRX(motorNumber);
	RCLCPP_INFO(nodeHandle->get_logger(),"created talon instance");

	talonSRX->SetInverted(invertMotor);

    talonSRX->Set(ControlMode::PercentOutput, 0);
    talonSRX->Set(ControlMode::Velocity, 0);

	RCLCPP_INFO(nodeHandle->get_logger(),"configured talon");

	//TalonSRXPIDSetConfiguration pid;
	TalonSRXConfiguration allConfigs;
	//StatusFrame statusFrame;

	messages::msg::TalonOut talonOut;
	//ros::Publisher talonOutPublisher=nodeHandle.advertise<messages::TalonOut>(infoTopic.c_str(),1);
	//ros::Subscriber speedSubscriber=nodeHandle.subscribe(speedTopic.c_str(),1,speedCallback);
	auto speedSubscriber=nodeHandle->create_subscription<std_msgs::msg::Float32>(speedTopic.c_str(),1,speedCallback);

	//ros::Subscriber stopSubscriber=nodeHandle.subscribe("STOP",1,stopCallback); 
	auto stopSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("STOP",1,stopCallback);
	//ros::Subscriber goSubscriber=nodeHandle.subscribe("GO",1,goCallback); 
	auto goSubscriber=nodeHandle->create_subscription<std_msgs::msg::Empty>("GO",1,goCallback);
	RCLCPP_INFO(nodeHandle->get_logger(),"set subscribers");

	while(rclcpp::ok()){
		if(GO)ctre::phoenix::unmanaged::FeedEnable(100);
        usleep(20);
		rclcpp::spin_some(nodeHandle);
    }
}


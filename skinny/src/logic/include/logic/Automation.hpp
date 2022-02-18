#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "AutomationTypes.hpp"

/** @class
 *
 *  @brief Header file for Automation
 * 
 * */

class Automation{
    public:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveLeftSpeedPublisher;
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32_<std::allocator<void> >, std::allocator<void> > > driveRightSpeedPublisher;

    rclcpp::Node::SharedPtr node;
    Position position;
    Quaternion orientationQuaternion;
    EulerAngles orientation;
    float currentLeftSpeed=0;
    float currentRightSpeed=0;


    virtual void automate() = 0;

    /** @brief Sets publisher node for left and right wheel motor speed
     * 
     * This function sets the driveLeftSpeedPublisher and driveRightSpeedPublisher.
     * @param rclcpp::Node::SharedPtr node
     * @return void
     * */
    void setNode(rclcpp::Node::SharedPtr node);

    /** @brief Sets axis orientations and calls EulerAngles.
     *
     * Sets position for x, y, z axes and w (homogenous vertex), and then 
     * calls toEulerAngles to convert the positions to Euler angles to orient 
     * the bot in 3d space.
     * @param Position
     * @return void
     * */
    void setPosition(Position position);

    /** @brief Assigns/publishes left/right motorspeeds
     * 
     * This function assigns the speed of the left and right motors and then publishes them.
     * @param left, right
     * @return void
     * */
    void changeSpeed(float left, float right);

    /** @brief Converts raw x,y,z-axis data to Euler angles to orient the bot in 3d space.
     *
     * This function takes in the x,y,z-axis coordinates established in setPosition and 
     * converts them to Euler angles. Euler angles describe the orientation of a body to a fixed coordinate 
     * system using each axes angle from its respective origin to the coordinate itself. 
     * This measurment of a coordinates angle from the origin in each individual axii indicates the 
     * rotation of the robot in the 3d reference system i.e, pitch, roll, and yaw.
     * @param q
     * @return angles
     * */
    EulerAngles toEulerAngles(Quaternion q); 

};

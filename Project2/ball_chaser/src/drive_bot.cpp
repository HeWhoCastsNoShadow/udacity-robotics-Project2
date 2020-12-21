#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBotService
{
public:
    DriveBotService()
    {
        // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
        motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        // Define a command_robot service with a handle_drive_request callback function
        drive_bot = n.advertiseService("/ball_chaser/command_robot", &DriveBotService::handle_drive_request, this);
    }

private:
    ros::Publisher motor_command_publisher;
    
    ros::ServiceServer drive_bot;
    
    ros::NodeHandle n;
    
    bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res);
};


bool DriveBotService::handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{
    ROS_INFO("ball_chaser::DriveToTarget received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);
    
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    
    // Set robot velocities
    motor_command.linear.x = (float)req.linear_x;
    motor_command.angular.z = (float)req.angular_z;
    
    // Publish velocities to drive the robot
    motor_command_publisher.publish(motor_command);
    
    // Return a response message
    res.msg_feedback = "Wheel velocities - linear_x: " + std::to_string(req.linear_x) + " , angular_z: " + std::to_string(req.angular_z);
    
    ROS_INFO_STREAM(res.msg_feedback);
    
    return true;
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");
    
    // Instantiate
    DriveBotService driveBotServiceObject;
    
    ROS_INFO("Ready to send command_robot commands");
    
    // Handle ROS communication events
    ros::spin();

    return 0;
}

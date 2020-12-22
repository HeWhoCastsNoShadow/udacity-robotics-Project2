#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImageClient
{
public:
    ProcessImageClient()
    {
        // Define a client service capable of requesting services from command_robot
        client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

        // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
       sub1 = n.subscribe("/camera/rgb/image_raw", 10, &ProcessImageClient::process_image_callback, this);
       
       ROS_INFO("process_image ready to detect images.");
    }
    
private:
    // Define a client that can request services
    ros::ServiceClient client;
    
    // ROS NodeHandle object
    ros::NodeHandle n;
    
    ros::Subscriber sub1;
    
    void drive_robot(float lin_x, float ang_z);
    
    void process_image_callback(const sensor_msgs::Image img);
};

// This function calls the command_robot service to drive the robot in the specified direction
void ProcessImageClient::drive_robot(float lin_x, float ang_z)
{
    ROS_INFO_STREAM("Moving the Robot with linear_x: " + std::to_string(lin_x) + " , angular_z: " + std::to_string(ang_z));
    
    ball_chaser::DriveToTarget srv;
    
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    // Call the ball_chaser service and pass the requested velocities
    if (!client.call(srv))
        ROS_ERROR("Failed to call service ball_chaser");
}

// This callback function continuously executes and reads the image data
void ProcessImageClient::process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    int normalized_width;
    int right_portion;
    int middle_portion;
    int idx = 0;
    unsigned char* rbg_pixel;
    bool white_ball_detected = false;
    
    right_portion = img.step/3;
    middle_portion = right_portion*2;
    
    for (int i = 0; i < img.height; i++)
    {
        // Pixel has 3 bytes so traverse pixels by 3 bytes
        for (int j = 0; j < img.step; j+=3)
        {
            // get index of the pixel
            idx = (i*img.step) + j;
            rbg_pixel = (unsigned char*)&img.data[idx];
            
            // Check if all RBG elements are white
            if ((rbg_pixel[0] == white_pixel) && (rbg_pixel[1] == white_pixel) && (rbg_pixel[2] == white_pixel))
            {
                white_ball_detected = true;
                
                // Find the side of the image
                if(j <= right_portion)
                {
                    drive_robot(0.0, 0.5);
                }
                else if (j <= middle_portion)
                {
                    drive_robot(0.5, 0.0);
                }
                else
                {
                    drive_robot(0.0, -0.5);
                }
                
                break;
            }
        }
    }
    
    // Request a stop when there's no white ball seen by the camera
    if (!white_ball_detected)
    {
        drive_robot(0.0, 0.0);
    }
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    
    ProcessImageClient processImageClientObject;

    // Handle ROS communication events
    ros::spin();

    return 0;
}

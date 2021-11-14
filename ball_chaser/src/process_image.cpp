#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <math.h>

class ImageProcessor {
    public:
        ImageProcessor(ros::NodeHandle* nh) {
            // class constructor
            
            // Define a client service capable of requesting services from command_robot
            client_ = nh->serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

            // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
            sub_ = nh->subscribe("/camera/rgb/image_raw", 10, &ImageProcessor::process_image_callback, this);
            nh_ = nh;

            //get color (int) from ROS parameters
            if(nh_->getParam("/process_image/ball_color", color_pixel_)) {
                ROS_INFO_STREAM(ros::this_node::getName() << ": using color ["<< color_pixel_ << "] of the ball from the parameters.");
            };

            ROS_INFO_STREAM(ros::this_node::getName() << "node: ready to process the camera images");
        }

        // This function calls the command_robot service to drive the robot in the specified direction
        void drive_robot(float lin_x, float ang_z)
        {
            // TODO: Request a service and pass the velocities to it to drive the robot
            
            srv_.request.linear_x = lin_x;
            srv_.request.angular_z = ang_z;
            if (!client_.call(srv_)) {
                ROS_INFO("Error: set wheels velocities: lin_X:%1.2f, ang_Z:%1.2f", lin_x, ang_z);
            }
        }

    // This callback function continuously executes and reads the image data
    void process_image_callback(const sensor_msgs::Image img)
    {

        // TODO: Loop through each pixel in the image and check if there's a bright white one
        // Then, identify if this pixel falls in the left, mid, or right side of the image
        // Depending on the white ball position, call the drive_bot function and pass velocities to it
        // Request a stop when there's no white ball seen by the camera

        int sub_width = img.width / 3.0; // divide the image upon three similar subwindows
        int pixel_length = img.step / img.width; // get color encoding bytes length

        int ball_min_xpos = img.width;
        int ball_max_xpos = 0;
        int ball_min_ypos = img.height;
        int ball_max_ypos = 0;
        //ROS_INFO("Image info: %d step, %d width", img.step, img.width);
        int absolute_pos = 0;
        for (int row = 0; row < img.height; ++row) 
            {
                int col_pos = 0;
                for (int col = 0; col < img.step; col +=pixel_length) 
                {
                    if (img.data[absolute_pos] == color_pixel_ && img.data[absolute_pos+1] == color_pixel_ && img.data[absolute_pos+2] == color_pixel_) {
                        // determine the ball borders
                        ball_min_xpos = std::min(ball_min_xpos, col_pos);
                        ball_max_xpos = std::max(ball_max_xpos, col_pos);
                        ball_min_ypos = std::min(ball_min_ypos, row);
                        ball_max_ypos = std::max(ball_max_ypos, row);
                        
                    }
                    col_pos++;
                    absolute_pos += pixel_length; 
                }
                
            }

        
        int ball_min_diameter = 1; // in px

        // estimate an approximal diameter of the ball
        int hor_length = ball_max_xpos - ball_min_xpos;
        int vert_lenght = ball_max_ypos - ball_min_ypos;

        int ballDiameter = std::min(vert_lenght, hor_length); // in px

        if (ballDiameter < ball_min_diameter) { 
        drive_robot(0.0, angles::from_degrees(30.0)); // loook around to search a ball
        return; // return if the ball is too small or not found
        }

        
        // estimate the ball's center
        int ballCenterX = ball_min_xpos + hor_length / 2.0;
        int ballCenterY = ball_min_ypos + vert_lenght / 2.0;

        /* estimate the driver's velocities */
        float maxLinXvelocity = 0.5; // m/s
        int cameraCenterX = img.width / 2.0;
        int cameraCenterY = img.height;

        // estimate linear velocities
        float linXvel = 0.1;
        float vel_rate = 1.0 - (ballDiameter / 2.0) / std::sqrt(std::pow(ballCenterX - cameraCenterX, 2.0) + std::pow(cameraCenterY - ballCenterY, 2.0));
        linXvel = std::max(linXvel, vel_rate * maxLinXvelocity );

        // estimate angular velocities
        
        tf::Vector3 ballVector(ballCenterX - cameraCenterX, cameraCenterY - ballCenterY, 0.0);
        ballVector.normalize();
        tf::Vector3 camVector(0.0, 1.0, 0.0);
        float angle = camVector.angle(ballVector); //estimate the angle between y-axis and the ball position vector (origin in camera center)
        
        tf::Vector3 xAxisVector(1.0, 0.0, 0.0);
        float ax_angle = xAxisVector.angle(ballVector); // estimate the angle between x-axis and the ball position vector (origin in camera center)
        float angle_dir = 1.0;
        if (ax_angle < angles::from_degrees(90)) { // the boll is located on the right side of the image
        angle_dir = -1.0;
        }

        float maxAngZvelocity = angles::from_degrees(5); // set max angular velocity in rad/s
        
        float currAngVel = angle * vel_rate * angle_dir; // accordingly to the ball distance
        
    
        // set new velocity values
        drive_robot(linXvel, currAngVel);
    }
        
    private:
        ros::ServiceClient client_;
        ball_chaser::DriveToTarget srv_;
        ros::Subscriber sub_;
        ros::NodeHandle* nh_;
        int color_pixel_ = 255;
};



int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    ImageProcessor iproc(&n);

    // Handle ROS communication events
    ros::spin();

    return 0;
}


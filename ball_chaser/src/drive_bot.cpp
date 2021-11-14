#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBot {
    public:
        // define the class constructor
        DriveBot(ros::NodeHandle* nh) {

            // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
            motor_command_publisher_ = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 10);

            // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
            service_ = nh->advertiseService("/ball_chaser/command_robot", &DriveBot::handle_drive_request, this);

            ROS_INFO_STREAM(ros::this_node::getName() << "node: ready to send the wheels joints commands");


        }

        // This function should publish the requested linear x and angular velocities to the robot wheel joints
        // After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
        bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
        {

            //ROS_INFO("DriveToTarget/joint_velocities received: lin_X:%1.2f, ang_Z:%1.2f", (float)req.linear_x, (float)req.angular_z);
            if (req.linear_x == prev_linx_ && req.angular_z ==prev_angz_) {return true;}

            // Form a request message
            geometry_msgs::Twist msg;
            msg.linear.x = req.linear_x;
            msg.linear.y = 0.0;
            msg.linear.z = 0.0;
            msg.angular.x = 0.0;
            msg.angular.y = 0.0;
            msg.angular.z = req.angular_z;
            
            // Send the request message
            motor_command_publisher_.publish(msg);

            // Return a response message
            res.msg_feedback = "Wheels joints velocities set: linear x " + std::to_string(msg.linear.x) + ", angular z " + std::to_string(msg.angular.z);
            ROS_INFO_STREAM(res.msg_feedback);

            prev_linx_ = req.linear_x;
            prev_angz_ = req.angular_z;
            
            return true;
        }

    private:
        // ROS::Publisher motor commands;
        ros::Publisher motor_command_publisher_;
        ros::ServiceServer service_;
        float prev_linx_ = 0.0;
        float prev_angz_ = 0.0;
};





int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    DriveBot dbot(&n);
    
    // TODO: Handle ROS communication events
    ros::spin();

    return 0;
}

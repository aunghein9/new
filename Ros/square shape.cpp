#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>

class TurtleSquare {
public:
    TurtleSquare() : nh_("~"), state_(0), side_length_(2.0), duration_(2.0) {
        
        nh_.param("side_length", side_length_, side_length_);
        nh_.param("duration", duration_, duration_);
        
        
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        pose_sub_ = nh_.subscribe("/turtle1/pose", 10, &TurtleSquare::poseCallback, this);
        
        
        timer_ = nh_.createTimer(ros::Duration(duration_), &TurtleSquare::timerCallback, this);
        
        ROS_INFO("Starting square movement with side length: %.2f", side_length_);
    }

    void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
        current_pose_ = *msg;
    }

    void timerCallback(const ros::TimerEvent&) {
        geometry_msgs::Twist cmd;
        
        switch(state_ % 4) {
            case 0: 
                cmd.linear.x = side_length_ / duration_;
                cmd.angular.z = 0;
                ROS_INFO("Moving forward");
                break;
            case 1: 
                cmd.linear.x = 0;
                cmd.angular.z = M_PI/2 / duration_; 
                ROS_INFO("Turning 90 degrees");
                break;
            case 2: 
                cmd.linear.x = side_length_ / duration_;
                cmd.angular.z = 0;
                ROS_INFO("Moving forward");
                break;
            case 3: 
                cmd.linear.x = 0;
                cmd.angular.z = M_PI/2 / duration_;
                ROS_INFO("Turning 90 degrees");
                break;
        }
        
        cmd_vel_pub_.publish(cmd);
        state_++;
        
        if(state_ >= 8) { 
            ROS_INFO("Square movement complete!");
            ros::shutdown();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber pose_sub_;
    ros::Timer timer_;
    
    turtlesim::Pose current_pose_;
    int state_;
    double side_length_;
    double duration_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_square");
    TurtleSquare ts;
    ros::spin();
    return 0;
}
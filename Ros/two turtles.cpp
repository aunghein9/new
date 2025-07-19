#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>
#include <deque>
#include <cmath>

class TurtleFollower {
public:
    TurtleFollower() : nh_("~"), history_size_(50), delay_(5.0) {
    
        nh_.param("delay", delay_, delay_);
        
        
        ros::service::waitForService("/spawn");
        ros::service::waitForService("/clear");
        
        
        spawnSecondTurtle();
        
        
        clearTurtleDrawings();
        

        cmd_vel_pub1_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        cmd_vel_pub2_ = nh_.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
        pose_sub1_ = nh_.subscribe("/turtle1/pose", 10, &TurtleFollower::pose1Callback, this);
        pose_sub2_ = nh_.subscribe("/turtle2/pose", 10, &TurtleFollower::pose2Callback, this);
        
        
        follow_timer_ = nh_.createTimer(ros::Duration(0.1), &TurtleFollower::followTimerCallback, this);
        
        
        move_timer_ = nh_.createTimer(ros::Duration(0.1), &TurtleFollower::moveTimerCallback, this);
        
        ROS_INFO("Turtle2 will follow Turtle1's path with %.1f second delay", delay_);
    }

    void spawnSecondTurtle() {
        turtlesim::Spawn srv;
        srv.request.x = 5.0;
        srv.request.y = 5.0;
        srv.request.theta = 0.0;
        srv.request.name = "turtle2";
        
        ros::service::call("/spawn", srv);
    }

    void clearTurtleDrawings() {
        std_srvs::Empty srv;
        ros::service::call("/clear", srv);
    }

    void pose1Callback(const turtlesim::Pose::ConstPtr& msg) {
    
        PoseStamped ps;
        ps.pose = *msg;
        ps.stamp = ros::Time::now();
        
        pose_history1_.push_back(ps);
        if (pose_history1_.size() > history_size_) {
            pose_history1_.pop_front();
        }
    }

    void pose2Callback(const turtlesim::Pose::ConstPtr& msg) {
        current_pose2_ = *msg;
    }

    void moveTimerCallback(const ros::TimerEvent&) {
    
        static int state = 0;
        static ros::Time last_state_change = ros::Time::now();
        
        geometry_msgs::Twist cmd;
        double state_duration = 2.0; 
        
        if ((ros::Time::now() - last_state_change).toSec() > state_duration) {
            state = (state + 1) % 4;
            last_state_change = ros::Time::now();
        }
        
        switch(state) {
            case 0: 
                cmd.linear.x = 1.0;
                cmd.angular.z = 0.0;
                break;
            case 1: 
                cmd.linear.x = 0.0;
                cmd.angular.z = M_PI/4; 
                break;
            case 2: 
                cmd.linear.x = 1.0;
                cmd.angular.z = 0.0;
                break;
            case 3: 
                cmd.linear.x = 0.0;
                cmd.angular.z = M_PI/4;
                break;
        }
        
        cmd_vel_pub1_.publish(cmd);
    }

    void followTimerCallback(const ros::TimerEvent&) {
        if (pose_history1_.empty()) return;
        
        
        ros::Time target_time = ros::Time::now() - ros::Duration(delay_);
        turtlesim::Pose target_pose;
        bool found = false;
        
        for (auto it = pose_history1_.rbegin(); it != pose_history1_.rend(); ++it) {
            if (it->stamp <= target_time) {

                target_pose = it->pose;
                found = true;
                break;
            }
        }
        
        if (!found) {
            target_pose = pose_history1_.front().pose;
        }
        
        
        geometry_msgs::Twist cmd;
        double distance = sqrt(pow(target_pose.x - current_pose2_.x, 2) + 
                             pow(target_pose.y - current_pose2_.y, 2));
        
        if (distance > 0.1) { 
            double angle_to_target = atan2(target_pose.y - current_pose2_.y,
                                         target_pose.x - current_pose2_.x);
            double angle_diff = angle_to_target - current_pose2_.theta;
            
            
            angle_diff = atan2(sin(angle_diff), cos(angle_diff));
            
            
            cmd.linear.x = std::min(0.5 * distance, 2.0);
            cmd.angular.z = 4.0 * angle_diff;
        }
        
        cmd_vel_pub2_.publish(cmd);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub1_, cmd_vel_pub2_;
    ros::Subscriber pose_sub1_, pose_sub2_;
    ros::Timer follow_timer_, move_timer_;
    
    struct PoseStamped {
        turtlesim::Pose pose;
        ros::Time stamp;
    };
    
    std::deque<PoseStamped> pose_history1_;
    turtlesim::Pose current_pose2_;
    size_t history_size_;
    double delay_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtle_follower");
    TurtleFollower tf;
    ros::spin();
    return 0;
}
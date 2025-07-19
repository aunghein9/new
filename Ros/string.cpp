#include "underwater_control/UnderwaterStatus.h"
#include <ros/ros.h>
#include <string>
#include <random>

class SensorPublisher {
public:
    SensorPublisher() : nh_("~") {
        pub_ = nh_.advertise<underwater_control::UnderwaterStatus>("sensor_data", 10);
        
        
        gen_.seed(std::random_device()());
        depth_dist_ = std::uniform_real_distribution<>(0.0, 100.0);
        temp_dist_ = std::uniform_real_distribution<>(5.0, 30.0);
        
        timer_ = nh_.createTimer(ros::Duration(1.0), 
                               &SensorPublisher::publishData, 
                               this);
    }

    void publishData(const ros::TimerEvent&) {
        underwater_control::UnderwaterStatus msg;
        
        
        msg.depth = depth_dist_(gen_);
        msg.temperature = temp_dist_(gen_);
        
        
        if (msg.depth > 80.0) {
            msg.status_message = "CRITICAL: Deep water operation";
        } else if (msg.temperature > 25.0) {
            msg.status_message = "WARNING: High temperature";
        } else {
            msg.status_message = "NORMAL: Operating within parameters";
        }
        
        pub_.publish(msg);
        ROS_INFO("Published: Depth=%.2fm, Temp=%.2f°C, Status=%s", 
                msg.depth, msg.temperature, msg.status_message.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Timer timer_;
    
    std::mt19937 gen_;
    std::uniform_real_distribution<> depth_dist_;
    std::uniform_real_distribution<> temp_dist_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_publisher");
    SensorPublisher node;
    ros::spin();
    return 0;
}
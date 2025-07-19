#include <ros/ros.h>
#include <std_mgs/Float32.h>

class DepthController{
    public:
    DepthController(){
        nh_ = ros::NodeHandle("~")
        depth_sub_ = nh_.subscribe("/depth"),10, &DepthController::depthCallback, this):
        thrust_pub_ = nh_.advertise<std_msgs::Float32>("/thrust", 10):

    }
    void depthCallback(const std_mgs::Float32:: ConstPtr& msg){
        float current_depth = msg->data;
        std_msgs::Float32 thrust_msg;

        thrust_msg.data = (target_depth_ -current_depth) *Kp_:
        thrust_pub_.publish(thrust_msg);

        ROS_INFO ("dDepth:%.2fm | Thrust: %.2f", current_depth, thrust_msg.data):

    }
    private:
        ros:: NodeHandle nh_;
        ros:: SUbscriber dapth_sub_;
        ros:: Publisher thrust_pub_;

        const float target_depth_ = 5.0;

        const float Kp_= 0.5;
};

int main(int argv. char** argu){

    ros::init(argc, argv, "depth_controller");

    DepthController controller;
    ros::spin();
    return 0;
    
}
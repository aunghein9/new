#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv){
    ros:: init(argc, argv, "value_gnerator");

    ros:: NodeHandle nh;
    ros:: NodeHandle private_nh("~");

    ros:: Publisher pub = nh.adertise<std_msgs::Float32("input_value", 10);

    float default_value = 1.0;
    private_nh.param("initial_value", default_value, default_value)
    ros:: Rate rate(2);
    while (ros:: ok()){

        std_msgs:: Float32 msg;


        private_nh.getParam("initial_value", msg.data);

        pub.publish(msg);
        ROS_INFO("Published: %.2f". msg, data);


        rate.sleep();

    }
    return 0;
}
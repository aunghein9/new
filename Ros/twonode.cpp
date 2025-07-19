#include "underwater_control/RawData.h"
#include "underwater_control/ProcessData.h"
#include <ros/ros.h>

class SensorNode {
public:
    SensorNode() : nh_("~") {

        raw_data_server_ = nh_.advertiseService(


            "provide_raw_data", 
            &SensorNode::handleRawDataRequest, 
            this
        );

    
        process_client_ = nh_.serviceClient<underwater_control::ProcessData>("process_sensor_data");
        
        timer_ = nh_.createTimer(ros::Duration(1.0),  &SensorNode::timerCallback, this);
                               
                              
    }

    bool handleRawDataRequest(
        underwater_control::RawData::Request &req,
        underwater_control::RawData::Response &res
    ) {
    

        res.depth = 10.0 + (rand() % 100) / 10.0;
        res.temperature = 15.0 + (rand() % 50) / 10.0;
        ROS_INFO("Provided raw data: Depth=%.1fm, Temp=%.1fC", 
                res.depth, res.temperature);
        return true;
    }

    void timerCallback(const ros::TimerEvent&) {


        underwater_control::ProcessData srv;
        if (raw_data_server_.exists()) {
            if (raw_data_server_.call(srv)) {

                ROS_INFO("Processing result: Quality=%.2f, Success=%d",
                        srv.response.quality_score, 
                        srv.response.success);
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer raw_data_server_;
    ros::ServiceClient process_client_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_node");
    SensorNode node;

    ros::spin();
    return 0;
}
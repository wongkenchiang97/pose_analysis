#include <pose_analysis/distance_data_concatenate.h>

#include <ros/ros.h>

int main(int argc,char** argv){
    ros::init(argc,argv,"distance_data_concatenate");

    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~");
    ROS_INFO("starting distance data concatenate node.");

    pose_analysis::DistanceDataConcatenatePtr distance_concatenator;
    distance_concatenator = std::make_shared<pose_analysis::DistanceDataConcatenate>(nh,pnh,argc,argv);

    return 0;
}
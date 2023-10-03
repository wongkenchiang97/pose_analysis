#include <pose_analysis/test_data_concatenate.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "analysis_data_concatenator");

    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~");
    ROS_INFO("starting test data concatenate node");

    pose_analysis::TestDataConcatenatePtr test_concatenator;
    test_concatenator = std::make_shared<pose_analysis::TestDataConcatenate>(nh, pnh, argc, argv);

    // ros::spinOnce();
    // ros::AsyncSpinner spinner(0);
    // spinner.start();
    // ros::waitForShutdown();

    return 0;
}
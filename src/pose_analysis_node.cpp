#include <pose_analysis/pose_analysis.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_analysis");

    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~");

    PoseAnalysisPtr pose_analysis = std::make_shared<PoseAnalysis>(nh, pnh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
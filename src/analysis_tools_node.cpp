#include <pose_analysis/analysis_tools.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"analysis_tools");
    ros::NodeHandlePtr nh = boost::make_shared<ros::NodeHandle>();
    ros::NodeHandlePtr pnh = boost::make_shared<ros::NodeHandle>("~");
    ROS_INFO("starting analysis tools.");

    pose_analysis::AnalysisToolsPtr tools = std::make_shared<pose_analysis::AnalysisTools>(nh,pnh);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
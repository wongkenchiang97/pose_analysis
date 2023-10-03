#ifndef POSE_ANALYSIS_H
#define POSE_ANALYSIS_H

#include <curses.h>
#include <deque>
#include <fstream>
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include <thread>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>

#include <Eigen/Eigen>

#include <pose_analysis/pose_data.h>

class PoseAnalysis {
private:
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr pnh_;
    ros::Subscriber pose_sub_;
    ros::Publisher error_pub_;
    ros::Publisher show_error_flag_pub_;
    std::string pose_topic_name_;
    double rack_distance_;
    std::deque<geometry_msgs::PoseStamped> pose_buffer_;
    std::vector<analysis::PoseData> pose_data_;
    std::vector<Eigen::Vector3d> pose_error_;
    std::vector<std::pair<double, double>> time_consumed_;
    std::vector<long double> time_data_;
    std::thread pose_extract_thread_;
    std::thread keyboard_input_thread_;
    std::mutex pose_data_mt_;
    Eigen::Matrix4d experiment_gt_;
    Eigen::Matrix4d measurement_to_gt_tf_;

    void poseCallback(const geometry_msgs::PoseStampedConstPtr& _msgs);
    void extractProcess();
    void keyPressProcess();
    void extractBufferPose();
    void writePoseError();
    void publishPoseError(const ros::Time& _stamp, const geometry_msgs::Point& _pt, const geometry_msgs::Quaternion& _q);
    void getTimeConsumed();
    void transformGroundtruthToNCUFrame();
    double getVectorStandardDeviation(const Eigen::VectorXd& _vec);
    Eigen::Matrix<double, -1, 3, Eigen::RowMajor> getPoseError();
    Eigen::Vector3d getPoseError(const geometry_msgs::PoseStamped& _est_pose);
    template <class T>
    Eigen::Matrix<std::string, -1, -1, 1> convertMatrixToStringMatrix(const Eigen::Matrix<T, -1, -1>& _input_mat, int row_size, int col_size);

public:
    PoseAnalysis(ros::NodeHandlePtr& _nh, ros::NodeHandlePtr& _pnh);
    ~PoseAnalysis();
    double quatToYaw(const geometry_msgs::Quaternion& _orientation);
    double getRelativeYawAngle(const geometry_msgs::Quaternion& _orientation);
};

PoseAnalysis::PoseAnalysis(ros::NodeHandlePtr& _nh, ros::NodeHandlePtr& _pnh)
{
    nh_ = _nh;
    pnh_ = _pnh;
    pnh_->param<std::string>("/pose_topic_name", pose_topic_name_, "/pose");
    pnh_->param<double>("rack_distance", rack_distance_, 0.0);
    std::cout << "rack_distance: " << std::to_string(rack_distance_) << std::endl;
    // std::cout << "pose_topic_name_: " << pose_topic_name_ << std::endl;
    pose_sub_ = nh_->subscribe(pose_topic_name_, 1000, &PoseAnalysis::poseCallback, this);
    error_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>("/pose_error", 10);
    show_error_flag_pub_ = nh_->advertise<std_msgs::Bool>("/show_error_flag", 3, true);
    std::vector<double> tmp_T(16);
    if (pnh_->getParam("/experiment_gt/T", tmp_T)) {
        experiment_gt_ = Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(tmp_T.data(), 4, 4);
        // std::cout<<"experiment_gt:\n"<<experiment_gt_<<std::endl;
    }
    // transformGroundtruthToNCUFrame();
    std::cout << "gt:\n"
              << experiment_gt_ << std::endl;
    pose_extract_thread_ = std::thread(&PoseAnalysis::extractProcess, this);
    keyboard_input_thread_ = std::thread(&PoseAnalysis::keyPressProcess, this);
}

PoseAnalysis::~PoseAnalysis()
{
    pose_extract_thread_.join();
    keyboard_input_thread_.join();
    // writePoseError();
}

void PoseAnalysis::transformGroundtruthToNCUFrame()
{
    Eigen::Matrix4d NCU_tf = Eigen::Matrix4d::Identity();
    NCU_tf.topLeftCorner(3, 3) = (Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())).toRotationMatrix();
    experiment_gt_ = NCU_tf * experiment_gt_;
}

double PoseAnalysis::quatToYaw(const geometry_msgs::Quaternion& _orientation)
{
    Eigen::Quaterniond tmp_q;
    tf::quaternionMsgToEigen(_orientation, tmp_q);
    Eigen::Vector3d planar_normalized_x_vec;
    planar_normalized_x_vec << tmp_q.toRotationMatrix().block<2, 1>(0, 0), 0.0;
    planar_normalized_x_vec.normalize();
    double yaw = std::acos(planar_normalized_x_vec.dot(Eigen::Vector3d::UnitX()));
    return yaw;
}

double PoseAnalysis::getRelativeYawAngle(const geometry_msgs::Quaternion& _orientation)
{
    Eigen::Quaterniond current_q;
    tf::quaternionMsgToEigen(_orientation, current_q);
    Eigen::Matrix3d rel_rot = current_q * experiment_gt_.topLeftCorner(3, 3).inverse();
    Eigen::Vector3d planar_x_vec;
    planar_x_vec << rel_rot.topLeftCorner(2, 1), 0.0;
    double yaw = (180.0 / M_PI) * std::acos(planar_x_vec(0));
    if (planar_x_vec(1) < 0) {
        yaw *= -1.0;
    }
    return yaw;
}

void PoseAnalysis::extractBufferPose()
{
    std::lock_guard<std::mutex> guard(pose_data_mt_);
    while (!pose_buffer_.empty()) {
        // ROS_INFO("Buffer size: %ld", pose_buffer_.size());
        double yaw = getRelativeYawAngle(pose_buffer_.front().pose.orientation);
        // ROS_INFO("[x,y,yaw] = [%f,%f,%f]", pose_buffer_.front().pose.position.x, pose_buffer_.front().pose.position.y, yaw);
        pose_error_.push_back(getPoseError(pose_buffer_.front()));
        publishPoseError(pose_buffer_.front().header.stamp, pose_buffer_.front().pose.position, pose_buffer_.front().pose.orientation);
        pose_data_.emplace_back(pose_buffer_.front().pose.position.x, pose_buffer_.front().pose.position.y, yaw);
        time_data_.emplace_back(pose_buffer_.front().header.stamp.sec
            + pose_buffer_.front().header.stamp.nsec * 1e-9);
        pose_buffer_.pop_front();
    }
}

Eigen::Matrix<double, -1, 3, Eigen::RowMajor> PoseAnalysis::getPoseError()
{
    Eigen::MatrixXd pose_error;
    Eigen::Map<Eigen::Matrix<double, -1, 3, Eigen::RowMajor>> pose_map(&(pose_data_.front().x), pose_data_.size(), 3);
    Eigen::Vector2d planar_gt = { experiment_gt_(0, 3), experiment_gt_(2, 3) };
    pose_error = pose_map.leftCols(2) - planar_gt.transpose().replicate(pose_data_.size(), 1);
    pose_error.conservativeResize(Eigen::NoChange, pose_error.cols() + 1);
    pose_error.col(pose_error.cols() - 1) = pose_map.rightCols(1);
    // std::cout<<"pose_error:\n"<<pose_error<<std::endl;
    return pose_error;
}

Eigen::Vector3d PoseAnalysis::getPoseError(const geometry_msgs::PoseStamped& _est_pose)
{
    Eigen::Vector3d output;
    Eigen::Matrix4d rel_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d planar_rel_pose = Eigen::Matrix4d::Identity();
    Eigen::Affine3d est_pose = Eigen::Affine3d::Identity();
    tf::poseMsgToEigen(_est_pose.pose, est_pose);
    // std::cout<<"est_pose(transformed):\n"<<measurement_to_gt_tf_*est_pose.matrix()<<"\n";
    rel_pose = experiment_gt_.inverse() * est_pose.matrix();
    planar_rel_pose.topLeftCorner(2, 2) = rel_pose.topLeftCorner(2, 2);
    planar_rel_pose.topLeftCorner(3, 3).colwise().normalize();
    planar_rel_pose.topRightCorner(2, 1) = rel_pose.topRightCorner(2, 1);

    double yaw = std::acos(planar_rel_pose.block<3, 1>(0, 0).dot(Eigen::Vector3d::UnitX()));
    if (planar_rel_pose(1, 0) < 0) {
        yaw *= -1.0;
    }

    Eigen::Vector3d euler_xyz = planar_rel_pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
    // std::cout<<"euler_xyz :"<<euler_xyz.transpose()<<"\n"<<std::endl;
    output << planar_rel_pose.topRightCorner(2, 1), 180 / M_PI * yaw;
    // std::cout << "output: " << output.transpose() << std::endl;

    return output;
}

void PoseAnalysis::writePoseError()
{
    std::lock_guard<std::mutex> guard(pose_data_mt_);

    assert(!time_data_.empty() || !pose_data_.empty());

    // write time series pose error
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
    std::string node_name = ros::this_node::getName();
    node_name.erase(0, 1);
    std::string time_series_csv_dir = ros::package::getPath(node_name);
    time_series_csv_dir.append("/data/analysis/time_series_pose_error_data.csv");
    std::ofstream file_time(time_series_csv_dir);
    if (!file_time.is_open()) {
        throw std::runtime_error("writePoseError(): failed to open pose csv file.");
    }
    std::cout << "writing pose data to : " << time_series_csv_dir << std::endl;
    std::vector<std::string> field_name = { "x_error(m)", "y_error(m)", "yaw_error(°)" };
    Eigen::Matrix<std::string, -1, -1, 1> header_time;
    header_time.conservativeResize(1, 4);
    header_time << "duration", field_name[0], field_name[1], field_name[2];
    Eigen::Map<Eigen::Matrix<long double, -1, -1>> time_map(time_data_.data(), time_data_.size(), 1);
    time_map -= Eigen::Matrix<long double, -1, -1>::Constant(time_data_.size(), 1, time_map(0));
    auto str_time_vec = convertMatrixToStringMatrix<long double>(time_map, time_data_.size(), 1);
    Eigen::Matrix<std::string, -1, -1> combined_time_data;
    combined_time_data.resize(pose_data_.size() + 1, 4);
    combined_time_data.topLeftCorner(1, 4) = header_time;
    combined_time_data.bottomLeftCorner(str_time_vec.rows(), str_time_vec.cols()) = str_time_vec;
    // combined_time_data.bottomRightCorner(pose_data_.size(),3) = convertMatrixToStringMatrix<double>(getPoseError(),pose_data_.size(),3);
    combined_time_data.bottomRightCorner(pose_data_.size(), 3) = convertMatrixToStringMatrix<double>(Eigen::Map<Eigen::Matrix3Xd>(pose_error_.data()->data(), 3, pose_error_.size()).transpose(), pose_error_.size(), 3);
    file_time << combined_time_data.format(CSVFormat);
    file_time.close();

    // write stdev data
    std::string stats_csv_dir = ros::package::getPath(node_name);
    stats_csv_dir.append("/data/analysis/stats_data.csv");
    std::ofstream file_stats(stats_csv_dir);
    auto pose_error_map = Eigen::Map<Eigen::Matrix3Xd>(pose_error_.data()->data(), 3, pose_error_.size());
    if (pose_error_map.cols() < 2) {
        throw std::runtime_error("empty data to estimate data standard deviation.");
    }
    double stdev_x = 100 * getVectorStandardDeviation(pose_error_map.row(0));
    double stdev_y = 100 * getVectorStandardDeviation(pose_error_map.row(1));
    double stdev_yaw = getVectorStandardDeviation(pose_error_map.row(2));
    Eigen::MatrixXd stdev_mat;
    Eigen::Matrix<std::string, -1, -1, 1> header_stats;
    Eigen::Matrix<std::string, -1, -1, 1> combine_matrix;
    Eigen::Matrix<std::string, -1, -1, 1> distance_vec;
    Eigen::Matrix<std::string, -1, -1, 1> str_stdev_mat;
    stdev_mat.resize(1, 3);
    stdev_mat.row(0) << stdev_x, stdev_y, stdev_yaw;
    field_name = { "σ_x(cm)", "σ_y(cm)", "σ_yaw(°)" };
    header_stats.conservativeResize(1, 4);
    str_stdev_mat = convertMatrixToStringMatrix(stdev_mat, 1, 3);
    distance_vec.resize(1, 1);
    combine_matrix.resize(distance_vec.rows() + 1, str_stdev_mat.cols() + 1);
    header_stats << "distance(cm)", field_name[0], field_name[1], field_name[2];
    distance_vec << std::to_string(rack_distance_);
    combine_matrix.topLeftCorner(header_stats.rows(), header_stats.cols()) = header_stats;
    combine_matrix.bottomLeftCorner(distance_vec.rows(), distance_vec.cols()) = distance_vec;
    combine_matrix.bottomRightCorner(str_stdev_mat.rows(), str_stdev_mat.cols()) = str_stdev_mat;
    if (!file_stats.is_open()) {
        throw std::runtime_error("writePoseError(): failed to open stats csv file.");
    }
    std::cout << "writing stats data to : " << stats_csv_dir << std::endl;
    std::cout << "combine_matrix:\n"
              << combine_matrix << std::endl;
    file_stats << combine_matrix.format(CSVFormat);
    file_stats.close();

    std_msgs::Bool show_error_flag;
    show_error_flag.data = true;
    show_error_flag_pub_.publish(show_error_flag);
}

double PoseAnalysis::getVectorStandardDeviation(const Eigen::VectorXd& _vec)
{
    double stddev = std::sqrt((_vec.array() - _vec.mean()).square().sum() / (_vec.size() - 1));
    return stddev;
}

void PoseAnalysis::publishPoseError(const ros::Time& _stamp, const geometry_msgs::Point& _pt, const geometry_msgs::Quaternion& _q)
{
    geometry_msgs::Vector3Stamped output_error;
    output_error.header.stamp = _stamp;
    output_error.vector.x = _pt.x - experiment_gt_(0, 3);
    output_error.vector.y = _pt.y - experiment_gt_(2, 3);
    output_error.vector.z = getRelativeYawAngle(_q);
    error_pub_.publish(output_error);
}

void PoseAnalysis::getTimeConsumed()
{
}

template <class T>
Eigen::Matrix<std::string, -1, -1, 1> PoseAnalysis::convertMatrixToStringMatrix(const Eigen::Matrix<T, -1, -1>& _input_mat, int _row_size, int _col_size)
{
    Eigen::Matrix<std::string, -1, -1, 1> str_mat;
    if (_row_size * _col_size != _input_mat.size()) {
        ROS_ERROR("incompatible matrix dimension.");
        return str_mat;
    }
    str_mat.resize(_row_size, _col_size);
    std::stringstream mat_ss;
    std::string row_str;
    std::string entity_str;
    int index = 0;
    mat_ss << _input_mat;
    while (std::getline(mat_ss, row_str, '\n')) {
        std::stringstream row_stream(row_str);
        while (std::getline(row_stream, entity_str, ' ')) {
            if (entity_str.empty()) {
                continue;
            }
            str_mat(index) = entity_str;
            index++;
        }
    }
    return str_mat;
}

/*---loops----*/

void PoseAnalysis::poseCallback(const geometry_msgs::PoseStampedConstPtr& _msgs)
{
    pose_buffer_.emplace_back(*_msgs);
}

void PoseAnalysis::extractProcess()
{
    auto nxt = std::chrono::steady_clock::now();
    while (ros::ok()) {
        nxt += std::chrono::milliseconds(1000);
        extractBufferPose();
        std::this_thread::sleep_until(nxt);
    }
}

void PoseAnalysis::keyPressProcess()
{
    int key_press;
    initscr();
    printw("press enter key to save pose data.\n");
    noecho();
    cbreak();
    while (ros::ok()) {
        key_press = getch();
        if (key_press == 10) {
            clrtoeol();
            refresh();
            endwin();
            writePoseError();
        }
    }
    clrtoeol();
    refresh();
    endwin();
}

typedef std::shared_ptr<PoseAnalysis> PoseAnalysisPtr;

#endif
#ifndef DISTANCE_DATA_CONCATENATE_H
#define DISTANCE_DATA_CONCATENATE_H

#include <fstream>

#include <ros/package.h>
#include <ros/ros.h>

#include <Eigen/Eigen>

namespace pose_analysis {

using MatrixXs = Eigen::Matrix<std::string, -1, -1>;
using MatrixRowXs = Eigen::Matrix<std::string, -1, -1, Eigen::RowMajor>;
using MatrixRowXd = Eigen::Matrix<double, -1, -1, Eigen::RowMajor>;

using MatrixXsRefConst = Eigen::Map<const MatrixXs>;
using MatrixRowXsRefConst = Eigen::Map<const MatrixRowXs>;
using MatrixRowXdRefConst = Eigen::Map<const MatrixRowXd>;

using MatrixXsRefConstPtr = std::shared_ptr<MatrixXsRefConst>;
using MatrixRowXsRefConstPtr = std::shared_ptr<MatrixRowXsRefConst>;
using MatrixRowXdRefConstPtr = std::shared_ptr<MatrixRowXdRefConst>;

class DistanceDataConcatenate {
private:
    ros::NodeHandlePtr nh_;
    ros::NodeHandlePtr pnh_;
    std::vector<std::string> allArgs_;
    std::vector<std::string> distance_elements_;
    MatrixRowXs time_file_header_;
    MatrixRowXs combined_str_csv_data_;
    Eigen::MatrixXd pose_error_stddev_;
    double rack_distance_;
    MatrixXsRefConstPtr dir_map_;
    MatrixXs distance_vector_;

    void getDirectories(int _argc, char**& _argv);
    void concatenateData();
    void writeCombinedData();
    double getVectorStandardDeviation(const Eigen::VectorXd& _vec);
    template <class T>
    Eigen::Matrix<std::string, -1, -1, 1> convertMatrixToStringMatrix(const Eigen::Matrix<T, -1, -1>& _input_mat, int _row_size, int _col_size);
    Eigen::MatrixXd convertStringMatrixToMatrixXd(const MatrixRowXs& _input_mat, int _row_size, int _col_size);

public:
    DistanceDataConcatenate(ros::NodeHandlePtr& _nh, ros::NodeHandlePtr& _pnh, int _argc, char**& _argv);
    ~DistanceDataConcatenate();
};

DistanceDataConcatenate::DistanceDataConcatenate(ros::NodeHandlePtr& _nh, ros::NodeHandlePtr& _pnh, int _argc, char**& _argv)
{
    nh_ = _nh;
    pnh_ = _pnh;
    getDirectories(_argc, _argv);
    std::cout << "dir_map_:\n"
              << *dir_map_ << std::endl;
    concatenateData();
    writeCombinedData();
}

DistanceDataConcatenate::~DistanceDataConcatenate() { }

void DistanceDataConcatenate::getDirectories(int _argc, char**& _argv)
{
    allArgs_ = std::vector<std::string>(_argv, _argv + _argc);
    dir_map_ = std::make_shared<MatrixXsRefConst>(allArgs_.data() + 1, _argc - 1, 1);
}

void DistanceDataConcatenate::concatenateData()
{
    pose_error_stddev_.resize(dir_map_->rows(), 3);
    for (int i = 0; i < dir_map_->rows(); i++) {
        std::vector<std::string> str_csv_data;
        std::vector<double> double_csv_data;
        std::ifstream file_time((*dir_map_)(i, 0));
        std::string row_str;
        std::string entity_str;
        int row = 1;
        size_t cols;
        while (std::getline(file_time, row_str, '\n')) {
            std::stringstream row_stream(row_str);
            while (std::getline(row_stream, entity_str, ',')) {
                // if(entity_str.empty()){
                //     continue;
                // }
                str_csv_data.push_back(entity_str);
            }
            row++;
        }
        row--;
        cols = str_csv_data.size() / row;

        // string matrix concatenation
        MatrixRowXsRefConst current_file_str_matrix_map(str_csv_data.data(), row, cols);
        if (i == 0) {
            combined_str_csv_data_.conservativeResize(row, cols);
            combined_str_csv_data_ << current_file_str_matrix_map;
            time_file_header_ = current_file_str_matrix_map.topRows(1);
            std::cout << "[rows,cols][" << i << "]: " << row << ", " << cols << std::endl;
            std::cout << "data[" << i << "]:\n"
                      << current_file_str_matrix_map << std::endl;
            std::cout << "combined_str_csv_data_:\n"
                      << combined_str_csv_data_.topRows(5) << std::endl;

        } else {
            std::cout << "i>0" << std::endl;
            //check column consistancy
            if (cols != combined_str_csv_data_.cols()) {
                throw std::runtime_error("incompatible column between files.");
            }
            //ignore header
            combined_str_csv_data_.conservativeResize(combined_str_csv_data_.rows() + row - 1, Eigen::NoChange);
            std::cout << "combined_str_csv_data_2:\n"
                      << combined_str_csv_data_.topRows(5) << std::endl;
            combined_str_csv_data_.bottomRows(row - 1) = current_file_str_matrix_map.bottomRows(row - 1);
            std::cout << "[rows,cols][" << i << "]: " << row - 1 << ", " << cols << std::endl;
            // std::cout<<"data["<<i<<"]:\n"<<current_file_str_matrix_map.bottomRows(row-1)<<std::endl;
        }

        //stdev calculation
        Eigen::MatrixXd tmp_pose_error = convertStringMatrixToMatrixXd(current_file_str_matrix_map.block(1, 1, row - 1, int(cols - 2)).transpose(), cols - 2, row - 1);
        // std::cout<<"tmp_pose_error["<<i<<"]:\n"<<tmp_pose_error.transpose()<<std::endl;
        std::string rounded_distance = current_file_str_matrix_map(row - 1, cols - 1);
        rounded_distance = std::to_string(int(std::stod(rounded_distance))) + std::string("cm");
        // std::cout<<"current_distance: "<<rounded_distance<<std::endl;
        distance_elements_.push_back(rounded_distance);
        if (i == 0) {
            pose_error_stddev_(i, 0) = 100 * getVectorStandardDeviation(tmp_pose_error.row(0));
            pose_error_stddev_(i, 1) = 100 * getVectorStandardDeviation(tmp_pose_error.row(1));
            pose_error_stddev_(i, 2) = getVectorStandardDeviation(tmp_pose_error.row(2));
            distance_vector_.conservativeResize(row - 1, 1);
            distance_vector_ << MatrixXs::Constant(row - 1, 1, rounded_distance);
        } else {
            pose_error_stddev_(i, 0) = 100 * getVectorStandardDeviation(tmp_pose_error.row(0));
            pose_error_stddev_(i, 1) = 100 * getVectorStandardDeviation(tmp_pose_error.row(1));
            pose_error_stddev_(i, 2) = getVectorStandardDeviation(tmp_pose_error.row(2));
            distance_vector_.conservativeResize(distance_vector_.rows() + row - 1, 1);
            distance_vector_.bottomRows(row - 1) = MatrixXs::Constant(row - 1, 1, rounded_distance);
        }
        std::cout << "combined_str_csv_data_3:\n"
                  << combined_str_csv_data_.topRows(5) << std::endl;
        // std::cout<<"combined_str_csv_data_:\n"<<combined_str_csv_data_<<std::endl;
    }
    // std::cout<<"combined_str_csv_data_:\n"<<combined_str_csv_data_<<std::endl;
    // std::cout<<"pose_error_stddev_:\n"<<pose_error_stddev_<<std::endl;
    std::cout << "combined_str_csv_data_:\n"
              << combined_str_csv_data_.topRows(10) << std::endl;
    // std::cout<<"combined_str_csv_data_.size(): "<<combined_str_csv_data_.rows()<<","<<combined_str_csv_data_.cols()<<std::endl;
    std::cout << "distance_vector_.size(): " << distance_vector_.rows() << "," << distance_vector_.cols() << std::endl;
}

void DistanceDataConcatenate::writeCombinedData()
{
    const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "\n");
    std::string time_series_csv_dir = ros::package::getPath("pose_analysis");
    time_series_csv_dir.append("/data/analysis/time_series_compare_data.csv");
    std::ofstream file_time(time_series_csv_dir);
    if (!file_time.is_open()) {
        throw std::runtime_error("writePoseError(): failed to open pose csv file.");
    }
    std::cout << "writing pose data to : " << time_series_csv_dir << std::endl;
    std::string distance_str = std::to_string(rack_distance_);
    MatrixRowXs output_mat;
    output_mat.conservativeResize(combined_str_csv_data_.rows(), time_file_header_.cols());
    output_mat.topRows(time_file_header_.rows()) = time_file_header_;
    std::cout << "combined_str_csv_data_.size(): " << combined_str_csv_data_.rows() << "," << combined_str_csv_data_.cols() << std::endl;
    std::cout << "output_mat.size(): " << output_mat.rows() << "," << output_mat.cols() << std::endl;
    output_mat.bottomLeftCorner(combined_str_csv_data_.rows() - 1, combined_str_csv_data_.cols() - 1) = combined_str_csv_data_.bottomLeftCorner(combined_str_csv_data_.rows() - 1, combined_str_csv_data_.cols() - 1);
    std::cout << "time_file_header_: " << time_file_header_ << std::endl;
    output_mat.bottomRightCorner(distance_vector_.rows(), distance_vector_.cols()) = distance_vector_;
    // std::cout<<"output_mat:\n"<<output_mat<<std::endl;
    file_time << output_mat.format(CSVFormat);
    file_time.close();

    // write stdev data
    std::string stats_csv_dir = ros::package::getPath("pose_analysis");
    stats_csv_dir.append("/data/analysis/stats_data.csv");
    // ROS_INFO("stats_csv_dir: %s",stats_csv_dir.c_str());
    std::ofstream file_stats(stats_csv_dir);
    int stats_mat_rows = combined_str_csv_data_.rows() - 1;
    int stats_mat_cols = combined_str_csv_data_.cols() - 1;
    // std::cout<<"combined_double_csv_data_:\n"<<combined_double_csv_data_.transpose()<<std::endl;
    if (pose_error_stddev_.cols() < 2) {
        throw std::runtime_error("empty data to estimate data standard deviation.");
    }
    MatrixRowXs header_stats;
    MatrixRowXs combine_matrix;
    MatrixXs stats_distance_vec = MatrixXsRefConst(distance_elements_.data(), distance_elements_.size(), 1);
    MatrixRowXs str_stdev_mat;
    std::vector<std::string> field_name = { "distance", "σ_x(cm)", "σ_y(cm)", "σ_yaw(°)" };
    header_stats.conservativeResize(1, 4);
    MatrixRowXd tmp_rowXd_matrix = pose_error_stddev_;
    str_stdev_mat = convertMatrixToStringMatrix(pose_error_stddev_, pose_error_stddev_.rows(), pose_error_stddev_.cols());
    combine_matrix.resize(stats_distance_vec.rows() + 1, str_stdev_mat.cols() + 1);
    // header_stats<<"distance(cm)",field_name[0],field_name[1],field_name[2];
    combine_matrix.topLeftCorner(header_stats.rows(), header_stats.cols()) = header_stats;
    combine_matrix.bottomLeftCorner(stats_distance_vec.rows(), stats_distance_vec.cols()) = stats_distance_vec;
    combine_matrix.bottomRightCorner(str_stdev_mat.rows(), str_stdev_mat.cols()) = str_stdev_mat;
    if (!file_stats.is_open()) {
        throw std::runtime_error("writePoseError(): failed to open stats csv file.");
    }
    std::cout << "writing stats data to : " << stats_csv_dir << std::endl;
    // std::cout<<"combine_matrix:\n"<<combine_matrix<<std::endl;
    file_stats << combine_matrix.format(CSVFormat);
    file_stats.close();
}

double DistanceDataConcatenate::getVectorStandardDeviation(const Eigen::VectorXd& _vec)
{
    double stddev = std::sqrt((_vec.array() - _vec.mean()).square().sum() / (_vec.size() - 1));
    return stddev;
}

template <class T>
Eigen::Matrix<std::string, -1, -1, 1> DistanceDataConcatenate::convertMatrixToStringMatrix(const Eigen::Matrix<T, -1, -1>& _input_mat, int _row_size, int _col_size)
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

Eigen::MatrixXd DistanceDataConcatenate::convertStringMatrixToMatrixXd(const MatrixRowXs& _input_mat, int _row_size, int _col_size)
{
    Eigen::Matrix<double, -1, -1, 1> output_mat;
    if (_row_size * _col_size != _input_mat.size()) {
        ROS_ERROR("incompatible matrix dimension.");
        return output_mat;
    }
    // std::cout<<"_input_mat:\n"<<_input_mat<<std::endl;
    output_mat.resize(_row_size, _col_size);
    for (int i = 0; i < _input_mat.size(); i++) {
        output_mat(i) = std::stod(_input_mat(i));
    }

    return output_mat;
}

typedef std::shared_ptr<DistanceDataConcatenate> DistanceDataConcatenatePtr;
}

#endif
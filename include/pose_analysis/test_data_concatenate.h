#ifndef TEST_DATA_CONCATENATE_H
#define TEST_DATA_CONCATENATE_H

#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <Eigen/Eigen>


namespace pose_analysis{

    using MatrixXs = Eigen::Matrix<std::string,-1,-1>;
    using MatrixRowXs = Eigen::Matrix<std::string,-1,-1,Eigen::RowMajor>;

    using MatrixXsRefConst = Eigen::Map<const MatrixXs>;
    using MatrixRowXsRefConst = Eigen::Map<const MatrixRowXs>;

    using MatrixXsRefConstPtr = std::shared_ptr<MatrixXsRefConst>;
    using MatrixRowXsRefConstPtr = std::shared_ptr<MatrixRowXsRefConst>;

    class TestDataConcatenate
    {
    private:
        ros::NodeHandlePtr nh_;
        ros::NodeHandlePtr pnh_;
        std::vector<std::string> allArgs_;
        MatrixRowXs combined_str_csv_data_;
        Eigen::Matrix<double,-1,-1,1> combined_double_csv_data_;
        double rack_distance_;
        MatrixXsRefConstPtr dir_map_;

        void getDirectories(int _argc,char**& _argv);
        void concatenateData();
        void writeCombinedData();
        double getVectorStandardDeviation(const Eigen::VectorXd& _vec);
        template <class T> Eigen::Matrix<std::string,-1,-1,1> convertMatrixToStringMatrix(const Eigen::Matrix<T,-1,-1>& _input_mat,int _row_size,int _col_size);
        Eigen::MatrixXd convertStringMatrixToMatrixXd(const MatrixRowXs& _input_mat,int _row_size,int _col_size);

    public:
        TestDataConcatenate(ros::NodeHandlePtr& _nh,ros::NodeHandlePtr& _pnh,int _argc,char**& _argv);
        ~TestDataConcatenate();
    };
    

    TestDataConcatenate::TestDataConcatenate(ros::NodeHandlePtr& _nh,ros::NodeHandlePtr& _pnh,int _argc,char**& _argv)
    {
        nh_ = _nh;
        pnh_ = _pnh;
        pnh_->getParam("rack_distance",rack_distance_); 
        getDirectories(_argc,_argv);
        std::cout<<"dir_map_:\n"<<*dir_map_<<std::endl;
        concatenateData();
        writeCombinedData();
        

    }
    
    TestDataConcatenate::~TestDataConcatenate(){}

    void TestDataConcatenate::getDirectories(int _argc,char**& _argv){
        allArgs_ = std::vector<std::string>(_argv, _argv + _argc);
        dir_map_ = std::make_shared<MatrixXsRefConst>(allArgs_.data()+1,_argc-1,1);
    }

    void TestDataConcatenate::concatenateData(){
        int test_index = 1;
        for(int i=0;i<dir_map_->rows();i++){
            std::vector<std::string> str_csv_data;
            std::vector<double> double_csv_data;
            std::ifstream file_time((*dir_map_)(i,0));
            std::string row_str;
            std::string entity_str;
            int row = 1;
            size_t cols;
            while(std::getline(file_time,row_str)){
                std::stringstream row_stream(row_str);
                while(std::getline(row_stream,entity_str,',')){
                    if(entity_str.empty()){
                        continue;
                    }
                    str_csv_data.push_back(entity_str);
                    // double_csv_data.push_back(std::stod(entity_str));
                }
                row++;
            }
            row--;
            cols = str_csv_data.size()/row;

            // string matrix concatenation
            if(i==0){
                MatrixRowXsRefConst current_file_str_matrix_map(str_csv_data.data(),row,cols);
                combined_str_csv_data_.conservativeResize(row,cols);
                combined_str_csv_data_ << current_file_str_matrix_map;
                // combined_str_csv_data_.bottomLeftCorner(row-1,1) = MatrixXs::Constant(row-1,1,std::string("test_")+std::to_string(test_index));
                // std::cout<<"[rows,cols]["<<i<<"]: "<<row<<", "<<cols<<std::endl;
                // std::cout<<"data["<<i<<"]:\n"<<current_file_str_matrix_map<<std::endl;
            }else{
                //check column consistancy
                if(cols != combined_str_csv_data_.cols()){
                    throw std::runtime_error("incompatible column between files.");
                }
                //ignore header
                MatrixRowXsRefConst current_file_str_matrix_map(str_csv_data.data(),row,cols);
                combined_str_csv_data_.conservativeResize(combined_str_csv_data_.rows()+row-1,Eigen::NoChange);
                combined_str_csv_data_.bottomRows(row-1) = current_file_str_matrix_map.bottomRows(row-1);
                // std::cout<<"[rows,cols]["<<i<<"]: "<<row-1<<", "<<cols<<std::endl;
                // std::cout<<"data["<<i<<"]:\n"<<current_file_str_matrix_map.bottomRows(row-1)<<std::endl;
            }
            combined_str_csv_data_.bottomLeftCorner(row-1,1) = MatrixXs::Constant(row-1,1,std::string("test_")+std::to_string(test_index));
            test_index++;
        }
        combined_str_csv_data_(0,0) = "test";
        // std::cout<<"combined data:\n"<<combined_str_csv_data_<<std::endl;
    }

    void TestDataConcatenate::writeCombinedData(){
        const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ", ", "\n");
        std::string time_series_csv_dir = ros::package::getPath("pose_analysis");
        time_series_csv_dir.append("/data/analysis/time_series_compare_data.csv");
        std::ofstream file_time(time_series_csv_dir);
        if(!file_time.is_open()){
            throw std::runtime_error("writePoseError(): failed to open pose csv file.");
        }
        std::cout<<"writing pose data to : "<<time_series_csv_dir<<std::endl;

        std::vector<std::string> field_name = {"test","x_error(m)","y_error(m)","yaw_error(°)","distance"};
        MatrixRowXs header_time = MatrixRowXsRefConst(field_name.data(),1,field_name.size());
        // header_time.conservativeResize(1,5);
        // header_time<<field_name[0],field_name[1],field_name[2],"distance";
        const int distance_vec_size = combined_str_csv_data_.cols();
        std::string distance_str = std::to_string(rack_distance_);
        MatrixXs distance_vec = Eigen::Matrix<std::string,-1,1>::Constant(combined_str_csv_data_.rows()-1,1,distance_str);
        MatrixRowXs output_mat;
        output_mat.conservativeResize(header_time.rows()+combined_str_csv_data_.rows()-1,header_time.cols());
        output_mat.topRows(header_time.rows()) = header_time;
        output_mat.bottomLeftCorner(combined_str_csv_data_.rows()-1,combined_str_csv_data_.cols()) = combined_str_csv_data_.bottomRightCorner(combined_str_csv_data_.rows()-1,combined_str_csv_data_.cols());
        output_mat.bottomRightCorner(distance_vec.rows(),distance_vec.cols()) = distance_vec;
        // std::cout<<"output_mat:\n"<<output_mat<<std::endl;
        file_time<<output_mat.format(CSVFormat);
        file_time.close();

        // write stdev data
        std::string stats_csv_dir = ros::package::getPath("pose_analysis");
        stats_csv_dir.append("/data/analysis/stats_data.csv");
        // ROS_INFO("stats_csv_dir: %s",stats_csv_dir.c_str());
        std::ofstream file_stats(stats_csv_dir);
        int stats_mat_rows = combined_str_csv_data_.rows()-1;
        int stats_mat_cols = combined_str_csv_data_.cols()-1;
        combined_double_csv_data_ = convertStringMatrixToMatrixXd(combined_str_csv_data_.bottomRightCorner(stats_mat_rows,stats_mat_cols),stats_mat_rows,stats_mat_cols);
        combined_double_csv_data_.transposeInPlace();
        // std::cout<<"combined_double_csv_data_:\n"<<combined_double_csv_data_.transpose()<<std::endl;
        if(combined_double_csv_data_.cols()<2){
            throw std::runtime_error("empty data to estimate data standard deviation.");
        }
        double stdev_x = 100*getVectorStandardDeviation(combined_double_csv_data_.row(0));
        double stdev_y = 100*getVectorStandardDeviation(combined_double_csv_data_.row(1));
        double stdev_yaw = getVectorStandardDeviation(combined_double_csv_data_.row(2));
        Eigen::MatrixXd stdev_mat;
        MatrixRowXs header_stats;
        MatrixRowXs combine_matrix;
        MatrixXs stats_distance_vec;
        MatrixRowXs str_stdev_mat;
        stdev_mat.resize(1,3);
        stdev_mat.row(0) << stdev_x,stdev_y,stdev_yaw;
        field_name = {"σ_x(cm)","σ_y(cm)","σ_yaw(°)"};
        header_stats.conservativeResize(1,4);
        str_stdev_mat = convertMatrixToStringMatrix(stdev_mat,1,3);
        stats_distance_vec.resize(1,1);
        combine_matrix.resize(stats_distance_vec.rows()+1,str_stdev_mat.cols()+1);
        header_stats<<"distance(cm)",field_name[0],field_name[1],field_name[2];
        stats_distance_vec<<std::to_string(rack_distance_);
        combine_matrix.topLeftCorner(header_stats.rows(),header_stats.cols()) = header_stats;
        combine_matrix.bottomLeftCorner(stats_distance_vec.rows(),stats_distance_vec.cols()) = stats_distance_vec;
        combine_matrix.bottomRightCorner(str_stdev_mat.rows(),str_stdev_mat.cols()) = str_stdev_mat;
        if(!file_stats.is_open()){
            throw std::runtime_error("writePoseError(): failed to open stats csv file.");
        }
        std::cout<<"writing stats data to : "<<stats_csv_dir<<std::endl;
        // std::cout<<"combine_matrix:\n"<<combine_matrix<<std::endl;
        file_stats<<combine_matrix.format(CSVFormat);
        file_stats.close();
    }

    double TestDataConcatenate::getVectorStandardDeviation(const Eigen::VectorXd& _vec){
        double stddev = std::sqrt((_vec.array()-_vec.mean()).square().sum()/(_vec.size()-1));
        return stddev;
    }

    template <class T>
    Eigen::Matrix<std::string,-1,-1,1> TestDataConcatenate::convertMatrixToStringMatrix(const Eigen::Matrix<T,-1,-1>& _input_mat,int _row_size,int _col_size){
        Eigen::Matrix<std::string,-1,-1,1> str_mat;
        if(_row_size*_col_size != _input_mat.size()){
            ROS_ERROR("incompatible matrix dimension.");
            return str_mat;
        }
        str_mat.resize(_row_size,_col_size);
        std::stringstream mat_ss;
        std::string row_str;
        std::string entity_str;
        int index = 0;
        mat_ss<<_input_mat;
        while(std::getline(mat_ss,row_str,'\n')){
            std::stringstream row_stream(row_str);
            while(std::getline(row_stream,entity_str,' ')){
                if(entity_str.empty()){
                    continue;
                }
                str_mat(index) = entity_str;
                index++;
            }
        }
        return str_mat;
    }
    
    Eigen::MatrixXd TestDataConcatenate::convertStringMatrixToMatrixXd(const MatrixRowXs& _input_mat,int _row_size,int _col_size){
        Eigen::Matrix<double,-1,-1,1> output_mat;
        if(_row_size*_col_size != _input_mat.size()){
            ROS_ERROR("incompatible matrix dimension.");
            return output_mat;
        }
        // std::cout<<"_input_mat:\n"<<_input_mat<<std::endl;
        output_mat.resize(_row_size,_col_size);
        for(int i=0;i<_input_mat.size();i++){
            output_mat(i) = std::stod(_input_mat(i));
        }

        return output_mat;
    }

    typedef std::shared_ptr<TestDataConcatenate> TestDataConcatenatePtr;

}

#endif
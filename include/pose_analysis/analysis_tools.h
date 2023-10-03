#ifndef ANALYSIS_TOOLS
#define ANALYSIS_TOOLS


#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// #include <tf_conversions/tf_eigen.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include "pose_analysis/eigenmvn.h"

#include <pcl/io/auto_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/bilateral_upsampling.h>
// #include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>


namespace pose_analysis{

    class AnalysisTools
    {
    private:
        ros::NodeHandlePtr nh_;
        ros::NodeHandlePtr pnh_;
        ros::Publisher model_pub_;
        ros::Publisher est_cloud_pub_;
        ros::Publisher noise_cloud_pub_;
        ros::Publisher noise_transformed_cloud_pub_;
        ros::Publisher upsample_model_cloud_pub_;
        ros::Publisher test_pose_pub_;
        ros::Publisher mesh_pub_;
        Eigen::Matrix4d gt_tf_;
        Eigen::Matrix4d est_tf_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr est_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr upsample_model_cloud_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud_;

        void readModelCloud();
        void readSourceCloud();
        void createSourceCloud();
        void createGroundtruthAtCameraFrame();
        void publishPointCloud2(const ros::Publisher& _publisher,pcl::PointCloud<pcl::PointXYZ>::ConstPtr _cloud,const ros::Time& _time,const std::string& _frame_id);
        void setModelCloudTransform();
        void getRelativePose();
        void publishTestingPose();
        void getPointCloudFromMesh(const std::string& _dir,double _rescale_factor);
        void publishMeshMsg(std::string _mesh_dir,double _rescale_factor);
        void writeModelPointCloud(std::string _pcd_dir);
        bool voxelGridSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& _input_cloud,const float& _leaf_size);
        Eigen::Matrix4d createEigenTransform(const Eigen::Matrix3d& _rotation,const Eigen::Vector3d& _translation);
        
    public:
        AnalysisTools(ros::NodeHandlePtr& _nh,ros::NodeHandlePtr& _pnh);
        double getVectorStandardDeviation(const Eigen::VectorXd& _vec);
        ~AnalysisTools();
    };

    AnalysisTools::AnalysisTools(ros::NodeHandlePtr& _nh,ros::NodeHandlePtr& _pnh)
    {
        nh_ = _nh;
        pnh_ = _pnh;
        model_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("model_cloud",1,true);
        est_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("est_cloud",1,true);
        upsample_model_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("upsample_model_cloud",1,true);
        noise_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("noise_cloud",1,true);
        noise_transformed_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("noise_transformed_cloud",1,true);
        test_pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("pose",3,true);
        mesh_pub_ = nh_->advertise<visualization_msgs::Marker>("mesh_marker",1,true);
        model_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        est_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        noise_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        upsample_model_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        readModelCloud();
        std::string mesh_dir = ros::package::getPath("pose_analysis");
        mesh_dir += "/data/mesh/_01_denser.stl";
        double rescaling_factor = 1e-2;
        // getPointCloudFromMesh(mesh_dir,rescaling_factor);

        // std::string pcd_dir = ros::package::getPath("pose_analysis");
        // pcd_dir += "/data/point_cloud/_01_denser.pcd";
        // writeModelPointCloud(pcd_dir);
        
        // // readSourceCloud();
        setModelCloudTransform();
        createSourceCloud();
        // // getRelativePose();
        // createGroundtruthAtCameraFrame();
        // publishTestingPose();
    }

    AnalysisTools::~AnalysisTools(){
        
    }

    void AnalysisTools::readModelCloud(){
        std::string pcd_dir = ros::package::getPath("pose_analysis");
        // pcd_dir += "/data/point_cloud/full_rack_model.pcd";
        pcd_dir += "/data/point_cloud/800x1200 PALLET.pcd";
        // std::cout<<"pcd_dir: "<<pcd_dir<<std::endl;
        if(pcl::io::loadPCDFile(pcd_dir,*model_cloud_)<0){
            throw std::runtime_error("failed to load pcd file.");
        }

        voxelGridSampling(model_cloud_,0.015f);
        // Eigen::Matrix4d model_init_tf = createEigenTransform((Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY())
        //                                     *Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d::UnitX())
        //                                     ).toRotationMatrix().inverse(),
        //                                 Eigen::Vector3d(-0.02,2.215,0.0));

        // Eigen::Matrix4d model_init_tf = createEigenTransform((Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitX())
        //                                     *Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY())
        //                                     ).toRotationMatrix(),
        //                                 Eigen::Vector3d(-0.02,2.215,-0.425));

        Eigen::Matrix4d model_init_tf = createEigenTransform((Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitZ())
                                        *Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitX())
                                        ).toRotationMatrix(),
                                        Eigen::Vector3d(0.4,-0.6,0.0));  
        // Eigen::Matrix4d model_init_tf = Eigen::Matrix4d::Identity();
        
        double scale_value = 1e-3;
        Eigen::Matrix4d scaled_tf = Eigen::Matrix4d::Identity();
        scaled_tf.block(0,0,3,3).diagonal() = Eigen::Vector3d::Constant(3,1,scale_value);

        gt_tf_ = Eigen::Matrix4d::Identity();
        // gt_tf_.topRightCorner(3,1) = Eigen::Vector3d(0.0,2.215,0);
        gt_tf_ = createEigenTransform(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.0,2.215,-0.425));

        // model_init_tf.block<3,1>(0,2) = Eigen::Vector3d::Zero(3);
        model_cloud_->getMatrixXfMap() = (model_init_tf*scaled_tf).cast<float>()*model_cloud_->getMatrixXfMap();
        // std::cout<<"model_init_tf:\n"<<model_init_tf<<std::endl;

        publishPointCloud2(model_pub_,model_cloud_,ros::Time::now(),"camera");
    }

    void AnalysisTools::setModelCloudTransform(){
        // Eigen::Matrix4d move_tf = createEigenTransform(Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d::UnitZ()).toRotationMatrix(),Eigen::Vector3d(0.0,0.0,0.0));
        Eigen::Matrix4d move_tf = Eigen::Matrix4d::Identity();
        // move_tf.block<3,1>(0,2) = Eigen::Vector3d::Zero(3);
        // std::cout<<"move_tf:\n"<<gt_tf_*move_tf*gt_tf_.inverse()<<std::endl;

        model_cloud_->getMatrixXfMap() = (gt_tf_*move_tf*gt_tf_.inverse()).cast<float>()*model_cloud_->getMatrixXfMap();
        publishPointCloud2(model_pub_,model_cloud_,ros::Time::now(),"camera");
    }

    void AnalysisTools::readSourceCloud(){
        std::string pcd_dir = ros::package::getPath("pose_analysis");
        pcd_dir += "/data/point_cloud/real_rack.pcd";
        // std::cout<<"pcd_dir: "<<pcd_dir<<std::endl;
        if(pcl::io::loadPCDFile(pcd_dir,*est_cloud_)<0){
            throw std::runtime_error("failed to load pcd file.");
        }

        Eigen::Matrix4d model_init_tf = createEigenTransform((Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitX())
                                            *Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY())
                                            ).toRotationMatrix().inverse(),
                                        Eigen::Vector3d(-0.02,2.215,0.0));

        Eigen::Matrix4d planar_tf = Eigen::Matrix4d::Identity();
        // planar_tf.block<3,1>(0,2) = Eigen::Vector3d::Zero(3);

        est_cloud_->getMatrixXfMap() = (planar_tf*model_init_tf).cast<float>()*est_cloud_->getMatrixXfMap();

        Eigen::Matrix4d delta_tf = createEigenTransform((Eigen::AngleAxisd(M_PI/4-M_PI*1.003/180,Eigen::Vector3d::UnitZ())*
                                                        Eigen::AngleAxisd(M_PI*3/180,Eigen::Vector3d::UnitY())*
                                                        Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX())).toRotationMatrix(),
                                        Eigen::Vector3d(0.0,0.0,0.0));

        est_tf_ = Eigen::Matrix4d::Identity();
        est_tf_ = gt_tf_*delta_tf;
        

        est_cloud_->getMatrixXfMap() = (est_tf_*gt_tf_.inverse()).cast<float>()*est_cloud_->getMatrixXfMap();
        

        publishPointCloud2(est_cloud_pub_,est_cloud_,ros::Time::now(),"camera");
    }

    void AnalysisTools::createSourceCloud(){
        // Eigen::Matrix4d delta_tf = createEigenTransform(Eigen::Matrix3d::Identity(),
        //                                Eigen::Vector3d(0.f,-0.1f,0.f));



        Eigen::Matrix4d delta_tf = createEigenTransform((Eigen::AngleAxisd(-M_PI*10.0/180,Eigen::Vector3d::UnitZ())).toRotationMatrix(),
                                        Eigen::Vector3d(0.0,0.0,0.0));
        
        Eigen::Matrix4d pallet_front_xsection_tf = Eigen::Matrix4d::Identity();
        pallet_front_xsection_tf(1,3) = -0.6;

        Eigen::Matrix4d proj_tf = Eigen::Matrix4d::Identity();
        proj_tf.block<3,1>(0,1) = Eigen::Vector3d::Zero();//project xz plane

        est_tf_ = Eigen::Matrix4d::Identity();
        est_tf_ = gt_tf_*delta_tf*pallet_front_xsection_tf*proj_tf;

        

        // est_tf_ = createEigenTransform((Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY())
        //                                     *Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d::UnitX())
        //                                     ).toRotationMatrix(),
        //                                 Eigen::Vector3d(0.02,2.215,0.0));


        // est_tf_ = createEigenTransform((Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d::UnitX())*
        //                                     Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitY())).toRotationMatrix(),
        //                                 Eigen::Vector3d(-0.02,2.215,0.0));

        // Eigen::Vector3d euler_est = est_tf_.block<3,3>(0,0).eulerAngles(0,1,2);
        // std::cout<<"euler_est: "<<euler_est.transpose()<<std::endl;
        

        est_cloud_->points.resize(model_cloud_->points.size());
        est_cloud_->getMatrixXfMap() = (est_tf_).cast<float>()*model_cloud_->getMatrixXfMap();
        // est_cloud_->getMatrixXfMap() = est_tf_.cast<float>()*model_cloud_->getMatrixXfMap();

        upsample_model_cloud_->points.resize(model_cloud_->points.size());
        upsample_model_cloud_->getMatrixXfMap() = (gt_tf_*delta_tf).cast<float>()*model_cloud_->getMatrixXfMap();


        publishPointCloud2(est_cloud_pub_,est_cloud_,ros::Time::now(),"camera");
        publishPointCloud2(upsample_model_cloud_pub_,upsample_model_cloud_,ros::Time::now(),"camera");
    }


    void AnalysisTools::publishPointCloud2(const ros::Publisher& _publisher,pcl::PointCloud<pcl::PointXYZ>::ConstPtr _cloud,const ros::Time& _time,const std::string& _frame_id){
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*_cloud,output);
        output.header.stamp = _time;
        output.header.frame_id = _frame_id;
        _publisher.publish(output);
    }

    Eigen::Matrix4d AnalysisTools::createEigenTransform(const Eigen::Matrix3d& _rotation,const Eigen::Vector3d& _translation){
        Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
        output.topLeftCorner(3,3) = _rotation;
        output.topRightCorner(3,1) = _translation;
        return output;
    }

    void AnalysisTools::getRelativePose(){
        Eigen::Matrix4d rel_tf = gt_tf_.inverse()*est_tf_;
        Eigen::Matrix4d planar_rel_tf = Eigen::Matrix4d::Identity();
        planar_rel_tf.topLeftCorner(2,2) = rel_tf.topLeftCorner(2,2);
        planar_rel_tf.topRightCorner(3,1) = rel_tf.topRightCorner(3,1);
        planar_rel_tf.topLeftCorner(3,3).colwise().normalize();
        // std::cout<<"gt_tf_:\n"<<gt_tf_<<std::endl;
        // std::cout<<"est_tf_:\n"<<est_tf_<<std::endl;
        std::cout<<"rel_tf:\n"<<rel_tf<<std::endl;
        std::cout<<"planar_rel_tf:\n"<<planar_rel_tf<<std::endl;


        Eigen::Vector3d output;
        Eigen::Vector3d output_planar;
        Eigen::Vector3d euler_xyz = 180/M_PI*(rel_tf.block<3,3>(0,0).eulerAngles(0,1,2));
        Eigen::Vector3d euler_planar = 180/M_PI*(planar_rel_tf.block<3,3>(0,0).eulerAngles(0,1,2));
        // if(euler_zyx(0)>=180){
        //     euler_zyx(0) = 180-euler_zyx(0);
        // }
        output<<rel_tf.topRightCorner(2,1),euler_planar(2);
        output_planar<<planar_rel_tf.topRightCorner(2,1),euler_xyz(2);
        std::cout<<"euler_xyz:"<<euler_xyz.transpose()<<std::endl;
        std::cout<<"euler_planar:"<<euler_planar.transpose()<<std::endl;
        std::cout<<"output: "<<output.transpose()<<std::endl;
    }

    void AnalysisTools::publishTestingPose(){
        Eigen::Vector3d mean = {0.0,0.0,0.0};
        Eigen::Matrix3d covar;
        double var_x = std::pow(0.02/3,2);
        double var_y = std::pow(0.01/3,2);
        double var_yaw = std::pow((M_PI*1.0/3.0)/180,2);
        covar << var_x,0,0,0,var_y,0,0,0,var_yaw;
        Eigen::EigenMultivariateNormal<double> gauss_gen(mean,covar);
        gauss_gen.setCovar(covar);
        gauss_gen.setMean(mean);
        const uint num_of_pts = 1;
        noise_cloud_->points.resize(num_of_pts);

        ros::Time measurement_time;
        std::string sensor_frame_id = "camera";
        
        Eigen::Vector3d stdevs;
        Eigen::Vector3d centroid;

        pcl::PointCloud<pcl::PointXYZ>::Ptr noise_transformed_cloud;
        noise_transformed_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        noise_transformed_cloud->points.resize(est_cloud_->points.size());

        ros::Rate rate(10);
        while(ros::ok()){
            measurement_time = ros::Time::now();
            noise_cloud_->getMatrixXfMap().topRows(3) = gauss_gen.samples(num_of_pts).cast<float>();
            Eigen::Matrix4d noise_delta_tf = createEigenTransform(Eigen::AngleAxisd(noise_cloud_->getMatrixXfMap()(2,0),Eigen::Vector3d::UnitZ()).toRotationMatrix(),
                                                                  Eigen::Vector3d(noise_cloud_->getMatrixXfMap()(0,0),noise_cloud_->getMatrixXfMap()(1,0),0.0));
            Eigen::Affine3d test_pose = Eigen::Affine3d::Identity();
            test_pose.matrix() = est_tf_*noise_delta_tf;
            noise_transformed_cloud->getMatrixXfMap() = (test_pose.matrix()*est_tf_.inverse()).cast<float>()*est_cloud_->getMatrixXfMap();

            publishPointCloud2(noise_transformed_cloud_pub_,noise_transformed_cloud,measurement_time,sensor_frame_id);
            publishPointCloud2(noise_cloud_pub_,noise_cloud_,measurement_time,sensor_frame_id);

            geometry_msgs::PoseStamped output_pose;
            tf::poseEigenToMsg(test_pose,output_pose.pose);
            output_pose.header.stamp = measurement_time;
            output_pose.header.frame_id = sensor_frame_id;
            test_pose_pub_.publish(output_pose);

            rate.sleep();
        }
    }

    double AnalysisTools::getVectorStandardDeviation(const Eigen::VectorXd& _vec){
        double stddev = std::sqrt((_vec.array()-_vec.mean()).square().sum()/(_vec.size()-1));
        return stddev;
    }

    void AnalysisTools::createGroundtruthAtCameraFrame(){
        Eigen::Matrix4d delta_tf = Eigen::Matrix4d::Identity();
        delta_tf.topLeftCorner(3,3) = Eigen::AngleAxisd(-M_PI*20/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();

        Eigen::Matrix4d gt_to_camera_tf = Eigen::Matrix4d::Identity();
        gt_to_camera_tf.topRightCorner(3,1) = Eigen::Vector3d(0.0,-0.425,2.215);
        gt_to_camera_tf.topLeftCorner(3,3) = (Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d::UnitX())*
                                Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ())).toRotationMatrix();;

        gt_to_camera_tf.topLeftCorner(3,3).colwise().normalize();
        std::cout<<"gt_to_camera_tf:\n"<<gt_to_camera_tf*delta_tf<<std::endl;
    }

    void AnalysisTools::getPointCloudFromMesh(const std::string& _dir,double _rescale_factor){
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFile(_dir,mesh);
        vtkSmartPointer<vtkPolyData> poly_data = vtkSmartPointer<vtkPolyData>::New();
        
        pcl::io::mesh2vtk(mesh,poly_data);
        pcl::io::vtkPolyDataToPointCloud(poly_data,*model_cloud_);
        voxelGridSampling(model_cloud_,0.01f);
        model_cloud_->getMatrixXfMap().topRows(3) = model_cloud_->getMatrixXfMap().topRows(3).array()*1e-2;
        std::cout<<"model_cloud_.size(): "<<model_cloud_->points.size()<<std::endl;
        std::cout<<"finish loading model point cloud."<<std::endl;
        // std::cout<<"model_cloud_:\n"<<model_cloud_->getMatrixXfMap().topLeftCorner(3,10).transpose()<<std::endl;

        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
        // pcl::PointCloud<pcl::PointNormal>::Ptr mls_pts = pcl::make_shared<pcl::PointCloud<pcl::PointNormal>>();
        // std::cout<<"mls_pts:\n"<<mls_pts->getMatrixXfMap().transpose()<<std::endl;

        // pcl::MovingLeastSquares<pcl::PointXYZ,pcl::PointNormal> mls;
        // mls.setInputCloud(model_cloud_);
        // mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
        // mls.setComputeNormals(true);
        // mls.setPolynomialOrder(2);
        // mls.setSearchMethod(tree);
        // mls.setSearchRadius(0.03);
        // // mls.setUpsamplingRadius(0.01);
        // // mls.setUpsamplingStepSize(0.02);
        // mls.setSqrGaussParam(std::pow(0.03,2));
        // mls.setNumberOfThreads(8);
        // // mls.setDilationVoxelSize(0.01);
        // mls.process(*mls_pts);
        // std::cout<<"mls_pts:\n"<<mls_pts->getMatrixXfMap().leftCols(10).transpose()<<std::endl;
        // upsample_model_cloud_->points.resize(mls_pts->points.size());
        // upsample_model_cloud_->getMatrixXfMap().topRows(3) = mls_pts->getMatrixXfMap().topRows(3);

        publishPointCloud2(model_pub_,model_cloud_,ros::Time::now(),"camera");
        // publishPointCloud2(upsample_model_cloud_pub_,upsample_model_cloud_,ros::Time::now(),"camera");
        publishMeshMsg(_dir,_rescale_factor);
    }

    void AnalysisTools::publishMeshMsg(std::string _mesh_dir,double _rescale_factor){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.mesh_resource = "package://pose_analysis/data/mesh/_01.stl";
        // marker.mesh_resource = _mesh_dir;
        marker.mesh_use_embedded_materials = true;
        marker.color.r = 0.4;
        marker.color.g = 0.4;
        marker.color.b = 0.4;
        marker.color.a = 1.0;
        marker.scale.x = 1e-2;
        marker.scale.y = 1e-2;
        marker.scale.z = 1e-2;

        mesh_pub_.publish(marker);
    }

    void AnalysisTools::writeModelPointCloud(std::string _pcd_dir){
        if(model_cloud_->empty()){
            throw std::runtime_error("empty point cloud.");
        }
        pcl::io::savePCDFileBinary(_pcd_dir,*model_cloud_);
        std::cout<<"saved point cloud data to: "<<_pcd_dir<<std::endl;
    }

    bool AnalysisTools::voxelGridSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr& _input_cloud,const float& _leaf_size){
        if(!_input_cloud){
            throw std::runtime_error("voxelGridSampling(): uninitialized pointer.");
            // return false;
        }
        if(_input_cloud->points.empty()){
            throw std::runtime_error("voxelGridSampling(): empty point cloud.");
            // return false;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(_input_cloud);
        vg.setLeafSize(_leaf_size,_leaf_size,_leaf_size);
        vg.filter(*tmp_cloud);
        tmp_cloud.swap(_input_cloud);
        return true;
    }
        

    typedef std::shared_ptr<AnalysisTools> AnalysisToolsPtr;

}



#endif
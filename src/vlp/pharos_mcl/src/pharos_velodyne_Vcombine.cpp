#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>

ros::NodeHandlePtr nh;
ros::NodeHandlePtr pnh;

ros::Publisher points_pub;
ros::Subscriber TV_points_sub;
ros::Subscriber LV_points_sub;
ros::Subscriber RV_points_sub;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

PointCloudPtr TVPoints_(new PointCloud);
PointCloudPtr LVPoints_(new PointCloud);
PointCloudPtr RVPoints_(new PointCloud);
PointCloudPtr transformed_TVPoints_(new PointCloud);
PointCloudPtr transformed_LVPoints_(new PointCloud);
PointCloudPtr transformed_RVPoints_(new PointCloud);

double TV_time_;
double LV_time_;
double RV_time_;

float time_diff;
bool match_;
bool match;

float VLP_T_X, VLP_T_Y, VLP_T_Z;
float VLP_L_X, VLP_L_Y, VLP_L_Z;
float VLP_R_X, VLP_R_Y, VLP_R_Z;


std::string vehicle_frame_id;
std::string topic_name;

PointCloudPtr Transform(PointCloudPtr source_cloud, double target_x, double target_y, double target_z, double target_yaw){

    PointCloudPtr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform (0,3) = target_x;
    transform (1,3) = target_y;
    transform (2,3) = target_z;

    transform (0,0) = cosf(target_yaw);
    transform (0,1) = -sinf(target_yaw);
    transform (1,0) = sinf(target_yaw);
    transform (1,1) = cosf(target_yaw);

    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform);

    return transformed_cloud;
}

void TVPointsCB(const sensor_msgs::PointCloud2ConstPtr &input){
    TV_time_ = double(input->header.stamp.sec) + double(input->header.stamp.nsec)*1e-9;
    pcl::fromROSMsg(*input,*TVPoints_);

    if(match_) match = (RV_time_==TV_time_) & (RV_time_==LV_time_);
    else match = (RV_time_-TV_time_<time_diff) & (RV_time_-LV_time_<time_diff);

    if(match){

        transformed_TVPoints_ = Transform(TVPoints_, VLP_T_X, VLP_T_Y, VLP_T_Z, 0.0);
        transformed_LVPoints_ = Transform(LVPoints_, VLP_L_X, VLP_L_Y, VLP_L_Z, 0.0); //1.57:0.65 1.62:0.7
        transformed_RVPoints_ = Transform(RVPoints_, VLP_R_X, VLP_R_Y, VLP_R_Z, 0.0);

        *transformed_TVPoints_ += *transformed_LVPoints_ + *transformed_RVPoints_;
        sensor_msgs::PointCloud2Ptr combined_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*transformed_TVPoints_, *combined_cloud);

        combined_cloud->header.frame_id = vehicle_frame_id;
        combined_cloud->header.stamp = input->header.stamp;
        points_pub.publish(combined_cloud);
    }
}
void LVPointsCB(const sensor_msgs::PointCloud2ConstPtr &input){
    LV_time_ = double(input->header.stamp.sec) + double(input->header.stamp.nsec)*1e-9;
    pcl::fromROSMsg(*input,*LVPoints_);

    if(match_) match = (RV_time_==TV_time_) & (RV_time_==LV_time_);
    else match = (RV_time_-TV_time_<time_diff) & (RV_time_-LV_time_<time_diff);

    if(match){

        transformed_TVPoints_ = Transform(TVPoints_, VLP_T_X, VLP_T_Y, VLP_T_Z, 0.0);
        transformed_LVPoints_ = Transform(LVPoints_, VLP_L_X, VLP_L_Y, VLP_L_Z, 0.0); //1.57:0.65 1.62:0.7
        transformed_RVPoints_ = Transform(RVPoints_, VLP_R_X, VLP_R_Y, VLP_R_Z, 0.0);

        *transformed_TVPoints_ += *transformed_LVPoints_ + *transformed_RVPoints_;
        sensor_msgs::PointCloud2Ptr combined_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*transformed_TVPoints_, *combined_cloud);

        combined_cloud->header.frame_id = vehicle_frame_id;
        combined_cloud->header.stamp = input->header.stamp;
        points_pub.publish(combined_cloud);
    }
}
void RVPointsCB(const sensor_msgs::PointCloud2ConstPtr &input){
    RV_time_ = double(input->header.stamp.sec) + double(input->header.stamp.nsec)*1e-9;
    pcl::fromROSMsg(*input,*RVPoints_);

    if(match_) match = (RV_time_==TV_time_) & (RV_time_==LV_time_);
    else match = (RV_time_-TV_time_<time_diff) & (RV_time_-LV_time_<time_diff);

    if(match){

        transformed_TVPoints_ = Transform(TVPoints_, VLP_T_X, VLP_T_Y, VLP_T_Z, 0.0);
        transformed_LVPoints_ = Transform(LVPoints_, VLP_L_X, VLP_L_Y, VLP_L_Z, 0.0); //1.57:0.65 1.62:0.7
        transformed_RVPoints_ = Transform(RVPoints_, VLP_R_X, VLP_R_Y, VLP_R_Z, 0.0);

        *transformed_TVPoints_ += *transformed_LVPoints_ + *transformed_RVPoints_;
        sensor_msgs::PointCloud2Ptr combined_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*transformed_TVPoints_, *combined_cloud);

        combined_cloud->header.frame_id = vehicle_frame_id;
        combined_cloud->header.stamp = input->header.stamp;
        points_pub.publish(combined_cloud);
    }
}

int Init(){
    nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
    pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));
    ros::NodeHandle tfnh("pharos_tf_broadcaster_node");

    pnh->param<float>("time_diff", time_diff, 0.05);
    pnh->param<std::string>("frame_id", vehicle_frame_id, "/vehicle_frame");
    pnh->param("match", match_, true);

    tfnh.getParam("T_velodyne_x",VLP_T_X);
    tfnh.getParam("T_velodyne_y",VLP_T_Y);
    tfnh.getParam("T_velodyne_z",VLP_T_Z);
    tfnh.getParam("L_velodyne_x",VLP_L_X);
    tfnh.getParam("L_velodyne_y",VLP_L_Y);
    tfnh.getParam("L_velodyne_z",VLP_L_Z);
    tfnh.getParam("R_velodyne_x",VLP_R_X);
    tfnh.getParam("R_velodyne_y",VLP_R_Y);
    tfnh.getParam("R_velodyne_z",VLP_R_Z);

    points_pub = nh->advertise<sensor_msgs::PointCloud2>("/vlp_cloud/mcl/vertical",1);

    TV_points_sub = nh->subscribe<sensor_msgs::PointCloud2>("/vlp_cloud/object_t",10, TVPointsCB);
    LV_points_sub = nh->subscribe<sensor_msgs::PointCloud2>("/vlp_cloud/object_l",10, LVPointsCB);
    RV_points_sub = nh->subscribe<sensor_msgs::PointCloud2>("/vlp_cloud/object_r",10, RVPointsCB);

    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pharos_velodyne_Vcombine");


    if(Init()){
        ROS_FATAL("pharos_velodyne_combine initialization failed");
        return -1;
    } else    ROS_INFO("started pharos_velodyne_Vcombine node");

    ros::spin();

    return 0;
}
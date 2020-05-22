#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>

ros::NodeHandlePtr nh;
ros::NodeHandlePtr pnh;

ros::Publisher points_pub;
ros::Subscriber TG_points_sub;
ros::Subscriber LG_points_sub;
ros::Subscriber RG_points_sub;

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

PointCloudPtr TGPoints_(new PointCloud);
PointCloudPtr LGPoints_(new PointCloud);
PointCloudPtr RGPoints_(new PointCloud);
PointCloudPtr transformed_TGPoints_(new PointCloud);
PointCloudPtr transformed_LGPoints_(new PointCloud);
PointCloudPtr transformed_RGPoints_(new PointCloud);

double TG_time_;
double LG_time_;
double RG_time_;

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


void TGPointsCB(const sensor_msgs::PointCloud2ConstPtr &input){
    TG_time_ = double(input->header.stamp.sec) + double(input->header.stamp.nsec)*1e-9;
    pcl::fromROSMsg(*input,*TGPoints_);

    if(match_) match = (RG_time_==TG_time_) & (RG_time_==LG_time_);
    else match = (RG_time_-TG_time_<time_diff) & (RG_time_-LG_time_<time_diff);

    if(match){

        // double now_time = ros::Time::now().toSec();
        // std::cout << "Rcombine dt : " << now_time-input->header.stamp.toSec() << std::endl;

        transformed_TGPoints_ = Transform(TGPoints_, VLP_T_X, VLP_T_Y, VLP_T_Z, 0.0);
        transformed_LGPoints_ = Transform(LGPoints_, VLP_L_X, VLP_L_Y, VLP_L_Z, 0.0); //1.57:0.65 1.62:0.7
        transformed_RGPoints_ = Transform(RGPoints_, VLP_R_X, VLP_R_Y, VLP_R_Z, 0.0);

        *transformed_TGPoints_ += *transformed_LGPoints_ + *transformed_RGPoints_;
        sensor_msgs::PointCloud2Ptr combined_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*transformed_TGPoints_, *combined_cloud);

        combined_cloud->header.frame_id = vehicle_frame_id;
        combined_cloud->header.stamp = input->header.stamp;
        points_pub.publish(combined_cloud);
    }
}
void LGPointsCB(const sensor_msgs::PointCloud2ConstPtr &input){
    LG_time_ = double(input->header.stamp.sec) + double(input->header.stamp.nsec)*1e-9;
    pcl::fromROSMsg(*input,*LGPoints_);

    if(match_) match = (RG_time_==TG_time_) & (RG_time_==LG_time_);
    else match = (RG_time_-TG_time_<time_diff) & (RG_time_-LG_time_<time_diff);

    if(match){
        // double now_time = ros::Time::now().toSec();
        // std::cout << "Rcombine dt : " << now_time-input->header.stamp.toSec() << std::endl;

        transformed_TGPoints_ = Transform(TGPoints_, VLP_T_X, VLP_T_Y, VLP_T_Z, 0.0);
        transformed_LGPoints_ = Transform(LGPoints_, VLP_L_X, VLP_L_Y, VLP_L_Z, 0.0); //1.57:0.65 1.62:0.7
        transformed_RGPoints_ = Transform(RGPoints_, VLP_R_X, VLP_R_Y, VLP_R_Z, 0.0);

        *transformed_TGPoints_ += *transformed_LGPoints_ + *transformed_RGPoints_;
        sensor_msgs::PointCloud2Ptr combined_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*transformed_TGPoints_, *combined_cloud);

        combined_cloud->header.frame_id = vehicle_frame_id;
        combined_cloud->header.stamp = input->header.stamp;
        points_pub.publish(combined_cloud);
    }
}
void RGPointsCB(const sensor_msgs::PointCloud2ConstPtr &input){
    RG_time_ = double(input->header.stamp.sec) + double(input->header.stamp.nsec)*1e-9;
    pcl::fromROSMsg(*input,*RGPoints_);

    if(match_) match = (RG_time_==TG_time_) & (RG_time_==LG_time_);
    else match = (RG_time_-TG_time_<time_diff) & (RG_time_-LG_time_<time_diff);

    if(match){
        // double now_time = ros::Time::now().toSec();
        // std::cout << "Rcombine dt : " << now_time-input->header.stamp.toSec() << std::endl;
        transformed_TGPoints_ = Transform(TGPoints_, VLP_T_X, VLP_T_Y, VLP_T_Z, 0.0);
        transformed_LGPoints_ = Transform(LGPoints_, VLP_L_X, VLP_L_Y, VLP_L_Z, 0.0); //1.57:0.65 1.62:0.7
        transformed_RGPoints_ = Transform(RGPoints_, VLP_R_X, VLP_R_Y, VLP_R_Z, 0.0);

        *transformed_TGPoints_ += *transformed_LGPoints_ + *transformed_RGPoints_;
        sensor_msgs::PointCloud2Ptr combined_cloud(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*transformed_TGPoints_, *combined_cloud);

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

    points_pub = nh->advertise<sensor_msgs::PointCloud2>("/vlp_cloud/mcl/road",1);

    TG_points_sub = nh->subscribe<sensor_msgs::PointCloud2>("/vlp_cloud/ground_t",10, TGPointsCB);
    LG_points_sub = nh->subscribe<sensor_msgs::PointCloud2>("/vlp_cloud/ground_l",10, LGPointsCB);
    RG_points_sub = nh->subscribe<sensor_msgs::PointCloud2>("/vlp_cloud/ground_r",10, RGPointsCB);

    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pharos_velodyne_Rcombine");


    if(Init()){
        ROS_FATAL("pharos_velodyne_combine initialization failed");
        return -1;
    } else    ROS_INFO("started pharos_velodyne_Rcombine node");

    ros::Rate loop_rate(100);
        while (nh->ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }   
    // ros::spin();

    return 0;
}
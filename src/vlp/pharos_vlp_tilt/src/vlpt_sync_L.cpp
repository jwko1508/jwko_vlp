//
// Created by jwkolab on 19. 5. 16.
//
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pharos_vlp_tilt/vector_perfect_array.h>
#include <pharos_vlp_tilt/VehiclePoseArray.h>
#include <pharos_msgs/ObjectInfoArray.h>
#include <nav_msgs/Odometry.h>
#include <pcl/conversions.h>
///*my custom msg
#include <velodyne_msgs/info.h>
#include <velodyne_msgs/custompoint.h>
#include <velodyne_msgs/cpoint.h>
///
#include <velodyne_msgs/VelodyneScan.h>
#include <pharos_msgs/StateStamped2016.h>

double diff_value;
bool isTimeLeft;
bool isErrorHori = false;
bool isLeftInit = false;

std::string sub_sync;
std::string sub_sync_center;
std::string pub_sync;
std::string frame_sync;

std::string pub_sync_cloud;


float L_velodyne_x , L_velodyne_y , L_velodyne_z;

std::vector<velodyne_msgs::custompoint> global_left;

class vlp_
{

public:

    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;

    vlp_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        ////////////param/////////////
        ///LEFT
        nh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_x",L_velodyne_x, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_y",L_velodyne_y, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_z",L_velodyne_z, 0);

        nh->param<bool>("isTimeLeft",isTimeLeft, false);

        /////////////sub/////////////
        pnh->param<std::string>("sub_sync",sub_sync, "left_my_msg");
        pnh->param<std::string>("sub_sync_center",sub_sync_center, "center_sync_msg");

        ////////////pub////////////////
        pnh->param<std::string>("pub_sync",pub_sync, "left_sync_msg");
        pnh->param<std::string>("pub_sync_cloud",pub_sync_cloud, "left_sync_cloud");

        pnh->param<std::string>("frame_sync",frame_sync, "mid_velodyne2");

        sub_left = nh->subscribe(sub_sync,10 , &vlp_::Left_VLP_CB,this );
        sub_center = nh->subscribe(sub_sync_center,10 , &vlp_::Vehicle_Top_CB,this );

        pub_left = nh->advertise<velodyne_msgs::custompoint>(pub_sync,10);

        pub_left_pcl = nh->advertise<sensor_msgs::PointCloud2>(pub_sync_cloud,10);
    }

    void Vehicle_Top_CB (const pharos_vlp_tilt::VehiclePoseArrayPtr &input)
    {


        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        if(!isLeftInit)
        {
            return;
        }

        pharos_vlp_tilt::VehiclePoseArrayPtr store_vehicle(new pharos_vlp_tilt::VehiclePoseArray);
        velodyne_msgs::custompointPtr point_(new velodyne_msgs::custompoint);

        store_vehicle = input;

//        std::cout<<"LLLLLLLLLLLLLLLLL : " << global_left.size() <<std::endl;

        double ltime = store_vehicle->header.stamp.toSec() - global_left[1].header.stamp.toSec();
        *point_ = global_left[1];

//        std::cout<<"before : " << "\t"<< ltime<<std::endl;

        if(ltime < 0)
        {
            ltime = store_vehicle->header.stamp.toSec() - global_left[0].header.stamp.toSec();
            *point_ = global_left[0];
        }

//        std::cout<<"after : " << "\t"<< ltime<<std::endl;
//        std::cout<<point_->infos[point_->cpoints.size() - 1].hori <<std::endl;



        for (int j = point_->cpoints.size() - 1; j >= 0; j--)
        {
            int lt_hori = ltime * 18000;

            int num = (point_->infos[point_->cpoints.size() - 1].hori - point_->infos[j].hori + lt_hori) / 180;

            if(lt_hori < 0 && j == point_->cpoints.size() - 1 && isErrorHori == false)
            {
                isErrorHori = true;
                ROS_ERROR("Sync_L is error!!!!!!!!! : %d",lt_hori);
            }

            Eigen::Vector2d position;
            position.x() = point_->cpoints[j].x;
            position.y() = point_->cpoints[j].y;

            position.x() = position.x() + store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].x + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].x - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].x)/180 * (180 - (point_->infos[point_->cpoints.size() - 1].hori - point_->infos[j].hori + lt_hori) % 180);
            position.y() = position.y() + store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].y + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].y - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].y)/180 * (180 - (point_->infos[point_->cpoints.size() - 1].hori - point_->infos[j].hori + lt_hori) % 180);

//            std::cout<<position.x()<<std::endl;

            double point__theta;
            point__theta = store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].theta + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].theta - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].theta)/180 * (180 - (point_->infos[point_->cpoints.size() - 1].hori - point_->infos[j].hori + lt_hori) % 180);


            Eigen::Matrix2d rotation2Dmatrix;
            rotation2Dmatrix << cos(point__theta), -sin(point__theta),
                    sin(point__theta), cos(point__theta);

            position = rotation2Dmatrix * position;

            point_->cpoints[j].x = position.x() - L_velodyne_x;
            point_->cpoints[j].y = position.y() - L_velodyne_y;
        }

        isErrorHori = false;

        point_->header.stamp = store_vehicle->vehicles[store_vehicle->vehicles.size() - 1].stamp.data;

        pub_left.publish(point_);

        pcl::PointCloud<pcl::PointXYZI> test_cloud_left;


        for (int l = 0; l < point_->cpoints.size(); l++)
        {
            pcl::PointXYZI pt;

            pt.x = point_->cpoints[l].x;
            pt.y = point_->cpoints[l].y;
            pt.z = point_->cpoints[l].z;
            pt.intensity = point_->cpoints[l].intensity;

            test_cloud_left.push_back(pt);
        }

        sensor_msgs::PointCloud2Ptr left_comped(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(test_cloud_left, *left_comped);

        left_comped->header.stamp = store_vehicle->vehicles[store_vehicle->vehicles.size() - 1].stamp.data;
        left_comped->header.frame_id = frame_sync;
        pub_left_pcl.publish(left_comped);

        ros::Time last = ros::Time::now();

        diff_value = last.toSec() - begin.toSec();

        if(isTimeLeft == true)
        {
            ROS_INFO("sync_L time : %lf", diff_value);
        }
    }

    void Left_VLP_CB (const velodyne_msgs::custompoint &input)
    {
        velodyne_msgs::custompointPtr left(new velodyne_msgs::custompoint);
        *left = input;

        isLeftInit =true;

        for (int i = 0; i < left->cpoints.size(); i++)
        {
            left->cpoints[i].x += L_velodyne_x;
            left->cpoints[i].y += L_velodyne_y;
        }

        if(global_left.size() > 1)
        {
            global_left.erase(global_left.begin() + 0);
        }

        global_left.push_back(*left);

        if(global_left.size() == 1)
        {
            global_left.push_back(*left);
        }

    }

protected:
    ros::Subscriber sub_left;
    ros::Subscriber sub_center;

    ros::Publisher pub_left;
    ros::Publisher pub_left_pcl;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_sync_L");       //노드명 초기화

    ROS_INFO("started vlpt_sync_L");

    vlp_ hello;

    ros::spin();
    return 0;

}

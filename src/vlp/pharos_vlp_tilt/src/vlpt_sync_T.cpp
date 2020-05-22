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
bool isTimeTop;

std::string sub_sync;
std::string sub_sync_center;
std::string pub_sync;
std::string frame_sync;

std::string pub_sync_cloud;


float T_velodyne_x , T_velodyne_y , T_velodyne_z;

velodyne_msgs::custompointPtr global_top(new velodyne_msgs::custompoint);

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
        ///top
        nh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_x",T_velodyne_x, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_y",T_velodyne_y, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_z",T_velodyne_z, 0);

        nh->param<bool>("isTimeTop",isTimeTop, false);

        /////////////sub/////////////
        pnh->param<std::string>("sub_sync",sub_sync, "left_my_msg");
        pnh->param<std::string>("sub_sync_center",sub_sync_center, "center_sync_msg");

        ////////////pub////////////////
        pnh->param<std::string>("pub_sync",pub_sync, "left_sync_msg");
        pnh->param<std::string>("pub_sync_cloud",pub_sync_cloud, "left_sync_cloud");

        pnh->param<std::string>("frame_sync",frame_sync, "mid_velodyne2");

        sub_top = nh->subscribe(sub_sync,10 , &vlp_::Top_VLP_CB,this );
        sub_center = nh->subscribe(sub_sync_center,10 , &vlp_::Vehicle_Top_CB,this );

        pub_top = nh->advertise<velodyne_msgs::custompoint>(pub_sync,10);

        pub_top_pcl = nh->advertise<sensor_msgs::PointCloud2>(pub_sync_cloud,10);
    }

    void Vehicle_Top_CB (const pharos_vlp_tilt::VehiclePoseArrayPtr &input)
    {

        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        pharos_vlp_tilt::VehiclePoseArrayPtr store_vehicle(new pharos_vlp_tilt::VehiclePoseArray);

        store_vehicle = input;

        for (int j = global_top->cpoints.size() - 1; j >= 0; j--)
        {
            int num = (global_top->infos[global_top->cpoints.size() - 1].hori - global_top->infos[j].hori) / 180;

//            std::cout<<store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].x<<std::endl;

            Eigen::Vector2d position;
            position.x() = global_top->cpoints[j].x;
            position.y() = global_top->cpoints[j].y;

            position.x() = position.x() + store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].x + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].x - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].x)/180 * (180 - (global_top->infos[global_top->cpoints.size() - 1].hori - global_top->infos[j].hori) % 180);
            position.y() = position.y() + store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].y + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].y - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].y)/180 * (180 - (global_top->infos[global_top->cpoints.size() - 1].hori - global_top->infos[j].hori) % 180);

//            std::cout<<position.x()<<std::endl;

            double global_top_theta;
            global_top_theta = store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].theta + (store_vehicle->vehicles[store_vehicle->vehicles.size() - 1 - num].theta - store_vehicle->vehicles[store_vehicle->vehicles.size() - 2 - num].theta)/180 * (180 - (global_top->infos[global_top->cpoints.size() - 1].hori - global_top->infos[j].hori) % 180);


            Eigen::Matrix2d rotation2Dmatrix;
            rotation2Dmatrix << cos(global_top_theta), -sin(global_top_theta),
                    sin(global_top_theta), cos(global_top_theta);

            position = rotation2Dmatrix * position;

            global_top->cpoints[j].x = position.x() - T_velodyne_x;
            global_top->cpoints[j].y = position.y() - T_velodyne_y;
        }


        global_top->header.stamp = store_vehicle->vehicles[store_vehicle->vehicles.size() - 1].stamp.data;

        pub_top.publish(global_top);

        pcl::PointCloud<pcl::PointXYZI> test_cloud_top;


        for (int l = 0; l < global_top->cpoints.size(); l++)
        {
            pcl::PointXYZI pt;

            pt.x = global_top->cpoints[l].x;
            pt.y = global_top->cpoints[l].y;
            pt.z = global_top->cpoints[l].z;
            pt.intensity = global_top->cpoints[l].intensity;

            test_cloud_top.push_back(pt);
        }

        sensor_msgs::PointCloud2Ptr top_comped(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(test_cloud_top, *top_comped);

        top_comped->header.stamp = store_vehicle->vehicles[store_vehicle->vehicles.size() - 1].stamp.data;
        top_comped->header.frame_id = frame_sync;
        pub_top_pcl.publish(top_comped);

        ros::Time last = ros::Time::now();

        diff_value = last.toSec() - begin.toSec();

        if(isTimeTop == true)
        {
            ROS_INFO("sync_T time : %lf", diff_value);
        }
    }

    void Top_VLP_CB (const velodyne_msgs::custompoint &input)
    {
        *global_top = input;

        for (int i = 0; i < global_top->cpoints.size(); i++)
        {
            global_top->cpoints[i].x += T_velodyne_x;
            global_top->cpoints[i].y += T_velodyne_y;
        }

    }

protected:
    ros::Subscriber sub_top;
    ros::Subscriber sub_center;

    ros::Publisher pub_top;
    ros::Publisher pub_top_pcl;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_sync_T");       //노드명 초기화

    ROS_INFO("started vlpt_sync_T");

    vlp_ hello;

    ros::spin();
    return 0;

}

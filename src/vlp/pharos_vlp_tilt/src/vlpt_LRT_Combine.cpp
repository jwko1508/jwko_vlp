//
// Created by won on 13/9/18.
//



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pharos_vlp_tilt/vector_perfect_array.h>
#include <pharos_msgs/ObjectInfoArray.h>
#include <nav_msgs/Odometry.h>
///전역변수///
pharos_vlp_tilt::vector_perfect_arrayPtr global_left (new pharos_vlp_tilt::vector_perfect_array);
pharos_vlp_tilt::vector_perfect_arrayPtr global_right (new pharos_vlp_tilt::vector_perfect_array);
pharos_vlp_tilt::vector_perfect_arrayPtr global_top (new pharos_vlp_tilt::vector_perfect_array);
///------////
double LRThreshold;
double CENTER_THRSHOLD;
double XYTHRESHOLD;
double LimitCenter;
float L_velodyne_x , L_velodyne_y , L_velodyne_z;
float R_velodyne_x , R_velodyne_y , R_velodyne_z;
float T_velodyne_x , T_velodyne_y , T_velodyne_z;

double diff_value;
bool isTimeCombine;
double pose_gap;

std::string frame_combine;
std::string sub_combine_top;
std::string sub_combine_right;
std::string sub_combine_left;
std::string pub_combine;
std::string pub_combine_cloud;

struct PoseType
{
    /// Center
    pharos_vlp_tilt::point odom_point;
    pharos_vlp_tilt::point odom_quaternion;

    double odom_pose_stamp;
};


PoseType global_left_pose;
PoseType global_right_pose;
PoseType global_top_pose;

nav_msgs::Odometry Odom_;

std::vector<PoseType> Odom_Array_;




struct CombineType
{
    /// Center
    double center_x;
    double center_y;
    double center_z;

    pharos_vlp_tilt::point min_object;
    pharos_vlp_tilt::point max_object;

};


class vlp_
{
private:
    pharos_vlp_tilt::perfect perfcc;

public:

    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;

    vlp_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        nh->param<double>("LR_THRSHOLD",LRThreshold, 0.8);
        nh->param<double>("CENTER_THRSHOLD",CENTER_THRSHOLD, 1.2);
        nh->param<double>("XYTHRESHOLD",XYTHRESHOLD, 0.3);
        nh->param<double>("LimitCenter",LimitCenter, 10);
        nh->param<double>("pose_gap",pose_gap, 0.015);
        nh->param<bool>("isTimeCombine",isTimeCombine, false);

        pnh->param<std::string>("frame_combine",frame_combine , "ublox_ant" );
        pnh->param<std::string>("sub_combine_top",sub_combine_top , "ublox_ant" );
        pnh->param<std::string>("sub_combine_right",sub_combine_right , "ublox_ant" );
        pnh->param<std::string>("sub_combine_left",sub_combine_left , "ublox_ant" );
        pnh->param<std::string>("pub_combine",pub_combine , "ublox_ant" );
        pnh->param<std::string>("pub_combine_cloud",pub_combine_cloud , "ublox_ant" );
        ///LEFT
        nh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_x",L_velodyne_x, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_y",L_velodyne_y, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_z",L_velodyne_z, 0);
        ///RIGHT
        nh->param<float>("/pharos_tf_broadcaster_node/R_velodyne_x",R_velodyne_x, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/R_velodyne_y",R_velodyne_y, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/R_velodyne_z",R_velodyne_z, 0);
        ///MID
        nh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_x",T_velodyne_x, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_y",T_velodyne_y, 0);
        nh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_z",T_velodyne_z, 0);


        sub_right = nh->subscribe(sub_combine_right, 1 , & vlp_::RCombine,this);
        sub_left = nh->subscribe(sub_combine_left, 1 , & vlp_::LCombine,this);
        sub_top = nh->subscribe(sub_combine_top, 1 , & vlp_::TCombine,this);

        pub_combine_pointcloud = nh->advertise<sensor_msgs::PointCloud2>(pub_combine_cloud,1);
        pub_my_final_output = nh->advertise<pharos_vlp_tilt::vector_perfect_array>(pub_combine,1);

//        pub_test_min_max = nh->advertise<sensor_msgs::PointCloud2>("square",1);


        /////////////////1026///////////////////////
//        pub_left_xyz = nh->advertise<sensor_msgs::PointCloud2>("L_LR_POINTS",10);
//        pub_right_xyz = nh->advertise<sensor_msgs::PointCloud2>("R_LR_POINTS",10);
//        pub_top_xyz = nh->advertise<sensor_msgs::PointCloud2>("T_LR_POINTS",10);
//        pub_vlp_l_center = nh->advertise<sensor_msgs::PointCloud2>("L_Center",10);
//        pub_vlp_r_center = nh->advertise<sensor_msgs::PointCloud2>("R_Center",10);
//        pub_vlp_t_center = nh->advertise<sensor_msgs::PointCloud2>("T_Center",10);
    }


    void ListSeqeunce(pharos_vlp_tilt::perfectarray& one  , std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& pointvector
            , std::vector<CombineType>& combinevector , double X , double Y, double Z    )
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        CombineType object_pose_info;

        ///

        pcl::PointXYZI pt;


        for(unsigned int i=0; i < one.objects.size(); i++)
        {

            one.objects[i].point.x += X;
            one.objects[i].point.y += Y;
            one.objects[i].point.z += Z;


            pt.x = one.objects[i].point.x;
            pt.y = one.objects[i].point.y;
            pt.z = one.objects[i].point.z;
            object_cloud->push_back(pt);
        }

        object_pose_info.min_object.x = one.min_object.x + X;
        object_pose_info.min_object.y = one.min_object.y + Y;
        object_pose_info.min_object.z = one.min_object.z + Z;
        object_pose_info.max_object.x = one.max_object.x + X;
        object_pose_info.max_object.y = one.max_object.y + Y;
        object_pose_info.max_object.z = one.max_object.z + Z;

        /// Center Position
        object_pose_info.center_x = one.center.x + X;
        object_pose_info.center_y = one.center.y + Y;
        object_pose_info.center_z = one.center.z + Z;

        pointvector.push_back(object_cloud);
        combinevector.push_back(object_pose_info);


    }

    void DoCombine(CombineType& c_std, CombineType& c_obj, pcl::PointCloud<pcl::PointXYZI>::Ptr& p_std, pcl::PointCloud<pcl::PointXYZI>::Ptr& p_obj)
    {
        for (int k = 0; k < p_obj->points.size(); k++)
        {
            p_std->push_back(p_obj->points[k]);
        }

        c_std.center_x = (c_std.center_x + c_obj.center_x) / 2;
        c_std.center_y = (c_std.center_y + c_obj.center_y) / 2;
        c_std.center_z = (c_std.center_z + c_obj.center_z) / 2;

        c_std.min_object.x = (c_std.min_object.x <= c_obj.min_object.x) ? c_std.min_object.x : c_obj.min_object.x;
        c_std.min_object.y = (c_std.min_object.y <= c_obj.min_object.y) ? c_std.min_object.y : c_obj.min_object.y;
        c_std.min_object.z = (c_std.min_object.z <= c_obj.min_object.z) ? c_std.min_object.z : c_obj.min_object.z;
        c_std.max_object.x = (c_std.max_object.x >= c_obj.max_object.x) ? c_std.max_object.x : c_obj.max_object.x;
        c_std.max_object.y = (c_std.max_object.y >= c_obj.max_object.y) ? c_std.max_object.y : c_obj.max_object.y;
        c_std.max_object.z = (c_std.max_object.z >= c_obj.max_object.z) ? c_std.max_object.z : c_obj.max_object.z;
    }


    void CombineObject( std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& l_pointvector
            , std::vector<CombineType>& l_combinevector
            , std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& r_pointvector
            , std::vector<CombineType>& r_combinevector)
    {
        /// 중심점을 비교해서 한다.

        for(int i = 0; i < l_combinevector.size(); i++)
        {
            for(int j = 0; j < r_combinevector.size(); j++)
            {
                if(l_combinevector[i].max_object.x >= r_combinevector[j].max_object.x - XYTHRESHOLD && l_combinevector[i].min_object.x <= r_combinevector[j].max_object.x + XYTHRESHOLD
                   && l_combinevector[i].max_object.y >= r_combinevector[j].max_object.y - XYTHRESHOLD && l_combinevector[i].min_object.y <= r_combinevector[j].max_object.y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    r_combinevector.erase(r_combinevector.begin() + j);
                    r_pointvector.erase(r_pointvector.begin() + j);
                    j = j - 1;
                }

                else if(l_combinevector[i].max_object.x >= r_combinevector[j].min_object.x - XYTHRESHOLD && l_combinevector[i].min_object.x <= r_combinevector[j].min_object.x + XYTHRESHOLD
                        && l_combinevector[i].max_object.y >= r_combinevector[j].min_object.y - XYTHRESHOLD && l_combinevector[i].min_object.y <= r_combinevector[j].min_object.y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    r_combinevector.erase(r_combinevector.begin() + j);
                    r_pointvector.erase(r_pointvector.begin() + j);
                    j = j - 1;
                }

                else if(l_combinevector[i].max_object.x >= r_combinevector[j].max_object.x - XYTHRESHOLD && l_combinevector[i].min_object.x <= r_combinevector[j].max_object.x + XYTHRESHOLD
                        && l_combinevector[i].max_object.y >= r_combinevector[j].min_object.y - XYTHRESHOLD && l_combinevector[i].min_object.y <= r_combinevector[j].min_object.y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    r_combinevector.erase(r_combinevector.begin() + j);
                    r_pointvector.erase(r_pointvector.begin() + j);
                    j = j - 1;
                }

                else if(l_combinevector[i].max_object.x >= r_combinevector[j].min_object.x - XYTHRESHOLD && l_combinevector[i].min_object.x <= r_combinevector[j].min_object.x + XYTHRESHOLD
                        && l_combinevector[i].max_object.y >= r_combinevector[j].max_object.y - XYTHRESHOLD && l_combinevector[i].min_object.y <= r_combinevector[j].max_object.y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    r_combinevector.erase(r_combinevector.begin() + j);
                    r_pointvector.erase(r_pointvector.begin() + j);
                    j = j - 1;
                }

//                else if(r_combinevector[j].max_object.x >= l_combinevector[i].max_object.x - XYTHRESHOLD && r_combinevector[j].min_object.x <= l_combinevector[i].max_object.x + XYTHRESHOLD
//                   && r_combinevector[j].max_object.y >= l_combinevector[i].max_object.y - XYTHRESHOLD && r_combinevector[j].min_object.y <= l_combinevector[i].max_object.y + XYTHRESHOLD)
//                {
//
//                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);
//
//                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
//                    r_combinevector.erase(r_combinevector.begin() + j);
//                    r_pointvector.erase(r_pointvector.begin() + j);
//                    j = j - 1;
//                }
//
//                else if(r_combinevector[j].max_object.x >= l_combinevector[i].min_object.x - XYTHRESHOLD && r_combinevector[j].min_object.x <= l_combinevector[i].min_object.x + XYTHRESHOLD
//                        && r_combinevector[j].max_object.y >= l_combinevector[i].min_object.y - XYTHRESHOLD && r_combinevector[j].min_object.y <= l_combinevector[i].min_object.y + XYTHRESHOLD)
//                {
//
//                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);
//
//                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
//                    r_combinevector.erase(r_combinevector.begin() + j);
//                    r_pointvector.erase(r_pointvector.begin() + j);
//                    j = j - 1;
//                }
//
//                else if(r_combinevector[j].max_object.x >= l_combinevector[i].max_object.x - XYTHRESHOLD && r_combinevector[j].min_object.x <= l_combinevector[i].max_object.x + XYTHRESHOLD
//                        && r_combinevector[j].max_object.y >= l_combinevector[i].min_object.y - XYTHRESHOLD && r_combinevector[j].min_object.y <= l_combinevector[i].min_object.y + XYTHRESHOLD)
//                {
//
//                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);
//
//                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
//                    r_combinevector.erase(r_combinevector.begin() + j);
//                    r_pointvector.erase(r_pointvector.begin() + j);
//                    j = j - 1;
//                }
//
//                else if(r_combinevector[j].max_object.x >= l_combinevector[i].min_object.x - XYTHRESHOLD && r_combinevector[j].min_object.x <= l_combinevector[i].min_object.x + XYTHRESHOLD
//                        && r_combinevector[j].max_object.y >= l_combinevector[i].max_object.y - XYTHRESHOLD && r_combinevector[j].min_object.y <= l_combinevector[i].max_object.y + XYTHRESHOLD)
//                {
//
//                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);
//
//                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
//                    r_combinevector.erase(r_combinevector.begin() + j);
//                    r_pointvector.erase(r_pointvector.begin() + j);
//                    j = j - 1;
//                }

                else if(r_combinevector[j].max_object.x >= l_combinevector[i].center_x - XYTHRESHOLD && r_combinevector[j].min_object.x <= l_combinevector[i].center_x + XYTHRESHOLD
                        && r_combinevector[j].max_object.y >= l_combinevector[i].center_y - XYTHRESHOLD && r_combinevector[j].min_object.y <= l_combinevector[i].center_y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    r_combinevector.erase(r_combinevector.begin() + j);
                    r_pointvector.erase(r_pointvector.begin() + j);
                    j = j - 1;
                }

                else if(l_combinevector[i].max_object.x >= r_combinevector[j].center_x - XYTHRESHOLD && l_combinevector[i].min_object.x <= r_combinevector[j].center_x + XYTHRESHOLD
                        && l_combinevector[i].max_object.y >= r_combinevector[j].center_y - XYTHRESHOLD && l_combinevector[i].min_object.y <= r_combinevector[j].center_y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], r_combinevector[j], l_pointvector[i], r_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    r_combinevector.erase(r_combinevector.begin() + j);
                    r_pointvector.erase(r_pointvector.begin() + j);
                    j = j - 1;
                }
                else{}

            }


        }
//
    }

    void CombineSelf(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& l_pointvector
            , std::vector<CombineType>& l_combinevector)
    {
        for(int i = 0; i < l_combinevector.size(); i++)
        {
            for(int j = i + 1; j < l_combinevector.size(); j++)
            {
                if(l_combinevector[i].max_object.x >= l_combinevector[j].max_object.x - XYTHRESHOLD && l_combinevector[i].min_object.x <= l_combinevector[j].max_object.x + XYTHRESHOLD
                   && l_combinevector[i].max_object.y >= l_combinevector[j].max_object.y - XYTHRESHOLD && l_combinevector[i].min_object.y <= l_combinevector[j].max_object.y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], l_combinevector[j], l_pointvector[i], l_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    l_combinevector.erase(l_combinevector.begin() + j);
                    l_pointvector.erase(l_pointvector.begin() + j);
                    j = i;
                }

                else if(l_combinevector[i].max_object.x >= l_combinevector[j].min_object.x - XYTHRESHOLD && l_combinevector[i].min_object.x <= l_combinevector[j].min_object.x + XYTHRESHOLD
                        && l_combinevector[i].max_object.y >= l_combinevector[j].min_object.y - XYTHRESHOLD && l_combinevector[i].min_object.y <= l_combinevector[j].min_object.y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], l_combinevector[j], l_pointvector[i], l_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    l_combinevector.erase(l_combinevector.begin() + j);
                    l_pointvector.erase(l_pointvector.begin() + j);
                    j = i;
                }

                else if(l_combinevector[i].max_object.x >= l_combinevector[j].max_object.x - XYTHRESHOLD && l_combinevector[i].min_object.x <= l_combinevector[j].max_object.x + XYTHRESHOLD
                        && l_combinevector[i].max_object.y >= l_combinevector[j].min_object.y - XYTHRESHOLD && l_combinevector[i].min_object.y <= l_combinevector[j].min_object.y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], l_combinevector[j], l_pointvector[i], l_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    l_combinevector.erase(l_combinevector.begin() + j);
                    l_pointvector.erase(l_pointvector.begin() + j);
                    j = i;
                }

                else if(l_combinevector[i].max_object.x >= l_combinevector[j].min_object.x - XYTHRESHOLD && l_combinevector[i].min_object.x <= l_combinevector[j].min_object.x + XYTHRESHOLD
                        && l_combinevector[i].max_object.y >= l_combinevector[j].max_object.y - XYTHRESHOLD && l_combinevector[i].min_object.y <= l_combinevector[j].max_object.y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], l_combinevector[j], l_pointvector[i], l_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    l_combinevector.erase(l_combinevector.begin() + j);
                    l_pointvector.erase(l_pointvector.begin() + j);
                    j = i;
                }

                else if(l_combinevector[j].max_object.x >= l_combinevector[i].center_x - XYTHRESHOLD && l_combinevector[j].min_object.x <= l_combinevector[i].center_x + XYTHRESHOLD
                        && l_combinevector[j].max_object.y >= l_combinevector[i].center_y - XYTHRESHOLD && l_combinevector[j].min_object.y <= l_combinevector[i].center_y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], l_combinevector[j], l_pointvector[i], l_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    l_combinevector.erase(l_combinevector.begin() + j);
                    l_pointvector.erase(l_pointvector.begin() + j);
                    j = i;
                }

                else if(l_combinevector[i].max_object.x >= l_combinevector[j].center_x - XYTHRESHOLD && l_combinevector[i].min_object.x <= l_combinevector[j].center_x + XYTHRESHOLD
                        && l_combinevector[i].max_object.y >= l_combinevector[j].center_y - XYTHRESHOLD && l_combinevector[i].min_object.y <= l_combinevector[j].center_y + XYTHRESHOLD)
                {

                    DoCombine(l_combinevector[i], l_combinevector[j], l_pointvector[i], l_pointvector[j]);

                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    l_combinevector.erase(l_combinevector.begin() + j);
                    l_pointvector.erase(l_pointvector.begin() + j);
                    j = i;
                }

                else{}

            }


        }

    }

    void TCombine(const pharos_vlp_tilt::vector_perfect_array &top_input)  /// 중간
    {

        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        ///CALLBACK 함수
        pharos_vlp_tilt::vector_perfect_arrayPtr save_top(new pharos_vlp_tilt::vector_perfect_array);;

        *save_top = top_input;
        global_top = save_top;
        global_top->header.stamp = top_input.header.stamp;
        ///
        double ltime = global_top->header.stamp.toSec() - global_left->header.stamp.toSec();
        double rtime = global_top->header.stamp.toSec() - global_right->header.stamp.toSec();

        if(rtime > 0.5)
        {
            rtime = 1000;
        }

        if(ltime > 0.5)
        {
            ltime = 1000;
        }

        if(fabs(ltime) == 0 && fabs(rtime) == 0){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> left_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> right_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> l_c_vec;
            std::vector<CombineType> r_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_left->one.size(); i++)
            {
                ListSeqeunce(global_left->one[i],left_vec,l_c_vec,L_velodyne_x,L_velodyne_y,L_velodyne_z);
            }

            for (int i = 0; i < global_right->one.size(); i++)
            {
                ListSeqeunce(global_right->one[i],right_vec,r_c_vec,R_velodyne_x,R_velodyne_y,R_velodyne_z);
            }

//            //////////////////
//            pcl::PointCloud<pcl::PointXYZI>::Ptr test_min_max(new pcl::PointCloud<pcl::PointXYZI>);
//            //////////////////
//            std::vector<CombineType> vec_listed_combine;
//            vec_listed_combine.reserve(10000);
//
//            for (int m = 0; m <t_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(t_c_vec[m]);
//            }
//
//            for (int m = 0; m <l_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(l_c_vec[m]);
//            }
//
//            for (int m = 0; m <r_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(r_c_vec[m]);
//            }
//
//            for (int j = 0; j < vec_listed_combine.size(); j++)
//            {
//
//
//                double gap_std_x = (vec_listed_combine[j].max_object.x - vec_listed_combine[j].min_object.x) / 30;
//                double gap_std_y = (vec_listed_combine[j].max_object.y - vec_listed_combine[j].min_object.y) / 30;
//                double gap_std_z = (vec_listed_combine[j].max_object.z - vec_listed_combine[j].min_object.z) / 30;
//                double gap_x = (vec_listed_combine[j].max_object.x - vec_listed_combine[j].min_object.x);
//                double gap_y = (vec_listed_combine[j].max_object.y - vec_listed_combine[j].min_object.y);
//                double gap_z = (vec_listed_combine[j].max_object.z - vec_listed_combine[j].min_object.z);
//
//                for (int l = 0; l < 3; l++)
//                {
//                    for (int k = 0; k < 30; k++)
//                    {
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//                    }
//
//                    for (int k = 1; k < 30; k++)
//                    {
//
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//                    }
//
//                    for (int k = 1; k < 30; k++)
//                    {
//
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                    }
//                }
//            }
//
//            sensor_msgs::PointCloud2Ptr sensor_pointcloud_test(new sensor_msgs::PointCloud2);
//            pcl::toROSMsg(*test_min_max, * sensor_pointcloud_test);
//            sensor_pointcloud_test->header.frame_id = frame_combine;
//            sensor_pointcloud_test->header.stamp = global_top->header.stamp;
//            pub_test_min_max.publish(*sensor_pointcloud_test);

            CombineObject(left_vec,l_c_vec,right_vec,r_c_vec);
            CombineSelf(left_vec,l_c_vec);
            CombineObject(top_vec,t_c_vec,left_vec,l_c_vec);
            CombineObject(top_vec,t_c_vec,right_vec,r_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr l_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr r_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < left_vec.size() ; ++i){///left
                pharos_vlp_tilt::perfectarrayPtr mpa (new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = l_c_vec[i].center_x , c_pt.y = l_c_vec[i].center_y , c_pt.z = l_c_vec[i].center_z;
                l_vlp_center->push_back(c_pt);

                for(int j=0; j < left_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = left_vec[i]->points[j].x;
                    perfcc.point.y = left_vec[i]->points[j].y;
                    perfcc.point.z = left_vec[i]->points[j].z;
                    left_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = l_c_vec[i].min_object;
                mpa->max_object = l_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *left_vec[i];
            }

            for(int i = 0; i < right_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = r_c_vec[i].center_x , c_pt.y = r_c_vec[i].center_y , c_pt.z = r_c_vec[i].center_z;
                r_vlp_center->push_back(c_pt);


                for(int j=0; j < right_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = right_vec[i]->points[j].x;
                    perfcc.point.y = right_vec[i]->points[j].y;
                    perfcc.point.z = right_vec[i]->points[j].z;
                    right_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = r_c_vec[i].min_object;
                mpa->max_object = r_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *right_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ///////////////////////////10 26 new ////////////////////
//            sensor_msgs::PointCloud2Ptr T_Center (new sensor_msgs::PointCloud2);
//            sensor_msgs::PointCloud2Ptr L_Center (new sensor_msgs::PointCloud2);
//            sensor_msgs::PointCloud2Ptr R_Center (new sensor_msgs::PointCloud2);
//
//            pcl::toROSMsg(*t_vlp_center,*T_Center);
//            pcl::toROSMsg(*l_vlp_center,*L_Center);
//            pcl::toROSMsg(*r_vlp_center,*R_Center);
//            T_Center->header.frame_id = frame_combine;     T_Center->header.stamp = global_top->header.stamp;
//            L_Center->header.frame_id = frame_combine;     L_Center->header.stamp = global_left->header.stamp;
//            R_Center->header.frame_id = frame_combine;     R_Center->header.stamp = global_right->header.stamp;
//
//            pub_vlp_t_center.publish(*T_Center);
//            pub_vlp_l_center.publish(*L_Center);
//            pub_vlp_r_center.publish(*R_Center);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time : %lf", diff_value);
            }

        }

        else if(fabs(ltime) == 0 && rtime == 1000){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> left_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> l_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_left->one.size(); i++)
            {
                ListSeqeunce(global_left->one[i],left_vec,l_c_vec,L_velodyne_x,L_velodyne_y,L_velodyne_z);
            }

            CombineObject(top_vec,t_c_vec,left_vec,l_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr l_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < left_vec.size() ; ++i){///left
                pharos_vlp_tilt::perfectarrayPtr mpa (new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = l_c_vec[i].center_x , c_pt.y = l_c_vec[i].center_y , c_pt.z = l_c_vec[i].center_z;
                l_vlp_center->push_back(c_pt);

                for(int j=0; j < left_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = left_vec[i]->points[j].x;
                    perfcc.point.y = left_vec[i]->points[j].y;
                    perfcc.point.z = left_vec[i]->points[j].z;
                    left_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = l_c_vec[i].min_object;
                mpa->max_object = l_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *left_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TL : %lf", diff_value);
            }

        }

        else if(fabs(rtime) == 0 && ltime == 1000){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> right_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> r_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_right->one.size(); i++)
            {
                ListSeqeunce(global_right->one[i],right_vec,r_c_vec,R_velodyne_x,R_velodyne_y,R_velodyne_z);
            }

            CombineObject(top_vec,t_c_vec,right_vec,r_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr r_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < right_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = r_c_vec[i].center_x , c_pt.y = r_c_vec[i].center_y , c_pt.z = r_c_vec[i].center_z;
                r_vlp_center->push_back(c_pt);


                for(int j=0; j < right_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = right_vec[i]->points[j].x;
                    perfcc.point.y = right_vec[i]->points[j].y;
                    perfcc.point.z = right_vec[i]->points[j].z;
                    right_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = r_c_vec[i].min_object;
                mpa->max_object = r_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *right_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TR : %lf", diff_value);
            }

        }

        else if(rtime == 1000 && ltime == 1000)
        {
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;

            std::vector<CombineType> t_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TONLY : %lf", diff_value);
            }
        }

        else
        {

        }



    }


    void LCombine(const pharos_vlp_tilt::vector_perfect_array &left_input){
        ///CALLBACK 함수

        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        pharos_vlp_tilt::vector_perfect_arrayPtr save_left(new pharos_vlp_tilt::vector_perfect_array);;

        *save_left = left_input;
        global_left = save_left;
        global_left->header.stamp = left_input.header.stamp;
        ///
        double ltime = global_top->header.stamp.toSec() - global_left->header.stamp.toSec();
        double rtime = global_top->header.stamp.toSec() - global_right->header.stamp.toSec();

        if(rtime > 0.5)
        {
            rtime = 1000;
        }

        if(ltime > 0.5)
        {
            ltime = 1000;
        }

        if(fabs(ltime) == 0 && fabs(rtime) == 0){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> left_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> right_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> l_c_vec;
            std::vector<CombineType> r_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_left->one.size(); i++)
            {
                ListSeqeunce(global_left->one[i],left_vec,l_c_vec,L_velodyne_x,L_velodyne_y,L_velodyne_z);
            }

            for (int i = 0; i < global_right->one.size(); i++)
            {
                ListSeqeunce(global_right->one[i],right_vec,r_c_vec,R_velodyne_x,R_velodyne_y,R_velodyne_z);
            }

//            //////////////////
//            pcl::PointCloud<pcl::PointXYZI>::Ptr test_min_max(new pcl::PointCloud<pcl::PointXYZI>);
//            //////////////////
//            std::vector<CombineType> vec_listed_combine;
//            vec_listed_combine.reserve(10000);
//
//            for (int m = 0; m <t_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(t_c_vec[m]);
//            }
//
//            for (int m = 0; m <l_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(l_c_vec[m]);
//            }
//
//            for (int m = 0; m <r_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(r_c_vec[m]);
//            }
//
//            for (int j = 0; j < vec_listed_combine.size(); j++)
//            {
//
//
//                double gap_std_x = (vec_listed_combine[j].max_object.x - vec_listed_combine[j].min_object.x) / 30;
//                double gap_std_y = (vec_listed_combine[j].max_object.y - vec_listed_combine[j].min_object.y) / 30;
//                double gap_std_z = (vec_listed_combine[j].max_object.z - vec_listed_combine[j].min_object.z) / 30;
//                double gap_x = (vec_listed_combine[j].max_object.x - vec_listed_combine[j].min_object.x);
//                double gap_y = (vec_listed_combine[j].max_object.y - vec_listed_combine[j].min_object.y);
//                double gap_z = (vec_listed_combine[j].max_object.z - vec_listed_combine[j].min_object.z);
//
//                for (int l = 0; l < 3; l++)
//                {
//                    for (int k = 0; k < 30; k++)
//                    {
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//                    }
//
//                    for (int k = 1; k < 30; k++)
//                    {
//
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//                    }
//
//                    for (int k = 1; k < 30; k++)
//                    {
//
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                    }
//                }
//            }
//
//            sensor_msgs::PointCloud2Ptr sensor_pointcloud_test(new sensor_msgs::PointCloud2);
//            pcl::toROSMsg(*test_min_max, * sensor_pointcloud_test);
//            sensor_pointcloud_test->header.frame_id = frame_combine;
//            sensor_pointcloud_test->header.stamp = global_top->header.stamp;
//            pub_test_min_max.publish(*sensor_pointcloud_test);

            CombineObject(left_vec,l_c_vec,right_vec,r_c_vec);
            CombineSelf(left_vec,l_c_vec);
            CombineObject(top_vec,t_c_vec,left_vec,l_c_vec);
            CombineObject(top_vec,t_c_vec,right_vec,r_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr l_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr r_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < left_vec.size() ; ++i){///left
                pharos_vlp_tilt::perfectarrayPtr mpa (new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = l_c_vec[i].center_x , c_pt.y = l_c_vec[i].center_y , c_pt.z = l_c_vec[i].center_z;
                l_vlp_center->push_back(c_pt);

                for(int j=0; j < left_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = left_vec[i]->points[j].x;
                    perfcc.point.y = left_vec[i]->points[j].y;
                    perfcc.point.z = left_vec[i]->points[j].z;
                    left_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = l_c_vec[i].min_object;
                mpa->max_object = l_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *left_vec[i];
            }

            for(int i = 0; i < right_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = r_c_vec[i].center_x , c_pt.y = r_c_vec[i].center_y , c_pt.z = r_c_vec[i].center_z;
                r_vlp_center->push_back(c_pt);


                for(int j=0; j < right_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = right_vec[i]->points[j].x;
                    perfcc.point.y = right_vec[i]->points[j].y;
                    perfcc.point.z = right_vec[i]->points[j].z;
                    right_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = r_c_vec[i].min_object;
                mpa->max_object = r_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *right_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ///////////////////////////10 26 new ////////////////////
//            sensor_msgs::PointCloud2Ptr T_Center (new sensor_msgs::PointCloud2);
//            sensor_msgs::PointCloud2Ptr L_Center (new sensor_msgs::PointCloud2);
//            sensor_msgs::PointCloud2Ptr R_Center (new sensor_msgs::PointCloud2);
//
//            pcl::toROSMsg(*t_vlp_center,*T_Center);
//            pcl::toROSMsg(*l_vlp_center,*L_Center);
//            pcl::toROSMsg(*r_vlp_center,*R_Center);
//            T_Center->header.frame_id = frame_combine;     T_Center->header.stamp = global_top->header.stamp;
//            L_Center->header.frame_id = frame_combine;     L_Center->header.stamp = global_left->header.stamp;
//            R_Center->header.frame_id = frame_combine;     R_Center->header.stamp = global_right->header.stamp;
//
//            pub_vlp_t_center.publish(*T_Center);
//            pub_vlp_l_center.publish(*L_Center);
//            pub_vlp_r_center.publish(*R_Center);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time : %lf", diff_value);
            }

        }

        else if(fabs(ltime) == 0 && rtime == 1000){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> left_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> l_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_left->one.size(); i++)
            {
                ListSeqeunce(global_left->one[i],left_vec,l_c_vec,L_velodyne_x,L_velodyne_y,L_velodyne_z);
            }

            CombineObject(top_vec,t_c_vec,left_vec,l_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr l_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < left_vec.size() ; ++i){///left
                pharos_vlp_tilt::perfectarrayPtr mpa (new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = l_c_vec[i].center_x , c_pt.y = l_c_vec[i].center_y , c_pt.z = l_c_vec[i].center_z;
                l_vlp_center->push_back(c_pt);

                for(int j=0; j < left_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = left_vec[i]->points[j].x;
                    perfcc.point.y = left_vec[i]->points[j].y;
                    perfcc.point.z = left_vec[i]->points[j].z;
                    left_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = l_c_vec[i].min_object;
                mpa->max_object = l_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *left_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TL : %lf", diff_value);
            }

        }

        else if(fabs(rtime) == 0 && ltime == 1000){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> right_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> r_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_right->one.size(); i++)
            {
                ListSeqeunce(global_right->one[i],right_vec,r_c_vec,R_velodyne_x,R_velodyne_y,R_velodyne_z);
            }

            CombineObject(top_vec,t_c_vec,right_vec,r_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr r_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < right_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = r_c_vec[i].center_x , c_pt.y = r_c_vec[i].center_y , c_pt.z = r_c_vec[i].center_z;
                r_vlp_center->push_back(c_pt);


                for(int j=0; j < right_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = right_vec[i]->points[j].x;
                    perfcc.point.y = right_vec[i]->points[j].y;
                    perfcc.point.z = right_vec[i]->points[j].z;
                    right_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = r_c_vec[i].min_object;
                mpa->max_object = r_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *right_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TR : %lf", diff_value);
            }

        }

        else if(rtime == 1000 && ltime == 1000)
        {
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;

            std::vector<CombineType> t_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TONLY : %lf", diff_value);
            }
        }

        else
        {

        }



    }


    void RCombine(const pharos_vlp_tilt::vector_perfect_array &right_input)  /// 오른쪽
    {

        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        ///CALLBACK 함수
        pharos_vlp_tilt::vector_perfect_arrayPtr save_right(new pharos_vlp_tilt::vector_perfect_array);

        *save_right = right_input;
        global_right = save_right;
        global_right->header.stamp = right_input.header.stamp;
        ///
        double ltime = global_top->header.stamp.toSec() - global_left->header.stamp.toSec();
        double rtime = global_top->header.stamp.toSec() - global_right->header.stamp.toSec();

        if(rtime > 0.5)
        {
            rtime = 1000;
        }

        if(ltime > 0.5)
        {
            ltime = 1000;
        }

        if(fabs(ltime) == 0 && fabs(rtime) == 0){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> left_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> right_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> l_c_vec;
            std::vector<CombineType> r_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_left->one.size(); i++)
            {
                ListSeqeunce(global_left->one[i],left_vec,l_c_vec,L_velodyne_x,L_velodyne_y,L_velodyne_z);
            }

            for (int i = 0; i < global_right->one.size(); i++)
            {
                ListSeqeunce(global_right->one[i],right_vec,r_c_vec,R_velodyne_x,R_velodyne_y,R_velodyne_z);
            }

//            //////////////////
//            pcl::PointCloud<pcl::PointXYZI>::Ptr test_min_max(new pcl::PointCloud<pcl::PointXYZI>);
//            //////////////////
//            std::vector<CombineType> vec_listed_combine;
//            vec_listed_combine.reserve(10000);
//
//            for (int m = 0; m <t_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(t_c_vec[m]);
//            }
//
//            for (int m = 0; m <l_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(l_c_vec[m]);
//            }
//
//            for (int m = 0; m <r_c_vec.size(); m++)
//            {
//                vec_listed_combine.push_back(r_c_vec[m]);
//            }
//
//            for (int j = 0; j < vec_listed_combine.size(); j++)
//            {
//
//
//                double gap_std_x = (vec_listed_combine[j].max_object.x - vec_listed_combine[j].min_object.x) / 30;
//                double gap_std_y = (vec_listed_combine[j].max_object.y - vec_listed_combine[j].min_object.y) / 30;
//                double gap_std_z = (vec_listed_combine[j].max_object.z - vec_listed_combine[j].min_object.z) / 30;
//                double gap_x = (vec_listed_combine[j].max_object.x - vec_listed_combine[j].min_object.x);
//                double gap_y = (vec_listed_combine[j].max_object.y - vec_listed_combine[j].min_object.y);
//                double gap_z = (vec_listed_combine[j].max_object.z - vec_listed_combine[j].min_object.z);
//
//                for (int l = 0; l < 3; l++)
//                {
//                    for (int k = 0; k < 30; k++)
//                    {
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//                    }
//
//                    for (int k = 1; k < 30; k++)
//                    {
//
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                        pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                        test_min_max->push_back(pt);
//                    }
//
//                    for (int k = 1; k < 30; k++)
//                    {
//
//                        pcl::PointXYZI pt;
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                        pt.x = vec_listed_combine[j].max_object.x;
//                        pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                        pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                        test_min_max->push_back(pt);
//
//                    }
//                }
//            }
//
//            sensor_msgs::PointCloud2Ptr sensor_pointcloud_test(new sensor_msgs::PointCloud2);
//            pcl::toROSMsg(*test_min_max, * sensor_pointcloud_test);
//            sensor_pointcloud_test->header.frame_id = frame_combine;
//            sensor_pointcloud_test->header.stamp = global_top->header.stamp;
//            pub_test_min_max.publish(*sensor_pointcloud_test);

            CombineObject(left_vec,l_c_vec,right_vec,r_c_vec);
            CombineSelf(left_vec,l_c_vec);
            CombineObject(top_vec,t_c_vec,left_vec,l_c_vec);
            CombineObject(top_vec,t_c_vec,right_vec,r_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr l_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr r_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < left_vec.size() ; ++i){///left
                pharos_vlp_tilt::perfectarrayPtr mpa (new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = l_c_vec[i].center_x , c_pt.y = l_c_vec[i].center_y , c_pt.z = l_c_vec[i].center_z;
                l_vlp_center->push_back(c_pt);

                for(int j=0; j < left_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = left_vec[i]->points[j].x;
                    perfcc.point.y = left_vec[i]->points[j].y;
                    perfcc.point.z = left_vec[i]->points[j].z;
                    left_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = l_c_vec[i].min_object;
                mpa->max_object = l_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *left_vec[i];
            }

            for(int i = 0; i < right_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = r_c_vec[i].center_x , c_pt.y = r_c_vec[i].center_y , c_pt.z = r_c_vec[i].center_z;
                r_vlp_center->push_back(c_pt);


                for(int j=0; j < right_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = right_vec[i]->points[j].x;
                    perfcc.point.y = right_vec[i]->points[j].y;
                    perfcc.point.z = right_vec[i]->points[j].z;
                    right_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = r_c_vec[i].min_object;
                mpa->max_object = r_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *right_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ///////////////////////////10 26 new ////////////////////
//            sensor_msgs::PointCloud2Ptr T_Center (new sensor_msgs::PointCloud2);
//            sensor_msgs::PointCloud2Ptr L_Center (new sensor_msgs::PointCloud2);
//            sensor_msgs::PointCloud2Ptr R_Center (new sensor_msgs::PointCloud2);
//
//            pcl::toROSMsg(*t_vlp_center,*T_Center);
//            pcl::toROSMsg(*l_vlp_center,*L_Center);
//            pcl::toROSMsg(*r_vlp_center,*R_Center);
//            T_Center->header.frame_id = frame_combine;     T_Center->header.stamp = global_top->header.stamp;
//            L_Center->header.frame_id = frame_combine;     L_Center->header.stamp = global_left->header.stamp;
//            R_Center->header.frame_id = frame_combine;     R_Center->header.stamp = global_right->header.stamp;
//
//            pub_vlp_t_center.publish(*T_Center);
//            pub_vlp_l_center.publish(*L_Center);
//            pub_vlp_r_center.publish(*R_Center);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time : %lf", diff_value);
            }

        }

        else if(fabs(ltime) == 0 && rtime == 1000){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> left_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> l_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_left->one.size(); i++)
            {
                ListSeqeunce(global_left->one[i],left_vec,l_c_vec,L_velodyne_x,L_velodyne_y,L_velodyne_z);
            }

            CombineObject(top_vec,t_c_vec,left_vec,l_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr l_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < left_vec.size() ; ++i){///left
                pharos_vlp_tilt::perfectarrayPtr mpa (new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = l_c_vec[i].center_x , c_pt.y = l_c_vec[i].center_y , c_pt.z = l_c_vec[i].center_z;
                l_vlp_center->push_back(c_pt);

                for(int j=0; j < left_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = left_vec[i]->points[j].x;
                    perfcc.point.y = left_vec[i]->points[j].y;
                    perfcc.point.z = left_vec[i]->points[j].z;
                    left_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = l_c_vec[i].min_object;
                mpa->max_object = l_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *left_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TL : %lf", diff_value);
            }

        }

        else if(fabs(rtime) == 0 && ltime == 1000){

            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> right_vec;

            std::vector<CombineType> t_c_vec;
            std::vector<CombineType> r_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }

            for (int i = 0; i < global_right->one.size(); i++)
            {
                ListSeqeunce(global_right->one[i],right_vec,r_c_vec,R_velodyne_x,R_velodyne_y,R_velodyne_z);
            }

            CombineObject(top_vec,t_c_vec,right_vec,r_c_vec);
            CombineSelf(top_vec,t_c_vec);


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr r_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            for(int i = 0; i < right_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = r_c_vec[i].center_x , c_pt.y = r_c_vec[i].center_y , c_pt.z = r_c_vec[i].center_z;
                r_vlp_center->push_back(c_pt);


                for(int j=0; j < right_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = right_vec[i]->points[j].x;
                    perfcc.point.y = right_vec[i]->points[j].y;
                    perfcc.point.z = right_vec[i]->points[j].z;
                    right_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = r_c_vec[i].min_object;
                mpa->max_object = r_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *right_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TR : %lf", diff_value);
            }

        }

        else if(rtime == 1000 && ltime == 1000)
        {
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> top_vec;

            std::vector<CombineType> t_c_vec;

            /// conver odvec_infos;om
            for (int i = 0; i < global_top->one.size(); i++)
            {
                ListSeqeunce(global_top->one[i],top_vec,t_c_vec,T_velodyne_x,T_velodyne_y,T_velodyne_z);
            }


            int myintn=1;
            pcl::PointCloud<pcl::PointXYZI>::Ptr final_object_output (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////1026////////////////////////////////////////
            pcl::PointCloud<pcl::PointXYZI>::Ptr t_vlp_center (new pcl::PointCloud<pcl::PointXYZI>);
            ////////////////////////////////////////////////////////////////
            pharos_vlp_tilt::vector_perfect_arrayPtr my_vec(new pharos_vlp_tilt::vector_perfect_array);


            for(int i = 0; i < top_vec.size(); ++i){///right
                pharos_vlp_tilt::perfectarrayPtr mpa(new pharos_vlp_tilt::perfectarray);
                pcl::PointXYZI c_pt;
                c_pt.x = t_c_vec[i].center_x , c_pt.y = t_c_vec[i].center_y , c_pt.z = t_c_vec[i].center_z;
                t_vlp_center->push_back(c_pt);


                for(int j=0; j < top_vec[i]->points.size(); ++j)
                {

                    perfcc.point.x = top_vec[i]->points[j].x;
                    perfcc.point.y = top_vec[i]->points[j].y;
                    perfcc.point.z = top_vec[i]->points[j].z;
                    top_vec[i]->points[j].intensity = myintn;
                    mpa->objects.push_back(perfcc);

                }
                mpa->min_object = t_c_vec[i].min_object;
                mpa->max_object = t_c_vec[i].max_object;
                my_vec->one.push_back(*mpa);

                myintn += 15;
                if(myintn > 250)
                {
                    myintn = 10;
                }
                *final_object_output += *top_vec[i];
            }

            sensor_msgs::PointCloud2 final_output;
            pcl::toROSMsg(*final_object_output , final_output);
            final_output.header.frame_id = frame_combine;
            final_output.header.stamp = global_top->header.stamp;
            pub_combine_pointcloud.publish(final_output);

            my_vec->header.frame_id = frame_combine;
            my_vec->header.stamp =  global_top->header.stamp;
            pub_my_final_output.publish(*my_vec);

            ros::Time last = ros::Time::now();

            diff_value = last.toSec() - begin.toSec();

            if(isTimeCombine == true)
            {
                ROS_INFO("combine time_TONLY : %lf", diff_value);
            }
        }

        else
        {

        }



    }




protected:
    ros::Subscriber sub_top;
    ros::Subscriber sub_left;
    ros::Subscriber sub_right;
    ros::Subscriber sub_ekf;

    ros::Publisher pub_combine_pointcloud;
    ros::Publisher pub_my_final_output;

    ///////1026///////
    ros::Publisher pub_top_xyz;
    ros::Publisher pub_left_xyz;
    ros::Publisher pub_right_xyz;
    ros::Publisher pub_vlp_t_center;
    ros::Publisher pub_vlp_l_center;
    ros::Publisher pub_vlp_r_center;


    ros::Publisher pub_test_min_max;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_LRT_Combine");       //노드명 초기화



    ROS_INFO("started vlpt_LRT_Combine");
//    ROS_INFO("SUBTOPIC : ");
//    ROS_INFO("PUBTOPIC : ");

    vlp_ hello;

    ros::spin();


}
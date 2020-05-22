

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pharos_vlp_tilt/perfectarray.h>
#include <iostream>
///전역변수///
float L_velodyne_x , L_velodyne_y , L_velodyne_z;

double diff_value;
bool isTimeLeft;
bool isMapInit = false;
std::vector<int> MapInitVector;

std::string map_topic;
std::string odom_topic;
std::string sub_filter;
std::string pub_filter;
std::string pub_filter_cloud;
std::string frame_filter;

///------////
nav_msgs::Odometry Odom_;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
nav_msgs::OccupancyGridPtr drivable_map (new nav_msgs::OccupancyGrid);






class vlp_
{
public:

    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;

    ros::Subscriber sub_topic;
    ros::Subscriber sub_map_topic;
    ros::Subscriber sub_odom_topic;
    ros::Publisher pub_map_filtered;

    vlp_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        nh->param<bool>("isTimeLeft",isTimeLeft, false);

        pnh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_x",L_velodyne_x, 0);
        pnh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_y",L_velodyne_y, 0);
        pnh->param<float>("/pharos_tf_broadcaster_node/L_velodyne_z",L_velodyne_z, 0);
        pnh->param<std::string>("odom_topic",odom_topic,"/odom/ekf");
        pnh->param<std::string>("map_topic",map_topic,"/odom/ekf");
        pnh->param<std::string>("sub_filter",sub_filter,"/odom/ekf");
        pnh->param<std::string>("pub_filter",pub_filter,"/odom/ekf");
        pnh->param<std::string>("pub_filter_cloud",pub_filter_cloud,"/odom/ekf");

        pnh->param<std::string>("frame_filter",frame_filter, "mid_velodyne2");


        sub_topic = nh->subscribe(sub_filter, 10 , &vlp_::LeftMapFiter,this);
        sub_map_topic = nh->subscribe(map_topic , 10 , &vlp_::MapCB,this);
        sub_odom_topic = nh->subscribe(odom_topic, 10 , &vlp_::tracking_odomCB,this);

        pub_map_filtered = nh->advertise<pharos_vlp_tilt::perfectarray>(pub_filter,10);



        test_pub = nh->advertise<sensor_msgs::PointCloud2>(pub_filter_cloud,10);
    }

    void tracking_odomCB(const nav_msgs::OdometryPtr& msg) //subscribe odom
    {

        //        std::cout<< "oooooooooooooooooo" <<std::endl;
        Odom_ = *msg;
        // std::cout<<"listen"<<std::endl;
    }

    void MapCB(const nav_msgs::OccupancyGridPtr& map)
    { /// const빼주니까 grid ptr사이
        isMapInit = true;
        drivable_map = map;

    }

    void LeftMapFiter(const pharos_vlp_tilt::perfectarray input)
    {///붙여지낚. 애매함

        if(!isMapInit)
        {
            return;
        }

        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        ///CALLBACK 함수
        pharos_vlp_tilt::perfectarrayPtr odomed_data(new pharos_vlp_tilt::perfectarray);
        pharos_vlp_tilt::perfectarrayPtr filtered_data(new pharos_vlp_tilt::perfectarray);
        *odomed_data = input;


        Eigen::Quaternionf q;
        q.x() = Odom_.pose.pose.orientation.x;
        q.y() = Odom_.pose.pose.orientation.y;
        q.z() = Odom_.pose.pose.orientation.z;
        q.w() = Odom_.pose.pose.orientation.w;
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);

        Eigen::AngleAxisf init_rotation02_x(euler[0], Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf init_rotation02_y(euler[1], Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf init_rotation02_z(euler[2], Eigen::Vector3f::UnitZ()); // imu <-> odom


        Eigen::Translation3f init_translation02((Odom_.pose.pose.position.x),
                                                (Odom_.pose.pose.position.y),
                                                (Odom_.pose.pose.position.z));


        Eigen::Matrix4f RT02 = (init_translation02 * init_rotation02_z * init_rotation02_y *
                                init_rotation02_x).matrix();

//        std::cout<<"odom"<< Odom_.pose.pose.position.x << Odom_.pose.pose.position.y << Odom_.pose.pose.orientation.z<<std::endl;

        //// odom 화


        double resolutionInverse = 1 / drivable_map->info.resolution;


        for (unsigned int i = 0; i < input.objects.size(); i++) {
            odomed_data->objects[i].point.x += L_velodyne_x;
            odomed_data->objects[i].point.y += L_velodyne_y;
            odomed_data->objects[i].point.z += L_velodyne_z;

            ///여기까지는 odomed_data는 차 중심으로 좌표변환이 되는것이다.

            Eigen::RowVector4f odom_pose, point_xyz;

//
            point_xyz(0) = odomed_data->objects[i].point.x;
            point_xyz(1) = odomed_data->objects[i].point.y;
            point_xyz(2) = odomed_data->objects[i].point.z;
            point_xyz(3) = 1;

            odom_pose.transpose() = RT02 * point_xyz.transpose();
//                    std::cout<<"odom_pose?"<<odom_pose(0);
            odomed_data->objects[i].point.x = odom_pose(0);
            odomed_data->objects[i].point.y = odom_pose(1);
            odomed_data->objects[i].point.z = odom_pose(2);
//                    std::cout<<"i와 라오라오랑ㄹj는"<<i<<"         "<<j<<std::endl;


//
//            std::cout<<odomed_data->objects[i].point.x <<"와" <<filtered_data->objects[i].point.x <<std::endl;
//            std::cout<<odomed_data->objects[i].point.y <<"와" <<filtered_data->objects[i].point.y <<std::endl;
//            std::cout<<odomed_data->objects[i].point.z <<"와" <<filtered_data->objects[i].point.z <<std::endl;
            if (drivable_map->data.size() != 0){ /// map data가 안들어와 있을때)

                int xIndex, yIndex;
                xIndex = (int) (
                        (odomed_data->objects[i].point.x -
                         drivable_map->info.origin.position.x) *
                        resolutionInverse);
                yIndex = (int) (
                        (odomed_data->objects[i].point.y -
                         drivable_map->info.origin.position.y) *
                        resolutionInverse);

                int mapIndex = MAP_IDX(drivable_map->info.width, xIndex, yIndex);
//
                if(mapIndex < 0)
                {
                    ROS_ERROR("Here is Out of map!!!!! Maybe CoreDump!!!!");
                    ROS_ERROR("So MapIndex is Negative!!! Restarting NOW!!!!!!!");
                }

                int mapdata = drivable_map->data[mapIndex];


//                std::cout << "mapdata : "<<mapdata<<std::endl;

//
                if (mapdata == 0) { /// 맵(도로)이랑 일치한 점이 하나라도 나오는 지점.
                    filtered_data->objects.push_back(input.objects[i]);


                } else {///보지 않아도 되는 것일때

                    ///모든점들이 여기에 해당되어야 지워지는 것.
                }
            }

            else if(drivable_map->data.size() == 0 && i == 0)
            {
//                if(MapInitVector.size() > 10)
//                {
//                    ROS_ERROR("Map have no data RRR!!!!!!!!!!!!!!!");
//                }
//
//                else if(!isMapInit)
//                {
//                    isMapInit = true;
//
//                    if(MapInitVector.size() > 15)
//                    {
//                        MapInitVector.erase(MapInitVector.begin());
//                    }
//
//                    MapInitVector.push_back(isMapInit);
//
//
//                    isMapInit = false;
//                }
                ROS_ERROR("Map have no data LLL!!!!!!!!!!!!!!!");

            }
        }


        pcl::PointCloud<pcl::PointXYZI>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        for(int i=0; i < filtered_data->objects.size(); i++) {
            pcl::PointXYZI pt;
            pt.x = filtered_data->objects[i].point.x;
            pt.y = filtered_data->objects[i].point.y;
            pt.z = filtered_data->objects[i].point.z;
            test_cloud->points.push_back(pt);
        }

//        pcl::PointCloud<pcl::PointXYZI>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZI>);
//        for(int i=0; i < odomed_data->objects.size(); i++) {
//            pcl::PointXYZI pt;
//            pt.x = odomed_data->objects[i].point.x;
//            pt.y = odomed_data->objects[i].point.y;
//            pt.z = odomed_data->objects[i].point.z;
//            test_cloud->points.push_back(pt);
//        }

        sensor_msgs::PointCloud2 test_cp;
        pcl::toROSMsg(*test_cloud,test_cp);
        test_cp.header.frame_id=frame_filter;
//        test_cp.header.frame_id="odom";
        test_pub.publish(test_cp);

        filtered_data->header.frame_id = frame_filter;      filtered_data->header.stamp = input.header.stamp;
        pub_map_filtered.publish(*filtered_data);

        ros::Time last = ros::Time::now();


        diff_value = last.toSec() - begin.toSec();
        if(isTimeLeft == true)
        {
            ROS_INFO("filter_L time : %lf", diff_value);
        }
    }

public:
    ros::Publisher test_pub;
};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_filter_L");       //노드명 초기화

    ROS_INFO("started vlpt_filter_L");

    vlp_ hello;

    ros::spin();


}
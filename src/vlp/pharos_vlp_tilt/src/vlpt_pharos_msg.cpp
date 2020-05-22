

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pharos_msgs/ObjectInfoArray.h>
#include <pharos_vlp_tilt/vector_perfect_array.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <nav_msgs/Odometry.h>
///전역변수///
double SET_UNDER_Z;
int remove_point_size;
double Set_bong_distance;
double Bigvolum;
int map_size;
double Set_Perception_Distance; /// 가까운거리 봉 없앨때 쓴다.
double XY_AREA; /// 가까운거리 봉 없앨때 쓴다.
double minZThreshold;

double diff_value;
bool isTimeCombine;


///------////
std::string frame_pharos;
std::string map_topic_name;
std::string odom_topic_name;
std::string sub_pharos;
std::string pub_pharos_senosorfusion;
std::string pub_pharos_pathplanning_cloud;

nav_msgs::Odometry Odom_;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
nav_msgs::OccupancyGridPtr drivable_map (new nav_msgs::OccupancyGrid);


class vlp_
{
public:

    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;


    vlp_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        nh->param<double>("Set_Perception_Distance",Set_Perception_Distance,12);
        nh->param<double>("XY_AREA",XY_AREA,0.6*0.6);
        nh->param<double>("SET_UNDER_Z",SET_UNDER_Z, 0.5);
        nh->param<int>("remove_point_size",remove_point_size, 20);
        nh->param<double>("minZThreshold",minZThreshold, 20);
        nh->param<double>("Set_bong_distance",Set_bong_distance, 0.3);
        nh->param<double>("Bigvolum",Bigvolum, 50);
        nh->param<int>("map_size",map_size, 100);
        nh->param<bool>("isTimeCombine",isTimeCombine, false);

        pnh->param<std::string>("frame_pharos",frame_pharos, "odom");
        pnh->param<std::string>("odom_topic_name",odom_topic_name, "/odom/novatel");
        pnh->param<std::string>("map_topic_name",map_topic_name , "drivable_map_perception_map");
        pnh->param<std::string>("sub_pharos",sub_pharos , "drivable_map_perception_map");
        pnh->param<std::string>("pub_pharos_senosorfusion",pub_pharos_senosorfusion , "drivable_map_perception_map");
        pnh->param<std::string>("pub_pharos_pathplanning_cloud",pub_pharos_pathplanning_cloud , "drivable_map_perception_map");


        sub_all = nh->subscribe(sub_pharos, 10 , & vlp_::AllInformation,this);
        sub_map = nh->subscribe(map_topic_name, 1, & vlp_::MapCB, this);
        Odom_sub = nh->subscribe(odom_topic_name, 10, &vlp_::tracking_odomCB, this);

        pub_pharos_msgs = nh->advertise<pharos_msgs::ObjectInfoArray>(pub_pharos_senosorfusion,10);
        pub_under_z_object_pointcloud = nh->advertise<sensor_msgs::PointCloud2>(pub_pharos_pathplanning_cloud,10);
//        pub_test = nh->advertise<sensor_msgs::PointCloud2>("square",10);

    }


    void tracking_odomCB(const nav_msgs::OdometryPtr& msg) //subscribe odom
    {

        Odom_ = *msg;

    }

    void MapCB(const nav_msgs::OccupancyGridPtr& map) { /// const빼주니까 grid ptr사이
        drivable_map = map;

    }

    double DistnaceXY(pharos_vlp_tilt::perfect input , pharos_vlp_tilt::perfect another )
    {
        double L;
        L= sqrt( (input.point.x - another.point.x)*(input.point.x - another.point.x) + (input.point.y - another.point.y)*(input.point.y - another.point.y) );
        return L;

    }

    double Distnaceself(pharos_msgs::ObjectSize input)
    {
        double L;
        L= sqrt( (input.max_x - input.min_x)*(input.max_x - input.min_x) + (input.max_y - input.min_y)*(input.max_y - input.min_y) );
        return L;

    }

    void AllInformation(const pharos_vlp_tilt::vector_perfect_array raw_input) {
        ///CALLBACK 함수

        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        pharos_vlp_tilt::vector_perfect_arrayPtr transformed_array(new pharos_vlp_tilt::vector_perfect_array);
        *transformed_array = raw_input;
        pharos_vlp_tilt::vector_perfect_arrayPtr base_novatel_array(new pharos_vlp_tilt::vector_perfect_array);
        *base_novatel_array = raw_input;

        pharos_msgs::ObjectInfoArrayPtr allobjects(new pharos_msgs::ObjectInfoArray);  /// ---> pharos_msgs 최종 아웃풋

        //////////////////
        pcl::PointCloud<pcl::PointXYZI>::Ptr test_min_max(new pcl::PointCloud<pcl::PointXYZI>);
        //////////////////



        //// odom 화
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

         /// odom 좌표계

        double resolutionInverse = 1 / drivable_map->info.resolution;
//        std::cout <<"resoult///////////////////////////////////////////////////////////////////////////////////////////////"<<resolutionInverse<<std::endl;
//        std::cout<<drivable_map->data.size()<<std::endl;
//            std::cout<<"111"<<std::endl;
        for (int i = 0; i < transformed_array->one.size(); i++) {
            ///trandfor 좌표 변환된 것들
//            std::cout<<"뭐야 왜 안들어가져?"<<std::endl;
//            int aliveordie = 0;
//            int check_map_size = 0;   ///map과 비교해서 일정갯수 이상만 뽑아 내게 한다.

            for (int j = 0; j < transformed_array->one[i].objects.size(); j++) {
                /// 점을 오돔좌표계로 바꾼다.
//                    std::cout<<"여기서조차 나오면 안된다."<<transformed_array->one[i].objects.size()<<std::endl;
                Eigen::RowVector4f odom_pose, point_xyz;

//                std::cout<<"이전값"<< transformed_array->one[i].objects[j].point.x<< transformed_array->one[i].objects[j].point.y<< transformed_array->one[i].objects[j].point.z<<std::endl;
                point_xyz(0) = transformed_array->one[i].objects[j].point.x;
                point_xyz(1) = transformed_array->one[i].objects[j].point.y;
                point_xyz(2) = transformed_array->one[i].objects[j].point.z;
                point_xyz(3) = 1;

                odom_pose.transpose() = RT02 * point_xyz.transpose();
//                    std::cout<<"odom_pose?"<<odom_pose(0);
                transformed_array->one[i].objects[j].point.x = odom_pose(0);
                transformed_array->one[i].objects[j].point.y = odom_pose(1);
                transformed_array->one[i].objects[j].point.z = odom_pose(2);
//                    std::cout<<"i와 라오라오랑ㄹj는"<<i<<"         "<<j<<std::endl;
//                if (drivable_map->data.size() != 0) { /// map data가 안들어와 있을때)
//
//                    int xIndex, yIndex;
//                    xIndex = (int) (
//                            (transformed_array->one[i].objects[j].point.x -
//                             drivable_map->info.origin.position.x) *
//                            resolutionInverse);
//                    yIndex = (int) (
//                            (transformed_array->one[i].objects[j].point.y -
//                             drivable_map->info.origin.position.y) *
//                            resolutionInverse);
//
//                    int mapIndex = MAP_IDX(drivable_map->info.width, xIndex, yIndex);
////
//
//                    int mapdata = drivable_map->data[mapIndex];
//
//
////                            std::cout<<"mapdata값은 = "<<mapdata<<std::endl;
//
//
//                    if (mapdata == 0) { /// 맵(도로)이랑 일치한 점이 하나라도 나오는 지점.
//                        aliveordie = 1;
//                        check_map_size++;
//                        continue;
//                        /// 여길 거쳐서 나오는 것들이 안없어지게 되는 것이다.
//
//                    } else {///보지 않아도 되는 것일때
//
//                        ///모든점들이 여기에 해당되어야 지워지는 것.
//                    }
//                }
            }


//            if (aliveordie == 0) { /// 전부다 나오는 것들..
//                //// 뽑아 내줘야 하는 데이터들
//                /// 두번재 포문 나가면 된다. 지울 필요도 없는것이다.
//                transformed_array->one.erase(transformed_array->one.begin() + i);
//                base_novatel_array->one.erase(base_novatel_array->one.begin() + i); /// 값들은 전부다 novatel 기분이다.
//                i += -1; /// 삭제 한 위치에서 한번 보는것.
//            } else if (aliveordie == 1 && check_map_size == transformed_array->one[i].objects.size()) {}
//
//            else if (aliveordie == 1 && check_map_size <= map_size) {/// 걸쳐있고 넘어간 포인터 숫자가 미만일때..
//                transformed_array->one.erase(transformed_array->one.begin() + i);
//                base_novatel_array->one.erase(base_novatel_array->one.begin() + i); /// 값들은 전부다 novatel 기분이다.
//                i += -1; /// 삭제 한 위치에서 한번 보는것.
//
//            } else {}

        }

        pharos_vlp_tilt::vector_perfect_arrayPtr input(new pharos_vlp_tilt::vector_perfect_array);
        input = transformed_array;

        pcl::PointCloud<pcl::PointXYZI>::Ptr path_planning_point_cloud (new pcl::PointCloud<pcl::PointXYZI>); /// path planning output

        int intensityyy = 10;

        for (int i = 0; i < input->one.size(); i++)
        {
            // std::cout << input->one[i].objects.size() << "object_point_size" << std::endl;


            double object_cx(0), object_cy(0), object_cz(0);   /// @won for center value


            double max_x(input->one[i].objects[0].point.x), min_x(input->one[i].objects[0].point.x),
                    max_y(input->one[i].objects[0].point.y), min_y(input->one[i].objects[0].point.y),
                    max_z(input->one[i].objects[0].point.z), min_z(input->one[i].objects[0].point.z);

            pharos_msgs::ObjectInfoPtr OneObject_Pharos_info (new pharos_msgs::ObjectInfo);                         /// @won for pharos_msg

            intensityyy += 20;

            if(intensityyy >= 250)
                intensityyy = 10;

            int check_bong = 1;  /// 최정적으로 1이 되면 봉을 뽑으면 된다. 그렇지 않다면 없애는것...

            ///뽕 없애는거
            if (input->one[i].objects.size() <= remove_point_size) /// 봉같은 것들 사이즈가 작을경우 해당 함수를 놀리는 것.
            {

                for (int bong = 0; bong < input->one[i].objects.size(); bong++) {
                    check_bong = (DistnaceXY(input->one[i].objects[0], input->one[i].objects[bong]) <=
                                  Set_bong_distance) ? 0 : 1;
                    if (check_bong == 0 )
                        continue;
                        /// 계속 점이 일정 범위안에 존재한다면 계속 0을 받게 된다.

                    else
                        break;
                }

                if (check_bong == 0)
                    continue;

            }

            for (int j = 0; j < input->one[i].objects.size(); j++)
            {
                ///일단 매점 더해준다. 이 for문 밖에 나눠주기.
                OneObject_Pharos_info->pose.x += input->one[i].objects[j].point.x;
                OneObject_Pharos_info->pose.y += input->one[i].objects[j].point.y;
                OneObject_Pharos_info->pose.z += input->one[i].objects[j].point.z;

                /// 최대값과 최솟값을 넣어준다.
                max_x = (max_x >= input->one[i].objects[j].point.x)? max_x : input->one[i].objects[j].point.x;
                min_x = (min_x <= input->one[i].objects[j].point.x)? min_x : input->one[i].objects[j].point.x;

                max_y = (max_y >= input->one[i].objects[j].point.y)? max_y : input->one[i].objects[j].point.y;
                min_y = (min_y <= input->one[i].objects[j].point.y)? min_y : input->one[i].objects[j].point.y;

                max_z = (max_z >= input->one[i].objects[j].point.z)? max_z : input->one[i].objects[j].point.z;
                min_z = (min_z <= input->one[i].objects[j].point.z)? min_z : input->one[i].objects[j].point.z;
                ///////////////////////////



//                pcl::PointXYZI pt;
//                if(input->one[i].max_object.z - input->one[i].min_object.z < 0.15)
//                {}
//                else
//                {
//                    pt.x = input->one[i].objects[j].point.x;
//                    pt.y = input->one[i].objects[j].point.y;
//                    pt.z = input->one[i].objects[j].point.z;
//                    pt.intensity = intensityyy;
//
//                    if(pt.z<SET_UNDER_Z)
//                    {
//                        path_planning_point_cloud->points.push_back(pt);
//                    }
//                }
            }/// 객채i의 포인터들 다룰때

            OneObject_Pharos_info->pose.x = OneObject_Pharos_info->pose.x/input->one[i].objects.size();
            OneObject_Pharos_info->pose.y = OneObject_Pharos_info->pose.y/input->one[i].objects.size();
            OneObject_Pharos_info->pose.z = OneObject_Pharos_info->pose.z/input->one[i].objects.size();

            OneObject_Pharos_info->size.min_x = min_x;
            OneObject_Pharos_info->size.max_x = max_x;
            OneObject_Pharos_info->size.min_y = min_y;
            OneObject_Pharos_info->size.max_y = max_y;
            OneObject_Pharos_info->size.min_z = min_z;
            OneObject_Pharos_info->size.max_z = max_z;

            OneObject_Pharos_info->collision.min_x = min_x;
            OneObject_Pharos_info->collision.max_x = max_x;
            OneObject_Pharos_info->collision.min_y = min_y;
            OneObject_Pharos_info->collision.max_y = max_y;
            OneObject_Pharos_info->collision.min_z = min_z;
            OneObject_Pharos_info->collision.max_z = max_z;


            ////여기가 파로스 메세지 뽑아주는 것이니까 조건을 여기다 주면 된다.
            double Center_L = sqrt( OneObject_Pharos_info->pose.x*OneObject_Pharos_info->pose.x + OneObject_Pharos_info->pose.y*OneObject_Pharos_info->pose.y );
            if(Center_L <= Set_Perception_Distance) /// 거리가 거의 12m일때.
            {
                /*if( (OneObject_Pharos_info->size.max_x-OneObject_Pharos_info->size.min_x)*(OneObject_Pharos_info->size.max_y-OneObject_Pharos_info->size.min_y)*0.44 <= XY_AREA ){/// 면적이 작을때..

                }

                else */if((OneObject_Pharos_info->size.max_x-OneObject_Pharos_info->size.min_x)*(OneObject_Pharos_info->size.max_y-OneObject_Pharos_info->size.min_y)
                        *(OneObject_Pharos_info->size.max_z-OneObject_Pharos_info->size.min_z) <= Bigvolum)
                {
                    // for (int j = 0; j < input->one[i].objects.size(); j++)
                    // {
                    //     pcl::PointXYZI pt;
                    //     pt.x = input->one[i].objects[j].point.x;
                    //     pt.y = input->one[i].objects[j].point.y;
                    //     pt.z = input->one[i].objects[j].point.z;
                    //     pt.intensity = intensityyy;

                    //     if (pt.z < Odom_.pose.pose.position.z + SET_UNDER_Z)
                    //     {
                    //         path_planning_point_cloud->points.push_back(pt);
                    //     }
                    // }
                }

                else if(OneObject_Pharos_info->size.min_z > Odom_.pose.pose.position.z + minZThreshold)
                {

                }

                else if(OneObject_Pharos_info->size.max_z - OneObject_Pharos_info->size.min_z < 0.15)
                {

                }

                else if(OneObject_Pharos_info->size.max_z < Odom_.pose.pose.position.z + 0.1)
                {

                }

                else if(Distnaceself(OneObject_Pharos_info->size) > 5 && OneObject_Pharos_info->size.max_z - OneObject_Pharos_info->size.min_z < 0.5)
                {

                }

                else
                {
                    for (int j = 0; j < input->one[i].objects.size(); j++)
                    {
                        pcl::PointXYZI pt;
                        pt.x = input->one[i].objects[j].point.x;
                        pt.y = input->one[i].objects[j].point.y;
                        pt.z = input->one[i].objects[j].point.z;
                        pt.intensity = intensityyy;

                        if (pt.z < Odom_.pose.pose.position.z + SET_UNDER_Z)
                        {
                            path_planning_point_cloud->points.push_back(pt);
                        }
                    }

                    if(OneObject_Pharos_info->pose.z < Odom_.pose.pose.position.z + SET_UNDER_Z)
                    allobjects->objects.push_back(*OneObject_Pharos_info);

                }
            }

            else
            {
                if((OneObject_Pharos_info->size.max_x-OneObject_Pharos_info->size.min_x)*(OneObject_Pharos_info->size.max_y-OneObject_Pharos_info->size.min_y)
                   *(OneObject_Pharos_info->size.max_z-OneObject_Pharos_info->size.min_z) <= Bigvolum )/// 거리가 12m밖이고 면적이
                {
                    // for (int j = 0; j < input->one[i].objects.size(); j++)
                    // {
                    //     pcl::PointXYZI pt;
                    //     pt.x = input->one[i].objects[j].point.x;
                    //     pt.y = input->one[i].objects[j].point.y;
                    //     pt.z = input->one[i].objects[j].point.z;
                    //     pt.intensity = intensityyy;

                    //     if (pt.z < Odom_.pose.pose.position.z + SET_UNDER_Z)
                    //     {
                    //         path_planning_point_cloud->points.push_back(pt);
                    //     }
                    // }

                }

                else if(OneObject_Pharos_info->size.min_z > Odom_.pose.pose.position.z + minZThreshold)
                {

                }

                else if(OneObject_Pharos_info->size.max_z - OneObject_Pharos_info->size.min_z < 0.15)
                {

                }

                else if(OneObject_Pharos_info->size.max_z < Odom_.pose.pose.position.z + 0.1)
                {

                }

                else if(Distnaceself(OneObject_Pharos_info->size) > 5 && OneObject_Pharos_info->size.max_z - OneObject_Pharos_info->size.min_z < 0.5)
                {

                }

                else
                {
                    for (int j = 0; j < input->one[i].objects.size(); j++)
                    {
                        pcl::PointXYZI pt;
                        pt.x = input->one[i].objects[j].point.x;
                        pt.y = input->one[i].objects[j].point.y;
                        pt.z = input->one[i].objects[j].point.z;
                        pt.intensity = intensityyy;

                        if (pt.z < Odom_.pose.pose.position.z + SET_UNDER_Z)
                        {
                            path_planning_point_cloud->points.push_back(pt);
                        }
                    }

                    if(OneObject_Pharos_info->pose.z < Odom_.pose.pose.position.z + SET_UNDER_Z)
                    allobjects->objects.push_back(*OneObject_Pharos_info);
                }
            }


        }///객채 다룰때

        ////////////
        // if(allobjects->objects.size() == 0 )
        // {
        //     pharos_msgs::ObjectInfo inf;
        //     inf.pose.x = 10000;
        //     inf.pose.y = 10000;
        //     allobjects->objects.push_back(inf);
        // }
        /// output under_z_msg (path planning)

        sensor_msgs::PointCloud2Ptr sensor_pointcloud_path(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*path_planning_point_cloud, * sensor_pointcloud_path);
        sensor_pointcloud_path->header.frame_id = frame_pharos;
        sensor_pointcloud_path->header.stamp = raw_input.header.stamp;
        pub_under_z_object_pointcloud.publish(*sensor_pointcloud_path);

//        sensor_msgs::PointCloud2Ptr sensor_pointcloud_test(new sensor_msgs::PointCloud2);
//        pcl::toROSMsg(*test_min_max, * sensor_pointcloud_test);
//        sensor_pointcloud_test->header.frame_id = frame_pharos;
//        sensor_pointcloud_test->header.stamp = raw_input.header.stamp;
//        pub_test.publish(*sensor_pointcloud_test);


        /// output pharos_msgs

        allobjects->header.frame_id = frame_pharos;
        allobjects->header.stamp = raw_input.header.stamp;
        pub_pharos_msgs.publish(*allobjects);

        ros::Time last = ros::Time::now();


        diff_value = last.toSec() - begin.toSec();

        if(isTimeCombine == true)
        {
            ROS_INFO("pharos_msg time : %lf", diff_value);
        }

    }




protected:
    ros::Subscriber sub_all;
    ros::Subscriber sub_map;
    ros::Subscriber Odom_sub;

    ros::Publisher pub_under_z_object_pointcloud;
    ros::Publisher pub_pharos_msgs;
    ////////////////
    ros::Publisher pub_test;
    ///////////////


};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_pharos_msg");       //노드명 초기화



    ROS_INFO("started vlpt_pharos_msg");
//    ROS_INFO("SUBTOPIC : ");
//    ROS_INFO("PUBTOPIC : ");

    vlp_ hello;

    ros::spin();


}

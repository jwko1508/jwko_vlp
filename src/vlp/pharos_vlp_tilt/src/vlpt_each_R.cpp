

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pharos_vlp_tilt/vector_perfect_array.h>
#include <geometry_msgs/PoseArray.h>


///전역변수///
int myIntensity = 10;
double CenterThreshold;
double LRThreshold;
double XYTHRESHOLD;

double diff_value;
bool isTimeRight;


std::string frame_each;
std::string pub_each;
std::string pub_each_cloud;
std::string sub_each;

///------////
struct CombineType
{
    /// Center
    double center_x;
    double center_y;
    double center_z;

    pharos_vlp_tilt::point min_object;
    pharos_vlp_tilt::point max_object;

    int min_hori;
    int max_hori;
    pharos_vlp_tilt::center_position min_center;
    pharos_vlp_tilt::center_position max_center;
};



struct InfoMation
{
    unsigned int hori = 0;
    unsigned int i = 0;
};

class vlp_
{
public:

    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;

    vlp_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        nh->param<double>("CenterThreshold",CenterThreshold, 1.5);
        nh->param<double>("LRThreshold",LRThreshold, 1.0);
        nh->param<double>("XYTHRESHOLD",XYTHRESHOLD, 0.3);
        nh->param<bool>("isTimeRight",isTimeRight, false);

        pnh->param<std::string>("frame_each",frame_each, "left_velodyne2");
        pnh->param<std::string>("pub_each",pub_each, "new_left_combine");
        pnh->param<std::string>("pub_each_cloud",pub_each_cloud, "new_left_combine");
        pnh->param<std::string>("sub_each",sub_each, "my_l_seg");

        sub_topic = nh->subscribe(sub_each, 1 , & vlp_::EachCombineObject,this);
        pub_left_each_add_imp = nh->advertise<pharos_vlp_tilt::vector_perfect_array>(pub_each,1);


        pub_test = nh->advertise<sensor_msgs::PointCloud2>(pub_each_cloud,1);
//        pub_test_min_max = nh->advertise<sensor_msgs::PointCloud2>("each_square_l",1);



    }

    void DoCombineLR(CombineType& std, CombineType& obj)
    {
        std.min_object.x = (std.min_object.x <= obj.min_object.x) ? std.min_object.x : obj.min_object.x;
        std.min_object.y = (std.min_object.y <= obj.min_object.y) ? std.min_object.y : obj.min_object.y;
        std.min_object.z = (std.min_object.z <= obj.min_object.z) ? std.min_object.z : obj.min_object.z;
        std.max_object.x = (std.max_object.x >= obj.max_object.x) ? std.max_object.x : obj.max_object.x;
        std.max_object.y = (std.max_object.y >= obj.max_object.y) ? std.max_object.y : obj.max_object.y;
        std.max_object.z = (std.max_object.z >= obj.max_object.z) ? std.max_object.z : obj.max_object.z;

        std.min_center = (std.min_hori <= obj.min_hori) ? std.min_center : obj.min_center;
        std.max_center = (std.max_hori >= obj.max_hori) ? std.max_center : obj.max_center;
    }

    double CenterXYDistance(CombineType std, CombineType obj)
    {
        double L;
        L = sqrt( (std.center_x-obj.center_x)*(std.center_x-obj.center_x) +(std.center_y-obj.center_y)*(std.center_y-obj.center_y) + (std.center_z-obj.center_z)*(std.center_z-obj.center_z) );
        return L;
    }


    pharos_vlp_tilt::center_position PointtoCenter(pharos_vlp_tilt::point pt)
    {
        pharos_vlp_tilt::center_position cpt;

        cpt.x = pt.x;
        cpt.y = pt.y;
        cpt.z = pt.z;

        return cpt;
    }

    void CombineObject(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& pointvector
                      ,std::vector<CombineType>& combinevector , std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& infovector)
    {
        for(unsigned int standard = 0; standard < combinevector.size(); standard++ )
        { /// i가 기준이다.
            for(unsigned int obj = standard+1; obj < combinevector.size(); obj++)
            {

                if(combinevector[standard].max_object.x >= combinevector[obj].max_object.x - XYTHRESHOLD && combinevector[standard].min_object.x <= combinevector[obj].max_object.x + XYTHRESHOLD
                && combinevector[standard].max_object.y >= combinevector[obj].max_object.y - XYTHRESHOLD && combinevector[standard].min_object.y <= combinevector[obj].max_object.y + XYTHRESHOLD)
                {


                    pcl::PointCloud<pcl::PointXYZI>::Ptr combine_center(new pcl::PointCloud<pcl::PointXYZI>);
                    for (int i = 0; i < pointvector[obj]->points.size(); i++)
                    {
                        pointvector[standard]->push_back(pointvector[obj]->points[i]);
                        infovector[standard]->push_back(infovector[obj]->points[i]);
                    }
//                    *combine_center = *pointvector[standard] + *pointvector[obj];
//                    *infovector[standard] = *infovector[standard] + *infovector[obj];

//                    pointvector[standard] = combine_center;



                    combinevector[standard].center_x = (combinevector[standard].center_x + combinevector[obj].center_x) / 2;
                    combinevector[standard].center_y = (combinevector[standard].center_y + combinevector[obj].center_y) / 2;
                    combinevector[standard].center_z = (combinevector[standard].center_z + combinevector[obj].center_z) / 2;

//                    DoCombineLR(combinevector[standard],combinevector[obj]);

                    combinevector[standard].min_object.x = (combinevector[standard].min_object.x <= combinevector[obj].min_object.x) ? combinevector[standard].min_object.x : combinevector[obj].min_object.x;
                    combinevector[standard].min_object.y = (combinevector[standard].min_object.y <= combinevector[obj].min_object.y) ? combinevector[standard].min_object.y : combinevector[obj].min_object.y;
                    combinevector[standard].min_object.z = (combinevector[standard].min_object.z <= combinevector[obj].min_object.z) ? combinevector[standard].min_object.z : combinevector[obj].min_object.z;
                    combinevector[standard].max_object.x = (combinevector[standard].max_object.x >= combinevector[obj].max_object.x) ? combinevector[standard].max_object.x : combinevector[obj].max_object.x;
                    combinevector[standard].max_object.y = (combinevector[standard].max_object.y >= combinevector[obj].max_object.y) ? combinevector[standard].max_object.y : combinevector[obj].max_object.y;
                    combinevector[standard].max_object.z = (combinevector[standard].max_object.z >= combinevector[obj].max_object.z) ? combinevector[standard].max_object.z : combinevector[obj].max_object.z;

                    combinevector[standard].min_center = (combinevector[standard].min_hori <= combinevector[obj].min_hori) ? combinevector[standard].min_center : combinevector[obj].min_center;
                    combinevector[standard].max_center = (combinevector[standard].max_hori >= combinevector[obj].max_hori) ? combinevector[standard].max_center : combinevector[obj].max_center;


                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    combinevector.erase(combinevector.begin() + obj);
                    pointvector.erase(pointvector.begin() + obj);
                    infovector.erase(infovector.begin()+obj);
                    obj = standard;
//                    ros::Time last = ros::Time::now();




                }

                else if(combinevector[standard].max_object.x >= combinevector[obj].min_object.x - XYTHRESHOLD && combinevector[standard].min_object.x <= combinevector[obj].min_object.x + XYTHRESHOLD
                && combinevector[standard].max_object.y >= combinevector[obj].min_object.y - XYTHRESHOLD && combinevector[standard].min_object.y <= combinevector[obj].min_object.y + XYTHRESHOLD)
                {
//                    diff_value = ros::Time::now().toSec();
//
//
//                    ros::Time begin = ros::Time::now();

                    pcl::PointCloud<pcl::PointXYZI>::Ptr combine_center(new pcl::PointCloud<pcl::PointXYZI>);
                    for (int i = 0; i < pointvector[obj]->points.size(); i++)
                    {
                        pointvector[standard]->push_back(pointvector[obj]->points[i]);
                        infovector[standard]->push_back(infovector[obj]->points[i]);
                    }
//                    *combine_center = *pointvector[standard] + *pointvector[obj];
//                    *infovector[standard] = *infovector[standard] + *infovector[obj];

//                    pointvector[standard] = combine_center;



                    combinevector[standard].center_x = (combinevector[standard].center_x + combinevector[obj].center_x) / 2;
                    combinevector[standard].center_y = (combinevector[standard].center_y + combinevector[obj].center_y) / 2;
                    combinevector[standard].center_z = (combinevector[standard].center_z + combinevector[obj].center_z) / 2;

//                    DoCombineLR(combinevector[standard],combinevector[obj]);

                    combinevector[standard].min_object.x = (combinevector[standard].min_object.x <= combinevector[obj].min_object.x) ? combinevector[standard].min_object.x : combinevector[obj].min_object.x;
                    combinevector[standard].min_object.y = (combinevector[standard].min_object.y <= combinevector[obj].min_object.y) ? combinevector[standard].min_object.y : combinevector[obj].min_object.y;
                    combinevector[standard].min_object.z = (combinevector[standard].min_object.z <= combinevector[obj].min_object.z) ? combinevector[standard].min_object.z : combinevector[obj].min_object.z;
                    combinevector[standard].max_object.x = (combinevector[standard].max_object.x >= combinevector[obj].max_object.x) ? combinevector[standard].max_object.x : combinevector[obj].max_object.x;
                    combinevector[standard].max_object.y = (combinevector[standard].max_object.y >= combinevector[obj].max_object.y) ? combinevector[standard].max_object.y : combinevector[obj].max_object.y;
                    combinevector[standard].max_object.z = (combinevector[standard].max_object.z >= combinevector[obj].max_object.z) ? combinevector[standard].max_object.z : combinevector[obj].max_object.z;

                    combinevector[standard].min_center = (combinevector[standard].min_hori <= combinevector[obj].min_hori) ? combinevector[standard].min_center : combinevector[obj].min_center;
                    combinevector[standard].max_center = (combinevector[standard].max_hori >= combinevector[obj].max_hori) ? combinevector[standard].max_center : combinevector[obj].max_center;


                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    combinevector.erase(combinevector.begin() + obj);
                    pointvector.erase(pointvector.begin() + obj);
                    infovector.erase(infovector.begin()+obj);
                    obj = standard;
//                    ros::Time last = ros::Time::now();


//                    diff_value = last.toSec() - begin.toSec();
//
//                    ROS_WARN("each_L time : %lf" , diff_value);

                }

                else if(combinevector[standard].max_object.x >= combinevector[obj].max_object.x - XYTHRESHOLD && combinevector[standard].min_object.x <= combinevector[obj].max_object.x + XYTHRESHOLD
                && combinevector[standard].max_object.y >= combinevector[obj].min_object.y - XYTHRESHOLD && combinevector[standard].min_object.y <= combinevector[obj].min_object.y + XYTHRESHOLD)
                {
//                    diff_value = ros::Time::now().toSec();
//
//
//                    ros::Time begin = ros::Time::now();

                    pcl::PointCloud<pcl::PointXYZI>::Ptr combine_center(new pcl::PointCloud<pcl::PointXYZI>);
                    for (int i = 0; i < pointvector[obj]->points.size(); i++)
                    {
                        pointvector[standard]->push_back(pointvector[obj]->points[i]);
                        infovector[standard]->push_back(infovector[obj]->points[i]);
                    }
//                    *combine_center = *pointvector[standard] + *pointvector[obj];
//                    *infovector[standard] = *infovector[standard] + *infovector[obj];

//                    pointvector[standard] = combine_center;



                    combinevector[standard].center_x = (combinevector[standard].center_x + combinevector[obj].center_x) / 2;
                    combinevector[standard].center_y = (combinevector[standard].center_y + combinevector[obj].center_y) / 2;
                    combinevector[standard].center_z = (combinevector[standard].center_z + combinevector[obj].center_z) / 2;

//                    DoCombineLR(combinevector[standard],combinevector[obj]);

                    combinevector[standard].min_object.x = (combinevector[standard].min_object.x <= combinevector[obj].min_object.x) ? combinevector[standard].min_object.x : combinevector[obj].min_object.x;
                    combinevector[standard].min_object.y = (combinevector[standard].min_object.y <= combinevector[obj].min_object.y) ? combinevector[standard].min_object.y : combinevector[obj].min_object.y;
                    combinevector[standard].min_object.z = (combinevector[standard].min_object.z <= combinevector[obj].min_object.z) ? combinevector[standard].min_object.z : combinevector[obj].min_object.z;
                    combinevector[standard].max_object.x = (combinevector[standard].max_object.x >= combinevector[obj].max_object.x) ? combinevector[standard].max_object.x : combinevector[obj].max_object.x;
                    combinevector[standard].max_object.y = (combinevector[standard].max_object.y >= combinevector[obj].max_object.y) ? combinevector[standard].max_object.y : combinevector[obj].max_object.y;
                    combinevector[standard].max_object.z = (combinevector[standard].max_object.z >= combinevector[obj].max_object.z) ? combinevector[standard].max_object.z : combinevector[obj].max_object.z;

                    combinevector[standard].min_center = (combinevector[standard].min_hori <= combinevector[obj].min_hori) ? combinevector[standard].min_center : combinevector[obj].min_center;
                    combinevector[standard].max_center = (combinevector[standard].max_hori >= combinevector[obj].max_hori) ? combinevector[standard].max_center : combinevector[obj].max_center;


                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    combinevector.erase(combinevector.begin() + obj);
                    pointvector.erase(pointvector.begin() + obj);
                    infovector.erase(infovector.begin()+obj);
                    obj = standard;
//                    ros::Time last = ros::Time::now();


//                    diff_value = last.toSec() - begin.toSec();
//
//                    ROS_WARN("each_L time : %lf" , diff_value);

                }

                else if(combinevector[standard].max_object.x >= combinevector[obj].min_object.x - XYTHRESHOLD && combinevector[standard].min_object.x <= combinevector[obj].min_object.x + XYTHRESHOLD
                && combinevector[standard].max_object.y >= combinevector[obj].max_object.y - XYTHRESHOLD && combinevector[standard].min_object.y <= combinevector[obj].max_object.y + XYTHRESHOLD)
                {
//                    diff_value = ros::Time::now().toSec();
//
//
//                    ros::Time begin = ros::Time::now();

                    pcl::PointCloud<pcl::PointXYZI>::Ptr combine_center(new pcl::PointCloud<pcl::PointXYZI>);
                    for (int i = 0; i < pointvector[obj]->points.size(); i++)
                    {
                        pointvector[standard]->push_back(pointvector[obj]->points[i]);
                        infovector[standard]->push_back(infovector[obj]->points[i]);
                    }
//                    *combine_center = *pointvector[standard] + *pointvector[obj];
//                    *infovector[standard] = *infovector[standard] + *infovector[obj];

//                    pointvector[standard] = combine_center;



                    combinevector[standard].center_x = (combinevector[standard].center_x + combinevector[obj].center_x) / 2;
                    combinevector[standard].center_y = (combinevector[standard].center_y + combinevector[obj].center_y) / 2;
                    combinevector[standard].center_z = (combinevector[standard].center_z + combinevector[obj].center_z) / 2;

//                    DoCombineLR(combinevector[standard],combinevector[obj]);

                    combinevector[standard].min_object.x = (combinevector[standard].min_object.x <= combinevector[obj].min_object.x) ? combinevector[standard].min_object.x : combinevector[obj].min_object.x;
                    combinevector[standard].min_object.y = (combinevector[standard].min_object.y <= combinevector[obj].min_object.y) ? combinevector[standard].min_object.y : combinevector[obj].min_object.y;
                    combinevector[standard].min_object.z = (combinevector[standard].min_object.z <= combinevector[obj].min_object.z) ? combinevector[standard].min_object.z : combinevector[obj].min_object.z;
                    combinevector[standard].max_object.x = (combinevector[standard].max_object.x >= combinevector[obj].max_object.x) ? combinevector[standard].max_object.x : combinevector[obj].max_object.x;
                    combinevector[standard].max_object.y = (combinevector[standard].max_object.y >= combinevector[obj].max_object.y) ? combinevector[standard].max_object.y : combinevector[obj].max_object.y;
                    combinevector[standard].max_object.z = (combinevector[standard].max_object.z >= combinevector[obj].max_object.z) ? combinevector[standard].max_object.z : combinevector[obj].max_object.z;

                    combinevector[standard].min_center = (combinevector[standard].min_hori <= combinevector[obj].min_hori) ? combinevector[standard].min_center : combinevector[obj].min_center;
                    combinevector[standard].max_center = (combinevector[standard].max_hori >= combinevector[obj].max_hori) ? combinevector[standard].max_center : combinevector[obj].max_center;


                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    combinevector.erase(combinevector.begin() + obj);
                    pointvector.erase(pointvector.begin() + obj);
                    infovector.erase(infovector.begin()+obj);
                    obj = standard;
//                    ros::Time last = ros::Time::now();


//                    diff_value = last.toSec() - begin.toSec();
//
//                    ROS_WARN("each_L time : %lf" , diff_value);

                }
                else if(combinevector[obj].max_object.x >= combinevector[standard].center_x - XYTHRESHOLD && combinevector[obj].min_object.x <= combinevector[standard].center_x + XYTHRESHOLD
                        && combinevector[obj].max_object.y >= combinevector[standard].center_y - XYTHRESHOLD && combinevector[obj].min_object.y <= combinevector[standard].center_y + XYTHRESHOLD)
                {
//                    diff_value = ros::Time::now().toSec();
//
//
//                    ros::Time begin = ros::Time::now();

                    pcl::PointCloud<pcl::PointXYZI>::Ptr combine_center(new pcl::PointCloud<pcl::PointXYZI>);
                    for (int i = 0; i < pointvector[obj]->points.size(); i++)
                    {
                        pointvector[standard]->push_back(pointvector[obj]->points[i]);
                        infovector[standard]->push_back(infovector[obj]->points[i]);
                    }
//                    *combine_center = *pointvector[standard] + *pointvector[obj];
//                    *infovector[standard] = *infovector[standard] + *infovector[obj];

//                    pointvector[standard] = combine_center;



                    combinevector[standard].center_x = (combinevector[standard].center_x + combinevector[obj].center_x) / 2;
                    combinevector[standard].center_y = (combinevector[standard].center_y + combinevector[obj].center_y) / 2;
                    combinevector[standard].center_z = (combinevector[standard].center_z + combinevector[obj].center_z) / 2;

//                    DoCombineLR(combinevector[standard],combinevector[obj]);

                    combinevector[standard].min_object.x = (combinevector[standard].min_object.x <= combinevector[obj].min_object.x) ? combinevector[standard].min_object.x : combinevector[obj].min_object.x;
                    combinevector[standard].min_object.y = (combinevector[standard].min_object.y <= combinevector[obj].min_object.y) ? combinevector[standard].min_object.y : combinevector[obj].min_object.y;
                    combinevector[standard].min_object.z = (combinevector[standard].min_object.z <= combinevector[obj].min_object.z) ? combinevector[standard].min_object.z : combinevector[obj].min_object.z;
                    combinevector[standard].max_object.x = (combinevector[standard].max_object.x >= combinevector[obj].max_object.x) ? combinevector[standard].max_object.x : combinevector[obj].max_object.x;
                    combinevector[standard].max_object.y = (combinevector[standard].max_object.y >= combinevector[obj].max_object.y) ? combinevector[standard].max_object.y : combinevector[obj].max_object.y;
                    combinevector[standard].max_object.z = (combinevector[standard].max_object.z >= combinevector[obj].max_object.z) ? combinevector[standard].max_object.z : combinevector[obj].max_object.z;

                    combinevector[standard].min_center = (combinevector[standard].min_hori <= combinevector[obj].min_hori) ? combinevector[standard].min_center : combinevector[obj].min_center;
                    combinevector[standard].max_center = (combinevector[standard].max_hori >= combinevector[obj].max_hori) ? combinevector[standard].max_center : combinevector[obj].max_center;


                    ///erase 중심점으로 할 경우에는 터널이 중심위치에서 차량 보일때 합쳐질 경우 가 있다.
                    combinevector.erase(combinevector.begin() + obj);
                    pointvector.erase(pointvector.begin() + obj);
                    infovector.erase(infovector.begin()+obj);
                    obj = standard;
//                    ros::Time last = ros::Time::now();


//                    diff_value = last.toSec() - begin.toSec();
//
//                    ROS_WARN("each_L time : %lf" , diff_value);
                }
                else{}

//                std::cout << combinevector[standard].min_object.x<< std::endl;
//                std::cout << combinevector[standard].min_object.y<< std::endl;
//                std::cout << combinevector[standard].min_object.z<< std::endl;
            }

        }





    }



    void ListSeqeunce(pharos_vlp_tilt::perfectarray& one  , std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& pointvector
            , std::vector<CombineType>& combinevector ,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr >& infooo    )
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        CombineType object_pose_info;
        pcl::PointCloud<pcl::PointXYZI>::Ptr laser_and_hori (new pcl::PointCloud<pcl::PointXYZI>);

        double center_x(0) ,center_y(0), center_z(0);
        pcl::PointXYZI pt;
        pcl::PointXYZI infopt;

         /// @won array에서는 해당 값의 i의 값을 넣는다.

        int info_min = 10000;
        int info_max = -1;


        for(unsigned int i=0; i < one.objects.size(); i++)
        {
            center_x += one.objects[i].point.x ,pt.x = one.objects[i].point.x;
            center_y += one.objects[i].point.y ,pt.y = one.objects[i].point.y;
            center_z += one.objects[i].point.z ,pt.z = one.objects[i].point.z;
            ///!!!!
            infopt.x = one.objects[i].info.hori;
            infopt.y = one.objects[i].info.laser;
            laser_and_hori->push_back(infopt);


            object_cloud->push_back(pt);

            /// @won  Left_end_point
            if(i == 0)/// index 즉 laser ling 넘버에 들어있는게 하나도 없을때.
            {
                object_pose_info.min_object.x = one.objects[i].point.x;
                object_pose_info.min_object.y = one.objects[i].point.y;
                object_pose_info.min_object.z = one.objects[i].point.z;
                object_pose_info.max_object.x = one.objects[i].point.x;
                object_pose_info.max_object.y = one.objects[i].point.y;
                object_pose_info.max_object.z = one.objects[i].point.z;

            }

            else /// 뭔가 들어 있을때 이제 비교가 시작되어야 한다.
            {

                object_pose_info.min_object.x = (object_pose_info.min_object.x >= one.objects[i].point.x) ? one.objects[i].point.x : object_pose_info.min_object.x;
                object_pose_info.min_object.y = (object_pose_info.min_object.y >= one.objects[i].point.y) ? one.objects[i].point.y : object_pose_info.min_object.y;
                object_pose_info.min_object.z = (object_pose_info.min_object.z >= one.objects[i].point.z) ? one.objects[i].point.z : object_pose_info.min_object.z;
                object_pose_info.max_object.x = (object_pose_info.max_object.x <= one.objects[i].point.x) ? one.objects[i].point.x : object_pose_info.max_object.x;
                object_pose_info.max_object.y = (object_pose_info.max_object.y <= one.objects[i].point.y) ? one.objects[i].point.y : object_pose_info.max_object.y;
                object_pose_info.max_object.z = (object_pose_info.max_object.z <= one.objects[i].point.z) ? one.objects[i].point.z : object_pose_info.max_object.z;
                info_min = (info_min >= one.objects[i].info.hori) ? one.objects[i].info.hori : info_min;
                info_max = (info_max <= one.objects[i].info.hori) ? one.objects[i].info.hori : info_max;
                object_pose_info.min_hori = info_min;
                object_pose_info.max_hori = info_max;
                object_pose_info.min_center = (info_min == one.objects[i].info.hori) ? PointtoCenter(one.objects[i].point) : object_pose_info.min_center;
                object_pose_info.max_center = (info_max == one.objects[i].info.hori) ? PointtoCenter(one.objects[i].point) : object_pose_info.max_center;
            }



        }

        /// Center Position
        object_pose_info.center_x = center_x/one.objects.size();
        object_pose_info.center_y = center_y/one.objects.size();
        object_pose_info.center_z = center_z/one.objects.size();


        pointvector.push_back(object_cloud);
        combinevector.push_back(object_pose_info);
        infooo.push_back(laser_and_hori);

    }

    double ReturnCenterDistance(CombineType stdinput , CombineType objinput)
    {

        double L;
        L = sqrt( (stdinput.center_x-objinput.center_x)*(stdinput.center_x-objinput.center_x) + (stdinput.center_y-objinput.center_y)*(stdinput.center_y-objinput.center_y) );
        /// Z값은 무시하고 xy에 대한 길이만
        return L;

    }








    void EachCombineObject(const pharos_vlp_tilt::vector_perfect_arrayPtr& input)
    {


        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        //////////////////
        pcl::PointCloud<pcl::PointXYZI>::Ptr test_min_max(new pcl::PointCloud<pcl::PointXYZI>);
        //////////////////

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > vec_listed_pointcloud;
        vec_listed_pointcloud.reserve(20000);
        std::vector<CombineType> vec_listed_combine;
        vec_listed_combine.reserve(10000);
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr > vec_infos;
        vec_infos.reserve(10000);

        for(unsigned int i=0; i < input->one.size(); i++)
        {
            ListSeqeunce( input->one[i] ,vec_listed_pointcloud, vec_listed_combine,vec_infos );

        }
        ///output : vec_point , vec_combine

//        for (int j = 0; j < vec_listed_combine.size(); j++)
//        {
//
//
//            double gap_std_x = (vec_listed_combine[j].max_object.x - vec_listed_combine[j].min_object.x) / 15;
//            double gap_std_y = (vec_listed_combine[j].max_object.y - vec_listed_combine[j].min_object.y) / 15;
//            double gap_std_z = (vec_listed_combine[j].max_object.z - vec_listed_combine[j].min_object.z) / 15;
//            double gap_x = (vec_listed_combine[j].max_object.x - vec_listed_combine[j].min_object.x);
//            double gap_y = (vec_listed_combine[j].max_object.y - vec_listed_combine[j].min_object.y);
//            double gap_z = (vec_listed_combine[j].max_object.z - vec_listed_combine[j].min_object.z);
//
//            for (int l = 0; l < 3; l++)
//            {
//                for (int k = 0; k < 16; k++)
//                {
//                    pcl::PointXYZI pt;
//
//                    pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                    pt.y = vec_listed_combine[j].max_object.y;
//                    pt.z = vec_listed_combine[j].max_object.z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                    pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                    pt.z = vec_listed_combine[j].max_object.z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                    pt.y = vec_listed_combine[j].max_object.y;
//                    pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x - k * gap_std_x;
//                    pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                    pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                    test_min_max->push_back(pt);
//                }
//
//                for (int k = 1; k < 15; k++)
//                {
//
//                    pcl::PointXYZI pt;
//
//                    pt.x = vec_listed_combine[j].max_object.x;
//                    pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                    pt.z = vec_listed_combine[j].max_object.z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                    pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                    pt.z = vec_listed_combine[j].max_object.z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                    pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                    pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x;
//                    pt.y = vec_listed_combine[j].max_object.y - k * gap_std_y;
//                    pt.z = vec_listed_combine[j].max_object.z - gap_z;
//
//                    test_min_max->push_back(pt);
//                }
//
//                for (int k = 1; k < 15; k++)
//                {
//
//                    pcl::PointXYZI pt;
//
//                    pt.x = vec_listed_combine[j].max_object.x;
//                    pt.y = vec_listed_combine[j].max_object.y;
//                    pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                    pt.y = vec_listed_combine[j].max_object.y;
//                    pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x - gap_x;
//                    pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                    pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = vec_listed_combine[j].max_object.x;
//                    pt.y = vec_listed_combine[j].max_object.y - gap_y;
//                    pt.z = vec_listed_combine[j].max_object.z - k * gap_std_z;
//
//                    test_min_max->push_back(pt);
//
//                }
//            }
//        }

        CombineObject(vec_listed_pointcloud, vec_listed_combine ,vec_infos);

//
        pharos_vlp_tilt::vector_perfect_arrayPtr my_left_output(new pharos_vlp_tilt::vector_perfect_array);



        ///test
        pcl::PointCloud<pcl::PointXYZI>::Ptr test(new pcl::PointCloud<pcl::PointXYZI>);
        int inten = 10;
        ///test



        for(int i = 0; i < vec_listed_pointcloud.size(); i++)
        {///한객체
//
            pharos_vlp_tilt::perfectarrayPtr hhhh(new pharos_vlp_tilt::perfectarray);
            for(int j=0; j <vec_listed_pointcloud[i]->points.size(); j++)
            {///객체 하나의 포인트 전부를 돌린다.
                pharos_vlp_tilt::perfectPtr sub_arr(new pharos_vlp_tilt::perfect);
                sub_arr->point.x = vec_listed_pointcloud[i]->points[j].x;
                sub_arr->point.y = vec_listed_pointcloud[i]->points[j].y;
                sub_arr->point.z = vec_listed_pointcloud[i]->points[j].z;
                sub_arr->info.laser = vec_infos[i]->points[j].y;
                sub_arr->info.hori = vec_infos[i]->points[j].x;
                hhhh->objects.push_back(*sub_arr);

                ///test///
                pcl::PointXYZI pt;

                pt.x = vec_listed_pointcloud[i]->points[j].x;
                pt.y = vec_listed_pointcloud[i]->points[j].y;
                pt.z = vec_listed_pointcloud[i]->points[j].z;

                pt.intensity = inten;

                test->push_back(pt);

            }

            inten += 15;

            hhhh->center.x = vec_listed_combine[i].center_x;
            hhhh->center.y = vec_listed_combine[i].center_y;
            hhhh->center.z = vec_listed_combine[i].center_z;
            hhhh->max_center = vec_listed_combine[i].max_center;
            hhhh->min_center = vec_listed_combine[i].min_center;
            hhhh->max_object = vec_listed_combine[i].max_object;
            hhhh->min_object = vec_listed_combine[i].min_object;
            hhhh->max_hori = vec_listed_combine[i].max_hori;
            hhhh->min_hori = vec_listed_combine[i].min_hori;

//
            my_left_output->one.push_back(*hhhh);

        }

//        for (int j = 0; j < my_left_output->one.size(); j++)
//        {
//
//
//            double gap_std_x = (my_left_output->one[j].max_object.x - my_left_output->one[j].min_object.x) / 15;
//            double gap_std_y = (my_left_output->one[j].max_object.y - my_left_output->one[j].min_object.y) / 15;
//            double gap_std_z = (my_left_output->one[j].max_object.z - my_left_output->one[j].min_object.z) / 15;
//            double gap_x = (my_left_output->one[j].max_object.x - my_left_output->one[j].min_object.x);
//            double gap_y = (my_left_output->one[j].max_object.y - my_left_output->one[j].min_object.y);
//            double gap_z = (my_left_output->one[j].max_object.z - my_left_output->one[j].min_object.z);
//
//            for (int l = 0; l < 3; l++)
//            {
//                for (int k = 0; k < 16; k++)
//                {
//                    pcl::PointXYZI pt;
//
//                    pt.x = my_left_output->one[j].max_object.x - k * gap_std_x;
//                    pt.y = my_left_output->one[j].max_object.y;
//                    pt.z = my_left_output->one[j].max_object.z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x - k * gap_std_x;
//                    pt.y = my_left_output->one[j].max_object.y - gap_y;
//                    pt.z = my_left_output->one[j].max_object.z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x - k * gap_std_x;
//                    pt.y = my_left_output->one[j].max_object.y;
//                    pt.z = my_left_output->one[j].max_object.z - gap_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x - k * gap_std_x;
//                    pt.y = my_left_output->one[j].max_object.y - gap_y;
//                    pt.z = my_left_output->one[j].max_object.z - gap_z;
//
//                    test_min_max->push_back(pt);
//                }
//
//                for (int k = 1; k < 15; k++)
//                {
//
//                    pcl::PointXYZI pt;
//
//                    pt.x = my_left_output->one[j].max_object.x;
//                    pt.y = my_left_output->one[j].max_object.y - k * gap_std_y;
//                    pt.z = my_left_output->one[j].max_object.z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x - gap_x;
//                    pt.y = my_left_output->one[j].max_object.y - k * gap_std_y;
//                    pt.z = my_left_output->one[j].max_object.z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x - gap_x;
//                    pt.y = my_left_output->one[j].max_object.y - k * gap_std_y;
//                    pt.z = my_left_output->one[j].max_object.z - gap_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x;
//                    pt.y = my_left_output->one[j].max_object.y - k * gap_std_y;
//                    pt.z = my_left_output->one[j].max_object.z - gap_z;
//
//                    test_min_max->push_back(pt);
//                }
//
//                for (int k = 1; k < 15; k++)
//                {
//
//                    pcl::PointXYZI pt;
//
//                    pt.x = my_left_output->one[j].max_object.x;
//                    pt.y = my_left_output->one[j].max_object.y;
//                    pt.z = my_left_output->one[j].max_object.z - k * gap_std_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x - gap_x;
//                    pt.y = my_left_output->one[j].max_object.y;
//                    pt.z = my_left_output->one[j].max_object.z - k * gap_std_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x - gap_x;
//                    pt.y = my_left_output->one[j].max_object.y - gap_y;
//                    pt.z = my_left_output->one[j].max_object.z - k * gap_std_z;
//
//                    test_min_max->push_back(pt);
//
//                    pt.x = my_left_output->one[j].max_object.x;
//                    pt.y = my_left_output->one[j].max_object.y - gap_y;
//                    pt.z = my_left_output->one[j].max_object.z - k * gap_std_z;
//
//                    test_min_max->push_back(pt);
//
//                }
//            }
//        }


        my_left_output->header.frame_id = frame_each;
        my_left_output->header.stamp = input->header.stamp;
        pub_left_each_add_imp.publish(*my_left_output);


        sensor_msgs::PointCloud2 test_cloud;
        pcl::toROSMsg(*test , test_cloud);
        test_cloud.header.frame_id = frame_each;
        test_cloud.header.stamp = input->header.stamp;
        pub_test.publish(test_cloud);

//        sensor_msgs::PointCloud2Ptr sensor_pointcloud_test(new sensor_msgs::PointCloud2);
//        pcl::toROSMsg(*test_min_max, * sensor_pointcloud_test);
//        sensor_pointcloud_test->header.frame_id = frame_each;
//        sensor_pointcloud_test->header.stamp = input->header.stamp;
//        pub_test_min_max.publish(*sensor_pointcloud_test);

        ros::Time last = ros::Time::now();

        diff_value = last.toSec() - begin.toSec();

        if(isTimeRight == true)
        {
            ROS_INFO("each_R time : %lf", diff_value);
        }

    }




protected:
    ros::Subscriber sub_topic;

    ros::Publisher pub_left_each_add_imp;
    ros::Publisher pub_test;

//    ////////////////
//    ros::Publisher pub_test_min_max;
//    ///////////////

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_each_L");       //노드명 초기화



    ROS_INFO("started vlpt_each_R");
//    ROS_INFO("SUBTOPIC : ");
//    ROS_INFO("PUBTOPIC : ");

    vlp_ hello;

    ros::spin();


}

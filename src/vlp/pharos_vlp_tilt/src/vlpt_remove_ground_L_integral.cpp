#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <velodyne_msgs/custompoint.h>
#include <pcl/common/transforms.h>
#include <math.h>
#include <Eigen/Geometry>
#include <pharos_vlp_tilt/perfectarray.h>
#include <pharos_vlp_tilt/vector_perfect_array.h>
#include <iostream>
#include <string>
#include <Eigen/QR>
//#include <Eigen/Dense>


#define le 3
//#define a 25

double diff_value;

double THRESHOLD;
double AREA_THRES;
double AREA_GAIN;
double ALPHA_GAIN;
double CAR_HEIGHT_;
double ALPHA_MAX_;
double SET_GROUND_DEG;
double SetUnderZ;
std::string sub_topic;
std::string pub_topic_ground;
std::string pub_topic_objects;
std::string pub_topic_my_msg;

std::string frame_remove;
double init_ALPHA_MAX_;
double Long_Distance;
double pt_angle;



class vlp_
{
public:

    ros::NodeHandlePtr n;
    ros::NodeHandlePtr pn;


    vlp_()
    {
        n = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pn = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        pn->param<double>("/vlpt_set_T/CAR_HEIGHT",CAR_HEIGHT_, -1.6);
        n->param<double>("ALPHA_MAX_",ALPHA_MAX_, 20);
        n->param<double>("init_ALPHA_MAX_",init_ALPHA_MAX_, 5);
        n->param<double>("Long_Distance",Long_Distance, 45);
        n->param<double>("SetUnderZ",SetUnderZ, 1);
        n->param<double>("SET_GROUND_DEG",SET_GROUND_DEG, 10);
        n->param<double>("THRESHOLD",THRESHOLD, 0.3);
        n->param<double>("AREA_THRES",AREA_THRES, 0.05);
        n->param<double>("AREA_GAIN",AREA_GAIN, 0.05);
        n->param<double>("ALPHA_GAIN",ALPHA_GAIN, 0.1);


        pn->param<std::string>("sub_topic",sub_topic, "left_my_msg");
        pn->param<std::string>("pub_topic_ground",pub_topic_ground, "my_l_ground");
        pn->param<std::string>("pub_topic_objects",pub_topic_objects, "my_l_object");
        pn->param<std::string>("frame_remove",frame_remove, "left_velodyne2");
        pn->param<std::string>("pub_topic_my_msg",pub_topic_my_msg, "l_object");

        sub_set = n->subscribe(sub_topic,1 , &vlp_::RemoveGround , this);

        pub_ground = n->advertise<sensor_msgs::PointCloud2>(pub_topic_ground,10);
        pub_object = n->advertise<sensor_msgs::PointCloud2>(pub_topic_objects,10);
        pub_msgs = n->advertise<pharos_vlp_tilt::perfectarray>(pub_topic_my_msg,10);

    }


    double distance_two_XY(pharos_vlp_tilt::point before ,pharos_vlp_tilt::point after)
    {
        double L;
        L = sqrt( (before.x-after.x)*(before.x-after.x) + (before.y - after.y)*(before.y - after.y) );
        return L;
    }

    double Area_Threshold(double L)
    {
        double A;
        A = L * AREA_THRES * AREA_GAIN / 2;
        return A;
    }

    double Area_Curbstone(pharos_vlp_tilt::point std, pharos_vlp_tilt::point obj)
    {
        double A;
        A = (distance_two_XY(std, obj))* (std.z - CAR_HEIGHT_);
        return A;
    }

    double Area (pharos_vlp_tilt::point std, pharos_vlp_tilt::point obj)
    {
        double A;
        A = (((obj.z - CAR_HEIGHT_) - (std.z - CAR_HEIGHT_))*(distance_two_XY(std, obj)))/2;
        return A;
    }

    double distance_two(pharos_vlp_tilt::point before ,pharos_vlp_tilt::point after)
    {
        double L;
        L = sqrt( (before.x-after.x)*(before.x-after.x) + (before.y - after.y)*(before.y - after.y) + (before.z - after.z)*(before.z - after.z) );
        return L;
    }

    double distance_two_center (pharos_vlp_tilt::center_position before ,pharos_vlp_tilt::center_position after)
    {
        double L;
        L = sqrt( (before.x-after.x)*(before.x-after.x) + (before.y - after.y)*(before.y - after.y) + (before.z - after.z)*(before.z - after.z) );
        return L;
    }


    bool distance_two_f(pharos_vlp_tilt::point before ,pharos_vlp_tilt::point after)
    {

        double Before_L ,After_L;
        Before_L = sqrt(before.x*before.x+before.y*before.y);
        After_L = sqrt(after.x*after.x + after.y*after.y);


        if(After_L <= Before_L){
            return true; //// 이전좀보다 최근점이 더 가까울때 절때 그럴일이 없다.
        }
        else
        {
            return false;
        }


    }

    void DoDistinction_Integral(std::vector<int> num_vec, pharos_vlp_tilt::perfectarrayPtr &pta)
    {
        for (int index = 0; index < num_vec.size(); index++)
        {
            if(index == 0) /// 첫번쨰 레이저 층일 때
            {
                pharos_vlp_tilt::point std_point; /// 첫번째 레이저 포인트를 비교할 정확히 땅인 기준점
                std_point.x = 0;
                std_point.y = 0;
                std_point.z = CAR_HEIGHT_;

                if(pta->objects[num_vec[index]].point.z <= CAR_HEIGHT_)
                {
                    pta->objects[num_vec[index]].state.is_ground = 1; /// 땅이다.
                }

                else
                {
                    double A = Area(std_point, pta->objects[num_vec[index]].point);
                    /// 땅과 두개의 포인트를 이용하여 사다리꼴을 만들어
                    /// 그 도형의 넓이를 구한다.

                    if(A > Area_Threshold(distance_two_XY(std_point, pta->objects[num_vec[index]].point))) /// 그 넓이가 문턱값보다 높으면
                    {
                        pta->objects[num_vec[index]].state.is_ground = 0; /// 물체이다.
                    }

                    else /// 아니면
                    {
                        pta->objects[num_vec[index]].state.is_ground = 1; /// 땅이다.
                    }
                }
            }

            else
            {
                if(pta->objects[num_vec[index]].point.z <= CAR_HEIGHT_)
                {
                    /// 차높이보다 낮을 때
                    pta->objects[num_vec[index]].state.is_ground = 1; /// 땅이다.
                    continue;
                }

                if(distance_two_f(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point) == true)
                {
                    /// 이전점보다 최근점이 더 가까울 때
                    pta->objects[num_vec[index]].state.is_ground = 0; /// 물체이다.
                    continue;
                }

                if(pta->objects[num_vec[index - 1]].state.is_ground == 1) /// 전점이 땅일 때
                {
                    double A = Area(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point);

                    if(A > Area_Threshold(distance_two_XY(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point))) /// 그 넓이가 문턱값보다 높으면
                    {
                        pta->objects[num_vec[index]].state.is_ground = 0; /// 물체이다.
                    }

                    else /// 아니면
                    {
                        pta->objects[num_vec[index]].state.is_ground = 1; /// 땅이다.
                    }
                }

                else
                {
                    double A = Area(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point);

                    if(pta->objects[num_vec[index]].point.z > pta->objects[num_vec[index - 1]].point.z && distance_two_XY(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point) < 0.5)
                    {
                        pta->objects[num_vec[index]].state.is_ground = 0; /// 물체이다.
                        continue;
                    }

//                    else if(A - Area_Curbstone(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point) < 0.025)
//                    {
//                        pta->objects[num_vec[index]].state.is_ground = 1; /// 땅이다.
//                    }

//                    else if()
//                    {
//
//                    }

                    if(pta->objects[num_vec[index - 1]].point.z > pta->objects[num_vec[index]].point.z)
                    {
                        if((A < Area_Threshold(distance_two_XY(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point))) * 1.25)
                        {
                            pta->objects[num_vec[index]].state.is_ground = 1; /// 땅이다.
                        }

                        else
                        {
                            pta->objects[num_vec[index]].state.is_ground = 0; /// 물체이다.
                        }
                    }

                    else
                    {
                        pta->objects[num_vec[index]].state.is_ground = 0; /// 땅이다.
                    }
                }
            }
        }

//        for (int i = 0; i < ; ++i) {
//
//        }

//        double alpha0;
//        bool isCheck = false;
//
//        for (int index = 0; index < num_vec.size(); index++)
//        {
//            if (index == 0) /// 첫번쨰 레이저 층일 때
//            {
//                pharos_vlp_tilt::point std_point; /// 첫번째 레이저 포인트를 비교할 정확히 땅인 기준점
//                std_point.x = 0;
//                std_point.y = 0;
//                std_point.z = CAR_HEIGHT_;
//
//                alpha0 = (pta->objects[index].point.y - std_point.y)/(pta->objects[index].point.x - std_point.x);
//                isCheck = true;
//            }
//
//            else if(index == 1)
//            {
//                double alpha1 = (pta->objects[index].point.y - pta->objects[index - 1].point.y)/(pta->objects[index].point.x - pta->objects[index - 1].point.x);
//
//                if(fabs(alpha1 - alpha0) < ALPHA_GAIN)
//                {
//                    pta->objects[index].state.is_ground = 1;
//                    pta->objects[index - 1].state.is_ground = 1;
//                }
//            }
//
//            if(pta->objects[index].state.is_ground == 0)
//            {
//                if(!isCheck)
//                {
//                    continue;
//                }
//
//                else
//                {
//                    double alpha1 = (pta->objects[index].point.y - pta->objects[index - 1].point.y)/(pta->objects[index].point.x - pta->objects[index - 1].point.x);
//
//                    if(fabs(alpha1 - alpha0) < ALPHA_GAIN)
//                    {
//                        pta->objects[index].state.is_ground = 1;
//                    }
//                }
//            }
//        }
    }


    void RemoveGround(const velodyne_msgs::custompointPtr &input)
    {
        diff_value = ros::Time::now().toSec();
        ros::Time begin = ros::Time::now();

        pharos_vlp_tilt::perfectarrayPtr point_bank (new pharos_vlp_tilt::perfectarray);
        std::vector<int> index_bank;
        pharos_vlp_tilt::perfectPtr point_ (new pharos_vlp_tilt::perfect);
        pharos_vlp_tilt::infoPtr info_ (new pharos_vlp_tilt::info);

        int hori = 1;

        for (int index = 0; index < input->cpoints.size(); index++)
        {
            point_->point.x = input->cpoints[index].x;
            point_->point.y = input->cpoints[index].y;
            point_->point.z = input->cpoints[index].z;
            point_->point.intensity = input->cpoints[index].intensity;

            point_->info.hori = hori;
            point_->info.laser = input->infos[index].ring;

            point_->state.is_ground = 1;

            point_bank->objects.push_back(*point_);
            index_bank.push_back(index);


            if(input->infos[index].hori != input->infos[index + 1].hori || index + 1 == input->cpoints.size())
            {
                DoDistinction_Integral(index_bank, point_bank);
                index_bank.clear();
                hori++;
            }
        }

//        std::cout << 1 << std::endl;
        pharos_vlp_tilt::perfectarrayPtr remove_ground_msg (new pharos_vlp_tilt::perfectarray);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pt (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr object_pt (new pcl::PointCloud<pcl::PointXYZI>);

        for (int i = 0; i < point_bank->objects.size(); i++)
        {
            pcl::PointXYZI pt;

            if(point_bank->objects[i].state.is_ground == 1)
            {
                pt.x = point_bank->objects[i].point.x;
                pt.y = point_bank->objects[i].point.y;
                pt.z = point_bank->objects[i].point.z;
                pt.intensity = point_bank->objects[i].point.intensity;

                ground_pt->push_back(pt);
            }

            else
            {
                pt.x = point_bank->objects[i].point.x;
                pt.y = point_bank->objects[i].point.y;
                pt.z = point_bank->objects[i].point.z;
                pt.intensity = point_bank->objects[i].point.intensity;

                object_pt->push_back(pt);

                if(point_bank->objects[i].point.z <= SetUnderZ)
                {
                    remove_ground_msg->objects.push_back(point_bank->objects[i]);
                }
            }
        }

        sensor_msgs::PointCloud2Ptr cloud_ground (new sensor_msgs::PointCloud2);
        sensor_msgs::PointCloud2Ptr cloud_object (new sensor_msgs::PointCloud2);

        pcl::toROSMsg(*ground_pt,*cloud_ground);
        pcl::toROSMsg(*object_pt,*cloud_object);

        cloud_ground->header.stamp = input->header.stamp;
        cloud_ground->header.frame_id = frame_remove;
        cloud_object->header.stamp = input->header.stamp;
        cloud_object->header.frame_id = frame_remove;

        remove_ground_msg->header.stamp = input->header.stamp;
        remove_ground_msg->header.frame_id = frame_remove;

        pub_msgs.publish(remove_ground_msg);
        pub_ground.publish(cloud_ground);
        pub_object.publish(cloud_object);



        ros::Time last = ros::Time::now();


        diff_value = last.toSec() - begin.toSec();

//        ROS_ERROR("diff_value time is :: %lf" , diff_value);

}




private:
    ros::Subscriber sub_set;

    ros::Publisher pub_ground;
    ros::Publisher pub_object;
    ros::Publisher pub_msgs;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vlpt_delay_compensator");

    vlp_ ground;

    ros::spin();

}
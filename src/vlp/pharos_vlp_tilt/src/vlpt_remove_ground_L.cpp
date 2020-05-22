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
bool isTimeLeft;

double THRESHOLD;
double CAR_HEIGHT_;
double ALPHA_MAX_;
double SET_GROUND_DEG;
double SetUnderZ;
std::string sub_remove;
std::string pub_ground_cloud;
std::string pub_objects_cloud;
std::string pub_objects;
bool isIntensity;

std::string frame_remove;
double init_ALPHA_MAX_;
double Long_Distance;
double pt_angle;

struct PointType
{
    std::vector<int> index;
    pharos_vlp_tilt::point center_pt;
    int min_index;
    int max_index;
};



class vlp_
{
public:

    ros::NodeHandlePtr n;
    ros::NodeHandlePtr pn;


    vlp_()
    {
        n = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pn = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        pn->param<double>("/vlpt_set_L/CAR_HEIGHT",CAR_HEIGHT_, -1.6);
        n->param<double>("ALPHA_MAX_",ALPHA_MAX_, 20);
        n->param<double>("init_ALPHA_MAX_",init_ALPHA_MAX_, 5);
        n->param<double>("Long_Distance",Long_Distance, 45);
        n->param<double>("SetUnderZ",SetUnderZ, 1);
        n->param<double>("SET_GROUND_DEG",SET_GROUND_DEG, 10);
        n->param<double>("THRESHOLD",THRESHOLD, 0.3);
        n->param<double>("pt_angle",pt_angle, 0.3);
        n->param<bool>("isIntensity",isIntensity, false);
        n->param<bool>("isTimeLeft",isTimeLeft, false);


        pn->param<std::string>("sub_remove",sub_remove, "left_my_msg");
        pn->param<std::string>("pub_ground_cloud",pub_ground_cloud, "my_l_ground");
        pn->param<std::string>("pub_objects_cloud",pub_objects_cloud, "my_l_object");
        pn->param<std::string>("frame_remove",frame_remove, "left_velodyne2");
        pn->param<std::string>("pub_objects",pub_objects, "l_object");

        sub_set = n->subscribe(sub_remove,1 , &vlp_::RemoveGround , this);

        pub_ground = n->advertise<sensor_msgs::PointCloud2>(pub_ground_cloud,10);
        pub_object = n->advertise<sensor_msgs::PointCloud2>(pub_objects_cloud,10);
        pub_msgs = n->advertise<pharos_vlp_tilt::perfectarray>(pub_objects,10);

//        pub_least_full_test = n->advertise<sensor_msgs::PointCloud2>("l_least_full_test", 10);

    }

    double distance_point (pharos_vlp_tilt::point pt)
    {
        double D;
        D = sqrt((pt.x * pt.x ) + (pt.y * pt.y) + (pt.z * pt.z));
        return D;
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


    template <typename DynamicEigenMatrix>
    void push_back_2(DynamicEigenMatrix& m, Eigen::Vector2d&& values, std::size_t row)
    {
        if(row >= m.rows()) {
            m.conservativeResize(row + 1, Eigen::NoChange);
        }
        m.row(row) = values;
    }


    template <typename DynamicEigenMatrix>
    void push_back_1(DynamicEigenMatrix& m, Eigen::VectorXd& values, std::size_t row)
    {
        if(row >= m.rows()) {
            m.conservativeResize(row + 1, Eigen::NoChange);
        }
        m.row(row) = values;
    }

    void DoDistinction_row(std::vector<int> num_vec , pharos_vlp_tilt::perfectarrayPtr& pta) {

        for (unsigned int index = 0; index < num_vec.size(); index++)
        {

            if (index == 0)
            {
                pharos_vlp_tilt::point init_point;
                init_point.x = 0, init_point.y = 0, init_point.z = CAR_HEIGHT_;


                double alpha = asinf((pta->objects[num_vec[index]].point.z - init_point.z)
                                     / distance_two(init_point, pta->objects[num_vec[index]].point));
                alpha = alpha * 180.0 / M_PI;

                if (alpha >= init_ALPHA_MAX_)
                {

                    /// 0 0 car_height와의 각도를 비교 했을때 높은 경우
                    pta->objects[num_vec[index]].state.is_ground = 1;

                }

                else
                {/// is 땅
                    pta->objects[num_vec[index]].state.is_ground = 0;

                }


            }

            else
            {


                if (pta->objects[num_vec[index]].point.z > -0.2)
                {
                    pta->objects[num_vec[index]].state.is_ground = 1;
                    continue;
                }

                if (distance_two_f(pta->objects[num_vec[index - 1]].point, pta->objects[num_vec[index]].point) == true)
                {
                    pta->objects[num_vec[index]].state.is_ground = 1;
                    continue;

                }


                double alpha = asinf((pta->objects[num_vec[index]].point.z - pta->objects[num_vec[index - 1]].point.z)
                                     / distance_two(pta->objects[num_vec[index - 1]].point,
                                                    pta->objects[num_vec[index]].point));
                alpha = alpha * 180.0 / M_PI;

                if (alpha >= ALPHA_MAX_)
                {
                    pta->objects[num_vec[index]].state.is_ground = 1;
                    continue;

                }

                if (pta->objects[num_vec[index - 1]].state.is_ground == 0)
                {///전점이 땅일때...
                    if (alpha < SET_GROUND_DEG)
                    {
                        pta->objects[num_vec[index]].state.is_ground = 0;


                    }

                    else
                    {
                        pta->objects[num_vec[index]].state.is_ground = 1;
                    }


                }

                else/// 점 점 ㅇ ㅣ 오 브 제 트
                {

                    if (pta->objects[num_vec[index]].point.z > pta->objects[num_vec[index - 1]].point.z + 0.2)
                    {
                        pta->objects[num_vec[index]].state.is_ground = 1;
                        continue;
                    }

                    else if (alpha < 1.5 && alpha > -1.5 && pta->objects[num_vec[index]].point.z < -1.2 &&
                               pta->objects[num_vec[index]].point.z < -1.2)
                    {
                        pta->objects[num_vec[index]].state.is_ground = 0;


                    }

                    else if (pta->objects[num_vec[index]].point.z < pta->objects[num_vec[index - 1]].point.z &&
                               pta->objects[num_vec[index]].point.z < -1.2)
                    {
                        pta->objects[num_vec[index]].state.is_ground = 0;

                    }


                }
            }
        }
    }


//    void col_vec_convert(std::vector<pharos_vlp_tilt::perfectarray> &vec, std::vector<int> &num_vec)
//    {
//        for (int o = 0; o < num_vec.size(); o++)
//        {
//            vec[input->objects[o].info.laser].objects.push_back(input->objects[o]);
//        }
//    }


    void row_convert (std::vector<pharos_vlp_tilt::perfectarray> &vec, pharos_vlp_tilt::vector_perfect_arrayPtr &input, pharos_vlp_tilt::perfectarrayPtr output, int &num)
    {
        for (int j = 0; j < input->one.size(); j++)
        {

            for (int i = 0; i < input->one[j].objects.size(); i++)
            {
                vec[input->one[j].objects[i].info.hori - 1].objects.push_back(input->one[j].objects[i]);
            }
        }



        for (int m = 0; m < num ; m++)
        {

            for (int i = 0; i < vec[m].objects.size(); i++)
            {
                output->objects.push_back(vec[m].objects[i]);
            }
        }
    }



    void col_same_state(pharos_vlp_tilt::perfectarrayPtr &expand, std::vector<PointType> &bank)
    {
        int inten = 10;
        PointType property;
        double center_x(0) ,center_y(0), center_z(0);
//        std::cout << "a0"  << std::endl;
        for (int i = 0; i < 16; i++)
        {
            for (int j = 0; j < expand->objects[expand->objects.size() - 1].info.hori; j++)
            {
//                std::cout << "a1"  << std::endl;


                if(j == 0)
                {
//                    std::cout << "a2"  << std::endl;
//                    vec[i].objects[j].point.intensity = inten;
                    if(expand->objects[i + 16 * j].state.is_del == 0)
                    {
                        property.index.push_back(i + 16 * j);
                        if(isIntensity == true)
                        {
                            expand->objects[i + 16 * j].point.intensity = inten;
                        }

                        property.min_index = i + 16 * j;

                        center_x += expand->objects[i + 16 * j].point.x;
                        center_y += expand->objects[i + 16 * j].point.y;
                        center_z += expand->objects[i + 16 * j].point.z;
                    }

                    else
                    {
                        continue;
                    }
                }

                else if(expand->objects[i + 16 * (j - 1)].state.is_del == 1
                        && expand->objects[i + 16 * j].state.is_del == 0)
                {
                    property.index.push_back(i + 16 * j);
                    if(isIntensity == true)
                    {
                        expand->objects[i + 16 * j].point.intensity = inten;
                    }

                    property.min_index = i + 16 * j;

                    center_x += expand->objects[i + 16 * j].point.x;
                    center_y += expand->objects[i + 16 * j].point.y;
                    center_z += expand->objects[i + 16 * j].point.z;
                }

                else if(j == expand->objects[expand->objects.size() - 1].info.hori - 1
                        && expand->objects[i + 16 * j].state.is_del == 0)
                {
//                    std::cout << "a3"  << std::endl;
//                    vec[i].objects[j].point.intensity = inten;
                    property.index.push_back(i + 16 * j);
                    if(isIntensity == true)
                    {
                        expand->objects[i + 16 * j].point.intensity = inten;
                    }

                    property.max_index = i + 16 * j;

                    property.center_pt.x = center_x/property.index.size();
                    property.center_pt.y = center_y/property.index.size();
                    property.center_pt.z = center_z/property.index.size();

                    bank.push_back(property);
                    property.index.clear();

                    center_x = 0, center_y = 0, center_z = 0;
//                    std::cout << vec[i].objects[j].state.is_ground  << std::endl;
                }

                else if(expand->objects[i + 16 * (j - 1)].state.is_ground == expand->objects[i + 16 * j].state.is_ground
                        && distance_two(expand->objects[i + 16 * j].point, expand->objects[i + 16 * (j - 1)].point) < 0.3
                        && expand->objects[i + 16 * (j - 1)].state.is_del == 0
                        && expand->objects[i + 16 * j].state.is_del == 0)
                {
//                    std::cout << "a4"  << std::endl;
                    if(isIntensity == true)
                    {
                        expand->objects[i + 16 * j].point.intensity = inten;
                    }
                    property.index.push_back(i + 16 * j);

                    center_x += expand->objects[i + 16 * j].point.x;
                    center_y += expand->objects[i + 16 * j].point.y;
                    center_z += expand->objects[i + 16 * j].point.z;
                }

                else
                {
                    if(expand->objects[i + 16 * (j - 1)].state.is_del == 0)
                    {
//                    std::cout << "a5"  << std::endl;
                        property.max_index = i + 16 * (j - 1);

                        property.center_pt.x = center_x/property.index.size();
                        property.center_pt.y = center_y/property.index.size();
                        property.center_pt.z = center_z/property.index.size();

                        bank.push_back(property);

                        property.index.clear();

                        center_x = 0, center_y = 0, center_z = 0;

                    inten += 10;

                    if (inten >= 250)
                        {
                          inten = 10;
                        }
                    }

                    if(expand->objects[i + 16 * j].state.is_del == 0)
                    {
                        if(isIntensity == true)
                        {
                            expand->objects[i + 16 * j].point.intensity = inten;
                        }
                        property.index.push_back(i + 16 * j);

                        property.min_index = i + 16 * j;

                        center_x += expand->objects[i + 16 * j].point.x;
                        center_y += expand->objects[i + 16 * j].point.y;
                        center_z += expand->objects[i + 16 * j].point.z;
                    }
                }
            }
        }
    }

    std::vector<int> check_state(pharos_vlp_tilt::perfectarrayPtr &expand, std::vector<PointType> &bank)
    {
        std::vector<int> save_i;

        for (int j = 0; j < bank.size(); j++)
        {
            if(expand->objects[bank[j].index[0]].state.is_ground == 0
            && distance_two(expand->objects[bank[j].min_index].point, expand->objects[bank[j].max_index].point) < 2)
            {
                save_i.push_back(j);

                for (int i = 0; i < bank[j].index.size(); i++)
                {
                    expand->objects[bank[j].index[i]].state.is_ground = 1;
                }
            }

        }

        for (int num = 0; num < save_i.size(); num++)
        {
            if (bank[save_i[num]].index.size() > 50)
            {
                    for (int i = 0; i < bank[save_i[num]].index.size(); i++)
                    {
                        expand->objects[bank[save_i[num]].index[i]].state.is_ground = 0;
                    }
                    continue;
            }

            for (int j = 0; j < bank[save_i[num]].index.size(); j++)
            {
                if(expand->objects[bank[save_i[num]].index[j]].point.z < CAR_HEIGHT_ + 0.05)
                {
                    expand->objects[bank[save_i[num]].index[j]].state.is_ground = 0;
                }
            }

            if (distance_two(expand->objects[bank[save_i[num]].min_index].point, expand->objects[bank[save_i[num]].max_index].point) < 2)
            {
                double alpha = asinf((expand->objects[bank[save_i[num]].max_index].point.z - expand->objects[bank[save_i[num]].min_index].point.z)
                                     / distance_two(expand->objects[bank[save_i[num]].min_index].point,
                                             expand->objects[bank[save_i[num]].max_index].point));
                alpha = alpha * 180.0 / M_PI;

                if(fabs(alpha) < 5)
                {
                    for (int i = 0; i < bank[save_i[num]].index.size(); i++)
                    {
                        expand->objects[bank[save_i[num]].index[i]].state.is_ground = 0;
                    }
                }
            }


            if (distance_two(expand->objects[bank[save_i[num]].min_index].point, expand->objects[bank[save_i[num]].max_index].point) > 4)
            {
                for (int i = 0; i < bank[save_i[num]].index.size(); i++)
                {
                    expand->objects[bank[save_i[num]].index[i]].state.is_ground = 0;
                }
                continue;
            }
        }

        return save_i;

    }



    void least_square_method(pharos_vlp_tilt::perfectarrayPtr &expand, std::vector<PointType> &bank, pharos_vlp_tilt::vector_perfect_arrayPtr &l_sol_f)
    {

        for (int j = 0; j < bank.size(); j++)
        {
            if(expand->objects[bank[j].index[0]].state.is_ground == 1)
            {
                if (expand->objects[bank[j].min_index].point.z < CAR_HEIGHT_+ 0.1 || expand->objects[bank[j].max_index].point.z < CAR_HEIGHT_ + 0.1)
                {

                    pharos_vlp_tilt::perfectarrayPtr least_array_full(new pharos_vlp_tilt::perfectarray);

                    Eigen::MatrixXd least_square_x_full(bank[j].index.size(), 2);
                    Eigen::VectorXd least_square_y_full(bank[j].index.size(), 1);
                    Eigen::Vector2d sol_full;
                    Eigen::MatrixXd persudo_inverse_full(bank[j].index.size(), 2);

                    for (int l = 0; l < bank[j].index.size(); l++)
                    {
                        Eigen::VectorXd least(1);


                        push_back_2(least_square_x_full, Eigen::Vector2d(expand->objects[bank[j].index[l]].point.x, 1), l);


                        least << expand->objects[bank[j].index[l]].point.y;


                        push_back_1(least_square_y_full, least, l);

                    }


                    persudo_inverse_full = (least_square_x_full.transpose() * least_square_x_full).inverse() *
                                           least_square_x_full.transpose();

                    sol_full = persudo_inverse_full * least_square_y_full;

                    for (int k = 0; k < bank[j].index.size(); k++)
                    {

                        pharos_vlp_tilt::perfectPtr least_pt_full(new pharos_vlp_tilt::perfect);

                        least_pt_full->point.x = expand->objects[bank[j].index[k]].point.x;
                        least_pt_full->point.y = sol_full[0] * expand->objects[bank[j].index[k]].point.x +
                                                 sol_full[1];
                        least_pt_full->point.z = expand->objects[bank[j].index[k]].point.z;

                        least_pt_full->point.intensity = expand->objects[bank[j].index[k]].point.intensity;

                        least_pt_full->state.is_ground = expand->objects[bank[j].index[k]].state.is_ground;

                        least_array_full->objects.push_back(*least_pt_full);


                    }

                    l_sol_f->one.push_back(*least_array_full);
                }
                else
                {
                    pharos_vlp_tilt::perfectarrayPtr dummy_collect(new pharos_vlp_tilt::perfectarray);
                    pharos_vlp_tilt::perfectPtr dummy_data(new pharos_vlp_tilt::perfect);
                    dummy_data->state.is_del = 1;
                    dummy_collect->objects.push_back(*dummy_data);
                    l_sol_f->one.push_back(*dummy_collect);
                }
            }
            else
            {
                pharos_vlp_tilt::perfectarrayPtr dummy_collect(new pharos_vlp_tilt::perfectarray);
                pharos_vlp_tilt::perfectPtr dummy_data(new pharos_vlp_tilt::perfect);
                dummy_data->state.is_del = 1;
                dummy_collect->objects.push_back(*dummy_data);
                l_sol_f->one.push_back(*dummy_collect);
            }




        }
    }


    void check_deviation (pharos_vlp_tilt::perfectarrayPtr &expand, std::vector<PointType> &bank, pharos_vlp_tilt::vector_perfect_arrayPtr &l_sol_f, std::vector<int> save_i)
    {

        for (int i = 0; i < save_i.size(); i++)
        {
            if(expand->objects[bank[save_i[i]].index[0]].state.is_ground == 1)
            {

                if(bank[save_i[i]].index.size() == 1)
                {
                    expand->objects[bank[save_i[i]].index[0]].state.is_ground = 0;
                }

                else if(bank[save_i[i]].index.size() == 2)
                {

                    if(distance_two(expand->objects[bank[save_i[i]].min_index].point, expand->objects[bank[save_i[i]].max_index].point) > 0.3)
                    {
                        for (int j = 0; j < bank[save_i[i]].index.size(); j++)
                        {
                            if (fabs(l_sol_f->one[save_i[i]].objects[j].point.y - expand->objects[bank[save_i[i]].index[j]].point.y) < 0.1)
                            {
                                expand->objects[bank[save_i[i]].index[j]].state.is_ground = 0;
                            }
                        }
                    }
                }

                else if(bank[save_i[i]].index.size() == 3)
                {

                    if(distance_two(expand->objects[bank[save_i[i]].min_index].point, expand->objects[bank[save_i[i]].max_index].point) > 0.5)
                    {
                        for (int j = 0; j < bank[save_i[i]].index.size(); j++)
                        {
                            if (fabs(l_sol_f->one[save_i[i]].objects[j].point.y - expand->objects[bank[save_i[i]].index[j]].point.y) < 0.1)
                            {
                                expand->objects[bank[save_i[i]].index[j]].state.is_ground = 0;
                            }
                        }
                    }
                }

                else if(bank[save_i[i]].index.size() == 4)
                {

                    if(distance_two(expand->objects[bank[save_i[i]].min_index].point, expand->objects[bank[save_i[i]].max_index].point) > 0.8)
                    {
                        for (int j = 0; j < bank[save_i[i]].index.size(); j++)
                        {
                            if (fabs(l_sol_f->one[save_i[i]].objects[j].point.y - expand->objects[bank[save_i[i]].index[j]].point.y) < 0.1)
                            {
                                expand->objects[bank[save_i[i]].index[j]].state.is_ground = 0;
                            }
                        }
                    }
                }

                else
                {

                    if(distance_two(expand->objects[bank[save_i[i]].min_index].point, expand->objects[bank[save_i[i]].max_index].point) > 1.2)
                    {
                        int count = 0;

                        for (int j = 0; j < bank[save_i[i]].index.size(); j++)
                        {
                            if (fabs(l_sol_f->one[save_i[i]].objects[j].point.y - expand->objects[bank[save_i[i]].index[j]].point.y) < 0.1)
                            {
                                count++;
                            }
                            if(count > 4)
                            {
                                for (int k = 0; k < bank[save_i[i]].index.size(); k++)
                                {
                                    expand->objects[bank[save_i[i]].index[k]].state.is_ground = 0;
                                }
                                break;
                            }
                        }
                    }
                }

            }
        }
    }

    void RemoveGround(const velodyne_msgs::custompointPtr &input)
    {
        diff_value = ros::Time::now().toSec();

        ros::Time begin = ros::Time::now();

        std::vector<pharos_vlp_tilt::perfectarray> vec_col(16);
        ///
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_object_data(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ground_data(new pcl::PointCloud<pcl::PointXYZI>);
        ///

//        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_least_full_data(new pcl::PointCloud<pcl::PointXYZI>);

        pharos_vlp_tilt::perfectarrayPtr point_bank (new pharos_vlp_tilt::perfectarray);
        std::vector<int> index_bank;
        pharos_vlp_tilt::perfectPtr point_ (new pharos_vlp_tilt::perfect);
        pharos_vlp_tilt::infoPtr info_ (new pharos_vlp_tilt::info);
        pharos_vlp_tilt::perfectarrayPtr expand_bank(new pharos_vlp_tilt::perfectarray);
        std::vector<PointType> property_bank;

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
                int full_data[16];

                for(int set_n =0; set_n < 16; set_n++)
                {
                    full_data[set_n] = -1;
                }

                DoDistinction_row(index_bank, point_bank);
//                col_vec_convert(vec_col, index_bank);

                for(int j =0; j < index_bank.size(); j++)
                {
                    full_data[point_bank->objects[index_bank[j]].info.laser] = index_bank[j];
                }

                for(int num=0; num< 16; num++ )
                {
                    if(full_data[num] != -1 ) /// @won 데이터가 살려져 있는것;
                    {
                        point_bank->objects[ full_data[num] ].state.is_del = 0;
                        point_bank->objects[ full_data[num] ].state.is_infect = 0;
                        expand_bank->objects.push_back(point_bank->objects[ full_data[num] ]);
                    }
                    else
                    {
                        pharos_vlp_tilt::perfect pt;
                        pt.state.is_del = 1;
                        pt.state.is_infect = 0;
                        pt.info.hori = hori;
                        expand_bank->objects.push_back(pt);
                    }
                }

                index_bank.clear();
                hori++;
            }
        }

//        std::cout << "expand" << "\t" << expand_bank->objects.size()  << std::endl;
//        std::cout << "point_bank" << "\t" << point_bank->objects.size()  << std::endl;
//        std::cout << "input" << "\t" << input->cpoints.size()  << std::endl;
//        std::cout << "expand" << "\t" << expand_bank->objects[expand_bank->objects.size() - 1].info.hori  << std::endl;
//        std::cout << "point_bank" << "\t" << point_bank->objects[point_bank->objects.size() - 1].info.hori  << std::endl;
//        std::cout << "input" << "\t" << input->infos[input->infos.size() - 1].hori  << std::endl;


        col_same_state(expand_bank, property_bank);  /// 0.009

        std::vector<int> save_ground;

        save_ground = check_state(expand_bank, property_bank); /// 0.0004

        pharos_vlp_tilt::vector_perfect_arrayPtr least_sol_full(new pharos_vlp_tilt::vector_perfect_array);

        least_square_method(expand_bank, property_bank, least_sol_full); /// 0.004


        check_deviation(expand_bank, property_bank, least_sol_full, save_ground); /// 0.0001


        pharos_vlp_tilt::perfectarrayPtr output(new pharos_vlp_tilt::perfectarray); /// 0.006

            for (int i = 0; i < expand_bank->objects.size(); i++)
            {
                ///test /// 위에꺼 잘나오는지

                if (expand_bank->objects[i].state.is_del == 0 && expand_bank->objects[i].state.is_ground == 1)
                {
                    pcl::PointXYZI pt1;
                    pt1.x = expand_bank->objects[i].point.x;
                    pt1.y = expand_bank->objects[i].point.y;
                    pt1.z = expand_bank->objects[i].point.z;
                    pt1.intensity = expand_bank->objects[i].point.intensity;
                    cloud_object_data->push_back(pt1);

                    if(expand_bank->objects[i].point.z <= SetUnderZ)
                    {
                        output->objects.push_back(expand_bank->objects[i]);
                    }

                }

            }
            ////
            for (int i = 0; i < expand_bank->objects.size(); i++)
            {
                ///test /// 위에꺼 잘나오는지

                if (expand_bank->objects[i].state.is_del == 0 && expand_bank->objects[i].state.is_ground == 0 && expand_bank->objects[i].point.z > -2.8)
                {
                    pcl::PointXYZI pt1;
                    pt1.x = expand_bank->objects[i].point.x;
                    pt1.y = expand_bank->objects[i].point.y;
                    pt1.z = expand_bank->objects[i].point.z;
                    pt1.intensity = expand_bank->objects[i].point.intensity;
                    cloud_ground_data->push_back(pt1);
                }

            }
//
//        for (int k = 0; k < least_sol_full->one.size(); k++)
//        {
//
//            for (int i = 0; i < least_sol_full->one[k].objects.size(); i++)
//            {
//                ///test /// 위에꺼 잘나오는지
//
//                pcl::PointXYZI pt1;
//                pt1.x = least_sol_full->one[k].objects[i].point.x;
//                pt1.y = least_sol_full->one[k].objects[i].point.y;
//                pt1.z = least_sol_full->one[k].objects[i].point.z;
//                pt1.intensity = least_sol_full->one[k].objects[i].point.intensity;
//                cloud_least_full_data->push_back(pt1);
//
//            }
//            //////////////////////
//        }


        sensor_msgs::PointCloud2Ptr object_sensor(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_object_data,*object_sensor);

        object_sensor->header.stamp = input->header.stamp;
        object_sensor->header.frame_id =frame_remove;
        pub_object.publish(*object_sensor);

        sensor_msgs::PointCloud2Ptr ground_sensor(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*cloud_ground_data,*ground_sensor);

        ground_sensor->header.stamp = input->header.stamp;
        ground_sensor->header.frame_id =frame_remove;
        pub_ground.publish(*ground_sensor);

        output->header.stamp = input->header.stamp;
        output->header.frame_id =frame_remove;
        pub_msgs.publish(*output);

//        sensor_msgs::PointCloud2Ptr test_least_full_sensor(new sensor_msgs::PointCloud2);
//        pcl::toROSMsg(*cloud_least_full_data,*test_least_full_sensor);
//
//        test_least_full_sensor->header.stamp = input->header.stamp;
//        test_least_full_sensor->header.frame_id =frame_remove;
//        pub_least_full_test.publish(*test_least_full_sensor);

        ros::Time last = ros::Time::now();

        diff_value = last.toSec() - begin.toSec();

        if(isTimeLeft == true)
        {
            ROS_INFO("remove_L time : %lf", diff_value);
        }
}




private:
    ros::Subscriber sub_set;

    ros::Publisher pub_ground;
    ros::Publisher pub_object;
    ros::Publisher pub_msgs;

//    ros::Publisher pub_least_full_test;


};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vlpt_remove_ground_L");

    ROS_INFO("started vlpt_remove_ground_L");

    vlp_ ground;

    ros::spin();

}
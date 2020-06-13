//
// Created by won on 30/8/18.
//

//
// *Created by won on 7/8/18.
// *
// *
#include <ros/ros.h>
#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
///*my custom msg
#include <velodyne_msgs/info.h>
#include <velodyne_msgs/custompoint.h>
#include <velodyne_msgs/cpoint.h>
///
#include <pharos_msgs/pointtype.h>
#include <pharos_msgs/pointtypes.h>
///전역변수///
double T_PX , T_MX , T_PY ,T_MY , T_PZ, T_MZ;
double R_PX , R_MX , R_PY ,R_MY , R_PZ, R_MZ;
double T_MOVE_Y , R_MOVE_Y;

float T_Xd , T_Yd , T_Zd;
float R_Xd , R_Yd , R_Zd;
double CAR_HEIGHT;

double ss;
double ee;
double current_sec = 0;

bool pitch_calibration;
bool isTimeTop;

double diff_value;

std::string sub_set;
std::string pub_set;
std::string pub_set_cloud;

std::string frame_set;
///------////

using namespace std;
class VLP16_
{
public:

    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;


    VLP16_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));



        nh->param<bool>("pitch_calibration",pitch_calibration, true);
        nh->param<bool>("isTimeTop",isTimeTop, false);

        /// *Left
        pnh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_roll",T_Xd, 0.0);
        pnh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_pitch",T_Yd, 0);
        pnh->param<float>("/pharos_tf_broadcaster_node/T_velodyne_yaw",T_Zd, 0.0);

        pnh->param<double>("T_PLUS_X",T_PX, 0);
        pnh->param<double>("T_MINUS_X",T_MX, 0);
        pnh->param<double>("T_PLUS_Y",T_PY, 0);
        pnh->param<double>("T_MINUS_Y",T_MY, 0);
        pnh->param<double>("T_PLUS_Z",T_PZ, 0);
        pnh->param<double>("T_MINUS_X",T_MZ, 0);
        pnh->param<double>("T_MOVE_Y",T_MOVE_Y, 0);

        pnh->param<double>("CAR_HEIGHT",CAR_HEIGHT, 0);


        pnh->param<std::string>("sub_set",sub_set, "mid_my_msg");
        pnh->param<std::string>("frame_set",frame_set, "mid_velodyne2");
        pnh->param<std::string>("pub_set",pub_set, "m_object");
        pnh->param<std::string>("pub_set_cloud",pub_set_cloud, "pub_set_cloud");

        /// *Right
//        nh->param<float>("/pharos_tf_broadcaster_node/R_velodyne_roll",R_Xd, 15.0);
//        nh->param<float>("/pharos_tf_broadcaster_node/R_velodyne_pitch",R_Yd, 0);
//        nh->param<float>("/pharos_tf_broadcaster_node/R_velodyne_yaw",R_Zd, -11.135);

//pnh->param<double>("R_PLUS_X",R_PX, 0);
      //  pnh->param<double>("R_MINUS_X",R_MX, 0);
///pnh->param<double>("R_PLUS_Y",R_PY, 0);
     //   pnh->param<double>("R_MINUS_Y",R_MY, 0);
//pnh->param<double>("R_PLUS_Z",R_PZ, 0);
      //  pnh->param<double>("R_MINUS_X",R_MZ, 0);
       // pnh->param<double>("R_MOVE_Y",R_MOVE_Y, 0);

        ///서브스크라이브는 this 사용 publish는 메세지형식 이것만 잘 생각해놓기
        sub_left = nh->subscribe(sub_set,1 , &VLP16_::LeftSetVertical,this );
      //  sub_right = nh->subscribe("ns2/my_custom_point_info", 1 , &VLP16_::RightSetVertical,this);

        pub_vertical_left = nh->advertise<sensor_msgs::PointCloud2>(pub_set_cloud,10);
        pub_velo_msg = nh->advertise<velodyne_msgs::custompoint>(pub_set,10);

//        pub_vertical_right = nh->advertise<sensor_msgs::PointCloud2>("vertical_right",10);
    //    pub_veloR_msg = nh->advertise<velodyne_msgs::custompoint>("right_my_msg",10);
    }





    void LeftSetVertical(const velodyne_msgs::custompoint &left_input_custom_ )
    {///LEFTCALLBACK 함수

        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_output (new pcl::PointCloud<pcl::PointXYZI>); ///출력 만을 위한 것
        velodyne_msgs::custompoint left_input_custom = left_input_custom_;

        velodyne_msgs::cpointPtr std_pt(new velodyne_msgs::cpoint);
        velodyne_msgs::cpointPtr pt_front(new velodyne_msgs::cpoint);
        velodyne_msgs::cpointPtr pt_back(new velodyne_msgs::cpoint);

        velodyne_msgs::infoPtr info_F(new velodyne_msgs::info);
        velodyne_msgs::infoPtr info_B(new velodyne_msgs::info);

        double T_Yd_T = T_Yd;

        pt_front->x = 0;
        pt_back->x = 0;
        info_F->ring = 100;
        info_B->ring = 100;

//        double PM_X = -CAR_HEIGHT / tanf(PM_ANGLE * M_PI/180);
//        double PP_X = -CAR_HEIGHT / tanf(PP_ANGLE * M_PI/180);
//        double PM_Z = PM_X * sinf((PM_ANGLE - 15) * M_PI/180) + CAR_HEIGHT;
//        double PP_Z = PP_X * sinf((PP_ANGLE - 15) * M_PI/180) + CAR_HEIGHT;

        for (int j = 0; j < left_input_custom.cpoints.size() ; j++)
        {
            if(fabs(left_input_custom.cpoints[j].y - 0.5) < 0.03 && left_input_custom.cpoints[j].x > 1 /*&& left_input_custom.cpoints[j].z < PP_Z
            && left_input_custom.cpoints[j].z > PM_Z && left_input_custom.cpoints[j].x < PM_X && left_input_custom.cpoints[j].x > PP_X*/
            )
            {
                if(info_F->ring >= left_input_custom.infos[j].ring)
                {
                    pt_front->x = left_input_custom.cpoints[j].x;
                    pt_front->y = left_input_custom.cpoints[j].y;
                    pt_front->z = left_input_custom.cpoints[j].z;
                    info_F->ring = left_input_custom.infos[j].ring;
                }

//                if(left_input_custom.cpoints[j].z > CAR_HEIGHT + 0.1 || left_input_custom.cpoints[j].z < CAR_HEIGHT - 0.1)
//                {
//                    pt->x = left_input_custom.cpoints[j].x;
//                    pt->y = left_input_custom.cpoints[j].y;
//                    pt->z = left_input_custom.cpoints[j].z;
//                }
            }

            if(fabs(left_input_custom.cpoints[j].y - 0.5) < 0.03 && left_input_custom.cpoints[j].x < -1.5)
            {
                if(info_B->ring >= left_input_custom.infos[j].ring)
                {
                    pt_back->x = left_input_custom.cpoints[j].x;
                    pt_back->y = left_input_custom.cpoints[j].y;
                    pt_back->z = left_input_custom.cpoints[j].z;
                    info_B->ring = left_input_custom.infos[j].ring;
                }

            }
        }

        std_pt->x = 0;
        std_pt->y = 0;
        std_pt->z = CAR_HEIGHT;

        if(fabs(pt_front->z - pt_back->z) < 0.02)
        {
            std_pt->z = (pt_front->z + pt_back->z) / 2;/*pt_front->z > pt_back->z ?
                    pt_back->z : pt_front->z;*/
//            CAR_HEIGHT = std_pt->z;
        }

        float alpha_front_T = -100;
        float alpha_back_T = -100;

        if(pt_front->x != 0 && pt_back->x != 0)
        {
            alpha_front_T = atanf((pt_front->z - std_pt->z) / (pt_front->x - std_pt->x));

            alpha_front_T = alpha_front_T * 180.0 / M_PI;

            alpha_back_T = atanf((std_pt->z - pt_back->z) / (std_pt->x - pt_back->x));

            alpha_back_T = alpha_back_T * 180.0 / M_PI;
        }

        if(fabs(alpha_front_T - alpha_back_T) < 2 && alpha_front_T != -100/* && fabs(alpha_pitch_R) < 10*/)
        {
            T_Yd_T = (alpha_front_T + alpha_back_T) / 2;
        }
        if(fabs(alpha_front_T) > 5 || fabs(alpha_back_T) > 5)
        {
            T_Yd_T = T_Yd;
        }

        if(pitch_calibration == false)
        {
            T_Yd_T = T_Yd;
        }

//        std::cout << "CAR_HEIGHT_T = "<< CAR_HEIGHT << std::endl;
//        std::cout << "T_Yd_T = "<< T_Yd_T << std::endl;
//        std::cout << "std_pt->z = "<< std_pt->z << std::endl;
//        std::cout << "ex_front = " << std::endl;
//        std::cout << "pt_front->x = " << pt_front->x << std::endl;
//        std::cout << "pt_front->y = " << pt_front->y << std::endl;
//        std::cout << "pt_front->z = " << pt_front->z << std::endl;
//        std::cout << "ex_back = " << std::endl;
//        std::cout << "pt_back->x = " << pt_back->x << std::endl;
//        std::cout << "pt_back->y = " << pt_back->y << std::endl;
//        std::cout << "pt_back->z = " << pt_back->z << std::endl;

        /// *important Matrix info
        float C_R_Matrix[3][3];
        C_R_Matrix[0][0] = cos(T_Zd*M_PI/180)*cos(T_Yd_T*M_PI/180);
        C_R_Matrix[0][1] = -sin(T_Zd*M_PI/180)*cos(T_Xd*M_PI/180) + cos(T_Zd*M_PI/180)*sin(T_Yd_T*M_PI/180)*sin(T_Xd*M_PI/180);
        C_R_Matrix[0][2] = sin(T_Zd*M_PI/180)*sin(T_Xd*M_PI/180) + cos(T_Zd*M_PI/180)*sin(T_Yd_T*M_PI/180)*cos(T_Xd*M_PI/180);
        C_R_Matrix[1][0] = sin(T_Zd*M_PI/180)*cos(T_Xd*M_PI/180);
        C_R_Matrix[1][1] = cos(T_Zd*M_PI/180)*cos(T_Xd*M_PI/180) + sin(T_Zd*M_PI/180)*sin(T_Yd_T*M_PI/180)*sin(T_Xd*M_PI/180);
        C_R_Matrix[1][2] = -cos(T_Zd*M_PI/180)*sin(T_Xd*M_PI/180) + sin(T_Zd*M_PI/180)*sin(T_Yd_T*M_PI/180)*cos(T_Xd*M_PI/180);
        C_R_Matrix[2][0] = -sin(T_Yd_T*M_PI/180);
        C_R_Matrix[2][1] = cos(T_Yd_T*M_PI/180)*sin(T_Xd*M_PI/180);
        C_R_Matrix[2][2] = cos(T_Yd_T*M_PI/180)*cos(T_Xd*M_PI/180);



        std::vector<int> save_i_info; ///수직 성분 바뀌기 전에 한번 i정보를 다 저장해주는것.
        std::vector<int> save_set_i;
        save_set_i.clear();

        int set_hori = 1;


        for(int i=0; i<left_input_custom.cpoints.size(); i++)
        {
            using namespace std;

            save_i_info.push_back(i);





            if( left_input_custom.infos[i].hori != left_input_custom.infos[i+1].hori || i+1 == left_input_custom.cpoints.size() ) ///변화하는 순간이므로. ring을 재 설정해주자.
//                cout<<"changed"<<endl;
            {
                int A[16];

                for(int array_n=0; array_n<16; array_n++)
                {
                    A[array_n] = -1;
                }   ///@won 단순한 초기값 정렬
                    ///여기서 array_n에는 링의 숫자가 그리고 대입값에는 i값을 넣어야 한다.

                for(int set_n=0; set_n < save_i_info.size(); set_n++)
                {
                    A[ left_input_custom.infos[ save_i_info[set_n] ].ring ]
                            = save_i_info[set_n];

                    left_input_custom.infos[save_i_info[set_n]].hori = set_hori;
                }

                set_hori++; ///@won 수평 성분 +해준다.

                for(int next_array_n = 0; next_array_n < 16; next_array_n++)
                {
                    if(A[next_array_n] != -1)
                    save_set_i.push_back( A[next_array_n] );
                }

                save_i_info.clear();
            }

        }/// @won 여기를 나오면 save_set_i_push_back은 i가 나열되어있는데 그 값들은 뒤죽박죽이다 그냥 save_set_i 를 순서대로 값들을 넣으면 된다.

        velodyne_msgs::custompoint new_my_custom_info;

        for(int i=0; i < save_set_i.size(); i++)
        {
            velodyne_msgs::cpoint set_point;
            velodyne_msgs::info set_info;
            pcl::PointXYZI pt;

            set_point.x = (C_R_Matrix[0][0]*left_input_custom.cpoints.at(save_set_i[i]).x) + (C_R_Matrix[0][1]*left_input_custom.cpoints.at(save_set_i[i]).y) +
                          (C_R_Matrix[0][2]*left_input_custom.cpoints.at(save_set_i[i]).z);

            set_point.y = (C_R_Matrix[1][0]*left_input_custom.cpoints.at(save_set_i[i]).x) + (C_R_Matrix[1][1]*left_input_custom.cpoints.at(save_set_i[i]).y) +
                          (C_R_Matrix[1][2]*left_input_custom.cpoints.at(save_set_i[i]).z);

            set_point.z = (C_R_Matrix[2][0]*left_input_custom.cpoints.at(save_set_i[i]).x) + (C_R_Matrix[2][1]*left_input_custom.cpoints.at(save_set_i[i]).y) +
                          (C_R_Matrix[2][2]*left_input_custom.cpoints.at(save_set_i[i]).z);
            set_point.intensity = left_input_custom.cpoints.at(save_set_i[i]).intensity;

            set_info = left_input_custom.infos.at( save_set_i[i] );

            /// @won Right Frame
            if(set_point.x <= T_PX && set_point.x >=T_MX && set_point.y <=T_PY && set_point.y >= T_MY && set_point.z <=T_PZ && set_point.z >=T_MZ )
            {}
            else
            {
                if(set_point.z <= -2.2)
                {}
                else {
                    new_my_custom_info.infos.push_back(set_info);
                    new_my_custom_info.cpoints.push_back(set_point);
                }
                pt.x = set_point.x;
                pt.y = set_point.y;
                pt.z = set_point.z;
                pt.intensity = set_point.intensity;
                pcl_output->push_back(pt);
            }

        }







        /// @won 순서대로 나오는지 보자 어디.

//        for(int i=0; i < new_my_custom_info.cpoints.size(); i++)
//        {
//            cout<<new_my_custom_info.infos[i].ring<<"\t"<<new_my_custom_info.infos[i].hori<<endl;
//        }


        sensor_msgs::PointCloud2 sensor_output;
        pcl::toROSMsg(*pcl_output,sensor_output);
        /// @won 여기서부터는 new my_custom_info에는 나열된 정보들이 들어가있다.



        sensor_output.header.frame_id = frame_set;
        sensor_output.header.stamp = left_input_custom.header.stamp;

        new_my_custom_info.header.frame_id=frame_set;
        new_my_custom_info.header.stamp = left_input_custom.header.stamp;







        pub_vertical_left.publish(sensor_output);
        pub_velo_msg.publish(new_my_custom_info);

        ros::Time last = ros::Time::now();

        diff_value = last.toSec() - begin.toSec();

        if(isTimeTop == true)
        {
            ROS_INFO("set_T time : %lf", diff_value);
        }


    }



    


protected:
    ros::Subscriber sub_right;
    ros::Subscriber sub_left;
    ros::Publisher pub_vertical_left;
    ros::Publisher pub_vertical_right;

    ros::Publisher pub_velo_msg;
    ros::Publisher pub_veloR_msg;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_set_T");       //노드명 초기화



    ROS_INFO("started vlpt_set_T");
//    ROS_INFO("SUBTOPIC : ");
//    ROS_INFO("PUBTOPIC : ");

    VLP16_ hello;

    ros::spin();


}

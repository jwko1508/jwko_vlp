//
// Created by won on 7/9/18.
//



#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pharos_vlp_tilt/perfectarray.h>
#include <pharos_vlp_tilt/vector_perfect_array.h>

///전역변수///
int vlp= 16;
double THRESHOLD;
double WEIGHT;
int MIN_OBJECT_NUM;
double AfterSetHeight;

double diff_value;
bool isTimeRight;

///------////
std::string frame_seg;
std::string sub_seg;
std::string pub_seg;
std::string pub_seg_cloud;

class vlp_
{
public:

    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;


    vlp_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        nh->param<bool>("isTimeRight",isTimeRight, false);

        nh->param<double>("THRESHOLD",THRESHOLD, 0.5);
        nh->param<int>("MIN_OBJECT_NUM",MIN_OBJECT_NUM, 20);
        nh->param<double>("AfterSetHeight",AfterSetHeight,0.5);   /// 값 이하만 출력
        pnh->param<std::string>("frame_seg",frame_seg, "nan");
        pnh->param<std::string>("sub_seg",sub_seg, "nan");
        pnh->param<std::string>("pub_seg",pub_seg, "nan");
        pnh->param<std::string>("pub_seg_cloud",pub_seg_cloud, "nan");

        /// # SUBSCRIBE
        sub_left = nh->subscribe(sub_seg, 10 , &vlp_::LeftSegmentation,this);
        //sub_right = nh->subscribe("r_into_segmentation",10,&vlp_::RightSegmentation,this );

        /// #PUBLISH
        /// #POINTCLOUD
        pub_left_pointcloud_seg = nh->advertise<sensor_msgs::PointCloud2>(pub_seg_cloud,1);
//        pub_right_pointcloud_seg = nh->advertise<sensor_msgs::PointCloud2>("r_seg",1);

        /// #MY MSG
        pub_left_my_msg = nh->advertise<pharos_vlp_tilt::vector_perfect_array>(pub_seg,1);
        //    pub_right_my_msg = nh->advertise<pharos_vlp_tilt::vector_perfect_array>("my_r_seg",1);


    }
    double ReturnWeight(pharos_vlp_tilt::point before)
    {
        double L;
        L = sqrt( before.x*before.x+before.y*before.y+before.z*before.z );
        return L;

    }
    double ReturnDistance(pharos_vlp_tilt::point before , pharos_vlp_tilt::point after)
    {
        double L;

        L = sqrt( (after.x-before.x)*(after.x-before.x)+(after.y-before.y)*(after.y-before.y)+(after.z-before.z)*(after.z-before.z) );

        return L;
    }
    std::vector<int> DataInfection(std::vector<int> input_i , pharos_vlp_tilt::perfectarrayPtr &input_my_msg , pharos_vlp_tilt::perfectarrayPtr &object_output)
    {
        std::vector<int> out_put_is; ///return 할 output i들

        for(int i=0; i < input_i.size(); i++)
        {
            int way[8] = {input_i[i]-vlp,   /// @won -> 요것들이 전부 input_my_msg에 해당되는 i값이다.
                          input_i[i]-vlp+1,
                          input_i[i]+1,
                          input_i[i]+vlp+1,
                          input_i[i]+vlp,
                          input_i[i]+vlp-1,
                          input_i[i]-1,
                          input_i[i]-vlp-1,
            };

            for(int n=0; n < 8; n++)
            {///각방향에 대한 전염시키고 안된것도 뭐 알아서.
                if(n==0)
                { /// @won left
                    if( input_i[i] < vlp )
                    {/// @won sequence 가 0일때
                        way[0] = (input_i[i] % vlp ) + (input_my_msg->objects.size() -vlp );

                        if (input_my_msg->objects[ way[0] ].state.is_infect == 0 && input_my_msg->objects[ way[0]].state.is_del ==0 )
                        {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[0]].point ) <= THRESHOLD  )
                            { /// @won  이건 같은 물체라 판단하는것이다.
//                                std::cout<<"전염되는중"<<std::endl;
                                input_my_msg->objects[ way[0] ].state.is_infect = 1;
                                out_put_is.push_back(way[0]);

                                object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;

                                object_output->objects.push_back( input_my_msg->objects[way[0]]);
                            }
                        }
                    }
                    else
                    {
                        if (input_my_msg->objects[ way[0] ].state.is_infect == 0 && input_my_msg->objects[ way[0]].state.is_del ==0 )
                        {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[0]].point ) <= THRESHOLD  )
                            { /// @won  이건 같은 물체라 판단하는것이다.
                                input_my_msg->objects[ way[0] ].state.is_infect = 1;
                                out_put_is.push_back(way[0]);

                                object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                object_output->objects.push_back( input_my_msg->objects[way[0]]);
                            }
                        }

                    }
                }
                else if(n==1)
                {
                    if( input_i[i] % vlp == 15) {/// 위로 올라가는거니까 laser이 15번째면 안된다.

                    }
                    else
                    {
                        if( input_i[i] < vlp ) ///seq가 0인 지점
                        {
                            way[1] = (input_i[i] % vlp ) + (input_my_msg->objects.size() -vlp +1 );
                            if (input_my_msg->objects[way[1]].state.is_infect == 0 && input_my_msg->objects[way[1]].state.is_del == 0)
                            {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                                if (ReturnDistance(input_my_msg->objects[input_i[i]].point,input_my_msg->objects[way[1]].point) <= THRESHOLD + 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point) )
                                {
                                    input_my_msg->objects[ way[1] ].state.is_infect = 1;
                                    out_put_is.push_back(way[1]);

                                    object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                    object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                    object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                    object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                    object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                    object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                    object_output->objects.push_back( input_my_msg->objects[way[1]]);
                                }
                            }
                        }
                        else ///@won
                        {
                            if (input_my_msg->objects[way[1]].state.is_infect == 0 && input_my_msg->objects[way[1]].state.is_del == 0)
                            {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                                if (ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[way[1]].point) <= THRESHOLD + 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point))
                                {
                                    input_my_msg->objects[ way[1] ].state.is_infect = 1;
                                    out_put_is.push_back(way[1]);

                                    object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                    object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                    object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                    object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                    object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                    object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                    object_output->objects.push_back( input_my_msg->objects[way[1]]);
                                }
                            }
                        }
                    }
                }
                else if(n==2)
                {
                    if( input_i[i] % vlp == 15)
                    {/// 위로 올라가는거니까 laser이 15번째면 안된다.

                    }
                    else
                    {
                        if (input_my_msg->objects[way[2]].state.is_infect == 0 && input_my_msg->objects[way[2]].state.is_del == 0)
                        {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                            if (ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[way[2]].point) <= THRESHOLD + 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point))
                            {
                                input_my_msg->objects[ way[2] ].state.is_infect = 1;
                                out_put_is.push_back(way[2]);

                                object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                object_output->objects.push_back( input_my_msg->objects[way[2]]);
                            }
                        }
                    }
                }
                else if(n==3)
                {
                    if( input_i[i] % vlp == 15 )
                    {/// 위로 올라가는거니까 laser이 15번째면 안된다.

                    }

                    else
                    {
                        if( input_i[i]+vlp >= input_my_msg->objects.size() )
                        {
                            way[3] = (input_i[i] % vlp) + 1;
                            if (input_my_msg->objects[ way[3] ].state.is_infect == 0 && input_my_msg->objects[ way[3]].state.is_del ==0 )
                            {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                                if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[3]].point ) <= THRESHOLD + 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point) )
                                {
                                    input_my_msg->objects[ way[3] ].state.is_infect = 1;
                                    out_put_is.push_back(way[3]);

                                    object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                    object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                    object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                    object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                    object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                    object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                    object_output->objects.push_back( input_my_msg->objects[way[3]]);
                                }
                            }
                        }
                        else
                        {
                            if (input_my_msg->objects[way[3]].state.is_infect == 0 && input_my_msg->objects[way[3]].state.is_del == 0)
                            {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                                if (ReturnDistance(input_my_msg->objects[input_i[i]].point,
                                                   input_my_msg->objects[way[3]].point) <=
                                    THRESHOLD + 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point)) {
                                    input_my_msg->objects[way[3]].state.is_infect = 1;
                                    out_put_is.push_back(way[3]);

                                    object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                    object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                    object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                    object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                    object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                    object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                    object_output->objects.push_back( input_my_msg->objects[way[3]]);
                                }
                            }
                        }
                    }
                }
                else if(n==4)
                {
                    if( input_i[i]+vlp >= input_my_msg->objects.size() )
                    {///요기는 묶는 부분
                        way[4] =input_i[i] % vlp;
                        if (input_my_msg->objects[ way[4] ].state.is_infect == 0 && input_my_msg->objects[ way[4]].state.is_del ==0 )
                        {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[4]].point ) <= THRESHOLD  )
                            {
                                input_my_msg->objects[ way[4] ].state.is_infect = 1;
                                out_put_is.push_back(way[4]);

                                object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                object_output->objects.push_back( input_my_msg->objects[way[4]]);
                            }
                        }
                    }

                    else
                    { /// 끝지점이 아닌부분 그러니까
                        if (input_my_msg->objects[ way[4] ].state.is_infect == 0 && input_my_msg->objects[ way[4]].state.is_del ==0 )
                        {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[4]].point ) <= THRESHOLD  )
                            {
                                input_my_msg->objects[ way[4] ].state.is_infect = 1;
                                out_put_is.push_back(way[4]);

                                object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                object_output->objects.push_back( input_my_msg->objects[way[4]]);
                            }
                        }
                    }
                }
                else if(n==5)
                {
                    if( input_i[i] % vlp == 0 ) /// 맽밑이니까 안 봐도 된다.
                    {}
                    else
                    {
                        if( input_i[i]+vlp >= input_my_msg->objects.size() )
                        {/// 끝자락일때
                            way[5] = (input_i[i] % vlp) -1;
                            if (input_my_msg->objects[ way[5] ].state.is_infect == 0 && input_my_msg->objects[ way[5]].state.is_del ==0 )
                            {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                                if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[5]].point ) <= THRESHOLD +0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point)   )
                                {
                                    input_my_msg->objects[ way[5] ].state.is_infect = 1;
                                    out_put_is.push_back(way[5]);

                                    object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                    object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                    object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                    object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                    object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                    object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                    object_output->objects.push_back( input_my_msg->objects[way[5]]);
                                }
                            }
                        }
                        else
                        {
                            if (input_my_msg->objects[ way[5] ].state.is_infect == 0 && input_my_msg->objects[ way[5]].state.is_del ==0 )
                            {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                                if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[5]].point ) <= THRESHOLD + 0.033 * ReturnWeight(input_my_msg->objects[input_i[i]].point) )
                                {
                                    input_my_msg->objects[ way[5] ].state.is_infect = 1;
                                    out_put_is.push_back(way[5]);

                                    object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                    object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                    object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                    object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                    object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                    object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                    object_output->objects.push_back( input_my_msg->objects[way[5]]);
                                }
                            }
                        }
                    }



                }
                else if(n==6)
                {
                    if(input_i[i] % vlp ==0 )
                    {}/// 가장 아래지역 걸릴때 i가....
                    else
                    {
                        if (input_my_msg->objects[ way[6] ].state.is_infect == 0 && input_my_msg->objects[ way[6]].state.is_del ==0 )
                        {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                            if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[6]].point ) <= THRESHOLD  )
                            {
                                input_my_msg->objects[ way[6] ].state.is_infect = 1;
                                out_put_is.push_back(way[6]);

                                object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                object_output->objects.push_back( input_my_msg->objects[way[6]]);
                            }
                        }
                    }
                }
                else///7시방향
                {
                    if(input_i[i] % vlp == 0 )
                    {}/// 가장 아래지역 걸릴때 i가....
                    else
                    {
                        if( input_i[i] < 16 ) ///seq=0인 지역
                        {
                            way[7] = (input_i[i] % vlp ) + (input_my_msg->objects.size() -vlp ) -1;
                            if (input_my_msg->objects[ way[7] ].state.is_infect == 0 && input_my_msg->objects[ way[7]].state.is_del ==0 )
                            {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                                if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[7]].point ) <= THRESHOLD  )
                                {
                                    input_my_msg->objects[ way[7] ].state.is_infect = 1;
                                    out_put_is.push_back(way[7]);

                                    object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                    object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                    object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                    object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                    object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                    object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                    object_output->objects.push_back( input_my_msg->objects[way[7]]);
                                }
                            }
                        }
                        else
                        {
                            if (input_my_msg->objects[ way[7] ].state.is_infect == 0 && input_my_msg->objects[ way[7]].state.is_del ==0 )
                            {/// @won 전염도 안되었고 실제로 찍힌 것일때때 전염을 시켜주는 것이다.
                                if( ReturnDistance(input_my_msg->objects[input_i[i]].point, input_my_msg->objects[ way[7]].point ) <= THRESHOLD  )
                                {
                                    input_my_msg->objects[ way[7] ].state.is_infect = 1;
                                    out_put_is.push_back(way[7]);

                                    object_output->min_object.x = (object_output->min_object.x >= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->min_object.x;
                                    object_output->min_object.y = (object_output->min_object.y >= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->min_object.y;
                                    object_output->min_object.z = (object_output->min_object.z >= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->min_object.z;
                                    object_output->max_object.x = (object_output->max_object.x <= input_my_msg->objects[ way[n] ].point.x) ? input_my_msg->objects[ way[n] ].point.x : object_output->max_object.x;
                                    object_output->max_object.y = (object_output->max_object.y <= input_my_msg->objects[ way[n] ].point.y) ? input_my_msg->objects[ way[n] ].point.y : object_output->max_object.y;
                                    object_output->max_object.z = (object_output->max_object.z <= input_my_msg->objects[ way[n] ].point.z) ? input_my_msg->objects[ way[n] ].point.z : object_output->max_object.z;


                                    object_output->objects.push_back( input_my_msg->objects[way[7]]);
                                }
                            }
                        }
                    }
                }

            }
        }
        return out_put_is;



    }


    std::vector<pharos_vlp_tilt::perfectarray> Vlp16Segmentation(pharos_vlp_tilt::perfectarrayPtr input)
    {

        std::vector<pharos_vlp_tilt::perfectarray> all_objects;
        for(int i=0; i < input->objects.size(); i++)
        {/// @won 최종적으로 16*hori만큼의 데이터들이 정렬되어 있는 것 전부를 돌린다.
            if(input->objects[i].state.is_infect == 0 && input->objects[i].state.is_del ==0 )
            {///시작한다. 1. 전염이 되지 않고 2. 실제 찍힌 데이터 혹은 사라지지않는 데이터 일때만 초기 값을 설정해준다.
                std::vector<int> init_one_i;
                std::vector<int> multiple_i;
                input->objects[i].state.is_infect = 1;
                pharos_vlp_tilt::perfectarrayPtr one_object(new pharos_vlp_tilt::perfectarray);

                one_object->min_object.x = 1000;
                one_object->min_object.y = 1000;
                one_object->min_object.z = 1000;
                one_object->max_object.x = -1000;
                one_object->max_object.y = -1000;
                one_object->max_object.z = -1000;

                one_object->objects.push_back(input->objects[i]);

                init_one_i.push_back(i); /// @won i를 대입한다.
//                std::cout<<i<<"cjt함수안에서의 사이즈는 : "<<init_one_i.size()<<std::endl;
                multiple_i = DataInfection(init_one_i , input, one_object);
                while(multiple_i.size() != 0)
                {
                    multiple_i = DataInfection(multiple_i,input, one_object);
                }
//                std::cout<<"while다음문이 나옴"<<std::endl;

                if(one_object->objects.size() <= MIN_OBJECT_NUM)
                {}///아무것도 넣지 않는다.
                else
                {
//                    std::cout<<"else"<<std::endl;
                    all_objects.push_back(*one_object);
                }
            }


        }
//        std::cout<<"함수안에서의 사이즈는안 : "<<all_objects.size()<<std::endl;
        return all_objects;

    }

    void LeftSegmentation(const pharos_vlp_tilt::perfectarrayPtr raw_left_input){
        ///CALLBACK 함수

        diff_value = ros::Time::now().toSec();

        ros::Time begin = ros::Time::now();

        std::vector<int> save_i;
        pharos_vlp_tilt::perfectarrayPtr left_input(new pharos_vlp_tilt::perfectarray);
        left_input = raw_left_input;
        pharos_vlp_tilt::perfectarrayPtr expand_my_msg(new pharos_vlp_tilt::perfectarray);


        for(int i=0; i < left_input->objects.size(); i++)
        {
            save_i.push_back(i);    /// hori가 바뀔떄까지 i를 저장 i는 실제로 인식된 데이터 이다.
            if(left_input->objects[i].info.hori != left_input->objects[i+1].info.hori || i+1 == left_input->objects.size() )
            {
                int full_data[vlp];
                for(int set_n =0; set_n < vlp; set_n++)
                {
                    full_data[set_n] = -1;
                }
                for(int j =0; j < save_i.size(); j++)
                {
                    full_data[left_input->objects[save_i[j]].info.laser] = save_i[j];
                }

                for(int num=0; num<vlp; num++ )
                {
                    if(full_data[num] != -1 ) /// @won 데이터가 살려져 있는것;
                    {
                        left_input->objects[ full_data[num] ].state.is_del = 0;
                        left_input->objects[ full_data[num] ].state.is_infect = 0;
                        expand_my_msg->objects.push_back(left_input->objects[ full_data[num] ]);
                    }
                    else
                    {
                        pharos_vlp_tilt::perfect pt;
                        pt.state.is_del = 1;
                        pt.state.is_infect = 0;
                        expand_my_msg->objects.push_back(pt);
                    }
                }
                save_i.clear();
            }
        }
        /// *여기서 부터 전염 시키는 알고리즘을 작성하면 된다.
        std::vector<pharos_vlp_tilt::perfectarray> objects_vector;
        objects_vector = Vlp16Segmentation(expand_my_msg);
        double my_intensity = 10;
        pcl::PointCloud<pcl::PointXYZI>::Ptr segmentation_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

        pharos_vlp_tilt::vector_perfect_arrayPtr my_wonderful_output(new pharos_vlp_tilt::vector_perfect_array);

        for(int i=0; i < objects_vector.size(); i++)
        {


            pharos_vlp_tilt::perfectarrayPtr After_Z_my_msg (new pharos_vlp_tilt::perfectarray);
            for(unsigned int j=0; j < objects_vector.at(i).objects.size(); j++)
            {
                if(objects_vector.at(i).objects[j].point.z <=AfterSetHeight)
                {
                    pharos_vlp_tilt::perfect is_z;
                    is_z.point = objects_vector.at(i).objects[j].point;
                    is_z.state = objects_vector.at(i).objects[j].state;
                    is_z.info = objects_vector.at(i).objects[j].info;
                    After_Z_my_msg->objects.push_back(is_z);

                }
            }
            if(After_Z_my_msg->objects.size() != 0)

            my_wonderful_output->one.push_back(*After_Z_my_msg);

            for(int k=0; k < objects_vector.at(i).objects.size(); k++)
            {///요게 한객체 다 하는것
                objects_vector.at(i).objects[k].point.intensity = my_intensity;
                pcl::PointXYZI pt;
                pt.x = objects_vector.at(i).objects[k].point.x;
                pt.y = objects_vector.at(i).objects[k].point.y;
                pt.z = objects_vector.at(i).objects[k].point.z;
                pt.intensity = objects_vector.at(i).objects[k].point.intensity;
                segmentation_pcl_point_cloud->push_back(pt);
//                std::cout<<"x값나오냐?"<<pt.x<<std::endl;
            }
            my_intensity += my_intensity + 10;
            if(my_intensity >250)
            {
                my_intensity = 10;
            }

        }

        sensor_msgs::PointCloud2 sensor_output;
        pcl::toROSMsg(*segmentation_pcl_point_cloud,sensor_output);

        sensor_output.header.frame_id = frame_seg;
        sensor_output.header.stamp = raw_left_input->header.stamp;
        pub_left_pointcloud_seg.publish(sensor_output);

        my_wonderful_output->header.frame_id = frame_seg;
        my_wonderful_output->header.stamp = raw_left_input->header.stamp;
        pub_left_my_msg.publish(my_wonderful_output);

        ros::Time last = ros::Time::now();


        diff_value = last.toSec() - begin.toSec();

        if(isTimeRight == true)
        {
            ROS_INFO("seg_R time : %lf", diff_value);
        }

    }




protected:
    ros::Subscriber sub_left;
    ros::Subscriber sub_right;
    ros::Publisher pub_left_pointcloud_seg;
    ros::Publisher pub_right_pointcloud_seg;
    ros::Publisher pub_left_my_msg;
    ros::Publisher pub_right_my_msg;


};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_segmentation_R");       //노드명 초기화



    ROS_INFO("started vlpt_segmentation_R");
//    ROS_INFO("SUBTOPIC : ");
//    ROS_INFO("PUBTOPIC : ");

    vlp_ hello;

    ros::spin();


}
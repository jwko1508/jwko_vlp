[목차](/README.md)          |           [Previous](/docs/mdfile/clustering.md)
# 포인트 마지막 처리

포인트 마지막 처리에서는 pharos for msg 코드를 설명하겠습니다.

## 1. Segmentation

[vlpt_pharos_msg.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_pharos_msg.cpp)라는 파일을 기준으로 AllInformation 콜백함수를 보겠습니다.

이 pharos for msg 코드는 매우 간단하고 필요한 부분만 저장하는 역할을 담당하고 있습니다.
따라서 간단하게 필요한 부분만 추출하여 설명하겠습니다.

```c
pharos_vlp_tilt::vector_perfect_arrayPtr transformed_array(new pharos_vlp_tilt::vector_perfect_array);
*transformed_array = raw_input;
pharos_vlp_tilt::vector_perfect_arrayPtr base_novatel_array(new pharos_vlp_tilt::vector_perfect_array);
*base_novatel_array = raw_input;

pharos_msgs::ObjectInfoArrayPtr allobjects(new pharos_msgs::ObjectInfoArray); 

//////////////////
pcl::PointCloud<pcl::PointXYZI>::Ptr test_min_max(new pcl::PointCloud<pcl::PointXYZI>);
//////////////////


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
```
가장 처음부분은 역시 변수저장을 하는 부분입니다.

그 후 이전 Map filter에서 사용하고 Eigen 라이브러리를 이용하여 회전 변환행렬을 만들어줍니다.

```c
for (int i = 0; i < transformed_array->one.size(); i++) 
{
    for (int j = 0; j < transformed_array->one[i].objects.size(); j++) 
    {
        
        Eigen::RowVector4f odom_pose, point_xyz;

        point_xyz(0) = transformed_array->one[i].objects[j].point.x;
        point_xyz(1) = transformed_array->one[i].objects[j].point.y;
        point_xyz(2) = transformed_array->one[i].objects[j].point.z;
        point_xyz(3) = 1;

        odom_pose.transpose() = RT02 * point_xyz.transpose();
        transformed_array->one[i].objects[j].point.x = odom_pose(0);
        transformed_array->one[i].objects[j].point.y = odom_pose(1);
        transformed_array->one[i].objects[j].point.z = odom_pose(2);
    }
}
```
Lidar기준의 좌표를 절대좌표계 기준으로 바꾸는 과정입니다. 여기까지 Map filter에 있는 과정과 같습니다.

```c
if (input->one[i].objects.size() <= remove_point_size)
{

    for (int bong = 0; bong < input->one[i].objects.size(); bong++) {
        check_bong = (DistnaceXY(input->one[i].objects[0], input->one[i].objects[bong]) <=
                      Set_bong_distance) ? 0 : 1;
        if (check_bong == 0 )
            continue;

        else
            break;
    }

    if (check_bong == 0)
        continue;

}
```
이 부분은 물체이지만 사이즈가 너무 작아 물체인지 땅인지 판단할 수 없는 부분을 제거하는 if문 입니다. 즉 너무 멀리 있어 물체인데 Lidar 1채널에만 살짝 스캔된 것은 데이터상으로 아직 판단하기 부적합해 제거하는 것입니다.

```c
for (int j = 0; j < input->one[i].objects.size(); j++)
{
    OneObject_Pharos_info->pose.x += input->one[i].objects[j].point.x;
    OneObject_Pharos_info->pose.y += input->one[i].objects[j].point.y;
    OneObject_Pharos_info->pose.z += input->one[i].objects[j].point.z;

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
}

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
```
이때까지의 데이터를 종합하여 msg형태를 만들어 저장합니다.

```c

double Center_L = sqrt( OneObject_Pharos_info->pose.x*OneObject_Pharos_info->pose.x + OneObject_Pharos_info->pose.y*OneObject_Pharos_info->pose.y );
if(Center_L <= Set_Perception_Distance) 
{
    /*if( (OneObject_Pharos_info->size.max_x-OneObject_Pharos_info->size.min_x)*(OneObject_Pharos_info->size.max_y-OneObject_Pharos_info->size.min_y)*0.44 <= XY_AREA ){
    }
    else */if((OneObject_Pharos_info->size.max_x-OneObject_Pharos_info->size.min_x)*(OneObject_Pharos_info->size.max_y-OneObject_Pharos_info->size.min_y)
            *(OneObject_Pharos_info->size.max_z-OneObject_Pharos_info->size.min_z) <= Bigvolum)
    {
       
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
       *(OneObject_Pharos_info->size.max_z-OneObject_Pharos_info->size.min_z) <= Bigvolum )
    {

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

```
부피가 큰 물체거나 높이에 따라 확실히 제거를 하고 다른 파트에게 보내줄 데이터를 골라냅니다.

이렇게 하여 사용한 모든 Lidar의 과정을 설명하였습니다.
C++의 지식이 많이 부족한 때부터 코드를 작성하였기 때문에 미숙한 부분도 많이 있습니다.
이러한 부분은 좀더 연구한 후 새로 만들려고 합니다.

봐주셔서 감사합니다.


[목차](/README.md)

[목차](/README.md)          |           [Previous](/docs/mdfile/clustering.md)
# 포인트 마지막 처리

포인트 마지막 처리에서는 pharos for msg 코드를 설명하겠습니다.

## 1. Segmentation

[vlpt_pharos_msg.cpp](/src/vlp/pharos_vlp_tilt/src/vlpt_pharos_msg.cpp)라는 파일을 기준으로 AllInformation 콜백함수를 보겠습니다.

이 pharos for msg 코드는 매우 간단하고 필요한 부분만 저장하는 역할을 담당하고 있어 간단히 설명하겠습니다.

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




[목차](/README.md)

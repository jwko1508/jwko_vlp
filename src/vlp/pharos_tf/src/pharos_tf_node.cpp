#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pharos_msgs/StateStamped2016.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>


nav_msgs::Odometry Vehicle_Odom_;
nav_msgs::Odometry Novatel_Odom_;
nav_msgs::Odometry Ublox_Odom_;
nav_msgs::Odometry MCL_Odom_;
nav_msgs::Odometry EKF_Odom_;

std::string vehicle_frame;
std::string front_bumper_frame_id;
std::string rear_bumper_frame_id;
std::string T_velodyne_frame_id;
std::string L_velodyne_frame_id;
std::string R_velodyne_frame_id;
std::string camera_frame_id;

float front_bumper_roll, front_bumper_pitch, front_bumper_yaw, front_bumper_x, front_bumper_y, front_bumper_z;
float rear_bumper_roll, rear_bumper_pitch, rear_bumper_yaw, rear_bumper_x, rear_bumper_y, rear_bumper_z;
float T_velodyne_roll, T_velodyne_pitch, T_velodyne_yaw, T_velodyne_x, T_velodyne_y, T_velodyne_z;
float L_velodyne_roll, L_velodyne_pitch, L_velodyne_yaw, L_velodyne_x, L_velodyne_y, L_velodyne_z;
float R_velodyne_roll, R_velodyne_pitch, R_velodyne_yaw, R_velodyne_x, R_velodyne_y, R_velodyne_z;
float camera_x, camera_y, camera_z, camera_roll, camera_pitch, camera_yaw;

bool Vehicle = false;
bool Novatel = false;
bool Ublox = false;
bool MCL = false;
bool EKF = false;

Eigen::Vector3f getRPYfromRM(Eigen::Matrix3f rotation_matrix){
    Eigen::Vector3f rpy;
    rpy[0] = atan2f(rotation_matrix(2,1),rotation_matrix(2,2));
    rpy[1] = atan2f(-rotation_matrix(2,0),sqrt(pow(rotation_matrix(2,1),2)+pow(rotation_matrix(2,2),2)));
    rpy[2] = atan2f(rotation_matrix(1,0),rotation_matrix(0,0));

    return rpy;
}

void VehicleStateCB(const pharos_msgs::StateStamped2016Ptr& msg){
    static tf::TransformBroadcaster br;

    //Novatel Odmo tf
    tf::Transform novatel_transform;
    novatel_transform.setOrigin( tf::Vector3(Novatel_Odom_.pose.pose.position.x, Novatel_Odom_.pose.pose.position.y, Novatel_Odom_.pose.pose.position.z) );

    std::string novatel_odom_frame_id = Novatel_Odom_.header.frame_id;
    std::string novatel_odom_child_frame_id = Novatel_Odom_.child_frame_id;

    tf::Quaternion novatel_quat;
    novatel_quat.setX(Novatel_Odom_.pose.pose.orientation.x);
    novatel_quat.setY(Novatel_Odom_.pose.pose.orientation.y);
    novatel_quat.setZ(Novatel_Odom_.pose.pose.orientation.z);
    novatel_quat.setW(Novatel_Odom_.pose.pose.orientation.w);

    novatel_transform.setRotation(novatel_quat);
    if(Novatel)
        br.sendTransform(tf::StampedTransform(novatel_transform, msg->header.stamp, novatel_odom_frame_id ,novatel_odom_child_frame_id));

    //Ublox Odom tf
    tf::Transform ublox_transform;
    ublox_transform.setOrigin( tf::Vector3(Ublox_Odom_.pose.pose.position.x, Ublox_Odom_.pose.pose.position.y, Ublox_Odom_.pose.pose.position.z) );

    std::string ublox_odom_frame_id = Ublox_Odom_.header.frame_id;
    std::string ublox_odom_child_frame_id = Ublox_Odom_.child_frame_id;

    tf::Quaternion ublox_quat;
    ublox_quat.setX(Ublox_Odom_.pose.pose.orientation.x);
    ublox_quat.setY(Ublox_Odom_.pose.pose.orientation.y);
    ublox_quat.setZ(Ublox_Odom_.pose.pose.orientation.z);
    ublox_quat.setW(Ublox_Odom_.pose.pose.orientation.w);

    ublox_transform.setRotation(ublox_quat);
    if(Ublox)
        br.sendTransform(tf::StampedTransform(ublox_transform, msg->header.stamp, ublox_odom_frame_id ,ublox_odom_child_frame_id));

    //MCL Odom tf
    tf::Transform mcl_transform;
    mcl_transform.setOrigin( tf::Vector3(MCL_Odom_.pose.pose.position.x, MCL_Odom_.pose.pose.position.y, MCL_Odom_.pose.pose.position.z) );

    std::string mcl_odom_frame_id = MCL_Odom_.header.frame_id;
    std::string mcl_odom_child_frame_id = MCL_Odom_.child_frame_id;

    tf::Quaternion mcl_quat;
    mcl_quat.setX(MCL_Odom_.pose.pose.orientation.x);
    mcl_quat.setY(MCL_Odom_.pose.pose.orientation.y);
    mcl_quat.setZ(MCL_Odom_.pose.pose.orientation.z);
    mcl_quat.setW(MCL_Odom_.pose.pose.orientation.w);

    mcl_transform.setRotation(mcl_quat); 
    if(MCL)
        br.sendTransform(tf::StampedTransform(mcl_transform, msg->header.stamp, mcl_odom_frame_id ,mcl_odom_child_frame_id));

    //EKF Odom tf
    tf::Transform ekf_transform;
    ekf_transform.setOrigin( tf::Vector3(EKF_Odom_.pose.pose.position.x, EKF_Odom_.pose.pose.position.y, EKF_Odom_.pose.pose.position.z) );

    std::string ekf_odom_frame_id = EKF_Odom_.header.frame_id;
    std::string ekf_odom_child_frame_id = EKF_Odom_.child_frame_id;

    tf::Quaternion ekf_quat;
    ekf_quat.setX(EKF_Odom_.pose.pose.orientation.x);
    ekf_quat.setY(EKF_Odom_.pose.pose.orientation.y);
    ekf_quat.setZ(EKF_Odom_.pose.pose.orientation.z);
    ekf_quat.setW(EKF_Odom_.pose.pose.orientation.w);

    ekf_transform.setRotation(ekf_quat);
    if(EKF)
        br.sendTransform(tf::StampedTransform(ekf_transform, msg->header.stamp, ekf_odom_frame_id ,ekf_odom_child_frame_id));

    //Vehicle Odom tf
    tf::Transform vehicle_transform;
    vehicle_transform.setOrigin(tf::Vector3(Vehicle_Odom_.pose.pose.position.x, Vehicle_Odom_.pose.pose.position.y, Vehicle_Odom_.pose.pose.position.z) );

    std::string vehicle_odom_frame_id = Vehicle_Odom_.header.frame_id;
    std::string vehicle_odom_child_frame_id = Vehicle_Odom_.child_frame_id;

    tf::Quaternion vehicle_quat;
    vehicle_quat.setX(Vehicle_Odom_.pose.pose.orientation.x);
    vehicle_quat.setY(Vehicle_Odom_.pose.pose.orientation.y);
    vehicle_quat.setZ(Vehicle_Odom_.pose.pose.orientation.z);
    vehicle_quat.setW(Vehicle_Odom_.pose.pose.orientation.w);

    vehicle_transform.setRotation(vehicle_quat);
    if(Vehicle)
        br.sendTransform(tf::StampedTransform(vehicle_transform, msg->header.stamp, vehicle_odom_frame_id, vehicle_odom_child_frame_id));

    //Front bumper tf
    tf::Transform front_bumper_Transform;
    front_bumper_Transform.setOrigin(tf::Vector3(front_bumper_x,front_bumper_y,front_bumper_z));
    tf::Quaternion front_bumper_qt;
    front_bumper_qt.setRPY(front_bumper_roll,front_bumper_pitch,front_bumper_yaw);
    front_bumper_Transform.setRotation(front_bumper_qt);
    br.sendTransform(tf::StampedTransform(front_bumper_Transform, msg->header.stamp, "vehicle_frame", front_bumper_frame_id));

    //Rear bumper tf
    tf::Transform rear_bumper_Transform;
    rear_bumper_Transform.setOrigin(tf::Vector3(rear_bumper_x,rear_bumper_y,rear_bumper_z));
    tf::Quaternion rear_bumper_qt;
    rear_bumper_qt.setRPY(rear_bumper_roll,rear_bumper_pitch,rear_bumper_yaw);
    rear_bumper_Transform.setRotation(rear_bumper_qt);
    br.sendTransform(tf::StampedTransform(rear_bumper_Transform, msg->header.stamp, "vehicle_frame", rear_bumper_frame_id));

    //top velodyne tf
    Eigen::AngleAxisf t_rotation_x (T_velodyne_roll*M_PI/180,Eigen::Vector3f::UnitX ());
    Eigen::AngleAxisf t_rotation_y (T_velodyne_pitch*M_PI/180,Eigen::Vector3f::UnitY ());
    Eigen::AngleAxisf t_rotation_z (T_velodyne_yaw*M_PI/180,Eigen::Vector3f::UnitZ ());
    Eigen::Matrix3f t_rm;
    t_rm = t_rotation_z*t_rotation_y*t_rotation_x;
    Eigen::Vector3f t_velodyne_rpy = getRPYfromRM(t_rm);
    tf::Transform t_Velo_Transform;
    t_Velo_Transform.setOrigin(tf::Vector3(T_velodyne_x,T_velodyne_y,T_velodyne_z));
    tf::Quaternion t_velodyne_qt;
    t_velodyne_qt.setRPY(t_velodyne_rpy[0],t_velodyne_rpy[1],t_velodyne_rpy[2]);
    t_Velo_Transform.setRotation(t_velodyne_qt);
    br.sendTransform(tf::StampedTransform(t_Velo_Transform, msg->header.stamp, "vehicle_frame", T_velodyne_frame_id));

    //left velodyne tf
    Eigen::AngleAxisf l_rotation_x (L_velodyne_roll*M_PI/180,Eigen::Vector3f::UnitX ());
    Eigen::AngleAxisf l_rotation_y (L_velodyne_pitch*M_PI/180,Eigen::Vector3f::UnitY ());
    Eigen::AngleAxisf l_rotation_z (L_velodyne_yaw*M_PI/180,Eigen::Vector3f::UnitZ ());
    Eigen::Matrix3f l_rm;
    l_rm = l_rotation_z*l_rotation_y*l_rotation_x;
    Eigen::Vector3f l_velodyne_rpy = getRPYfromRM(l_rm);
    tf::Transform l_Velo_Transform;
    l_Velo_Transform.setOrigin(tf::Vector3(L_velodyne_x,L_velodyne_y,L_velodyne_z));
    tf::Quaternion l_velodyne_qt;
    l_velodyne_qt.setRPY(l_velodyne_rpy[0],l_velodyne_rpy[1],l_velodyne_rpy[2]);
    l_Velo_Transform.setRotation(l_velodyne_qt);
    br.sendTransform(tf::StampedTransform(l_Velo_Transform, msg->header.stamp, "vehicle_frame", L_velodyne_frame_id));

    //right velodyne tf
    Eigen::AngleAxisf r_rotation_x (R_velodyne_roll*M_PI/180,Eigen::Vector3f::UnitX ());
    Eigen::AngleAxisf r_rotation_y (R_velodyne_pitch*M_PI/180,Eigen::Vector3f::UnitY ());
    Eigen::AngleAxisf r_rotation_z (R_velodyne_yaw*M_PI/180,Eigen::Vector3f::UnitZ ());
    Eigen::Matrix3f r_rm;
    r_rm = r_rotation_z*r_rotation_y*r_rotation_x;
    Eigen::Vector3f r_velodyne_rpy = getRPYfromRM(r_rm);
    tf::Transform r_Velo_Transform;
    r_Velo_Transform.setOrigin(tf::Vector3(R_velodyne_x,R_velodyne_y,R_velodyne_z));
    tf::Quaternion r_velodyne_qt;
    r_velodyne_qt.setRPY(r_velodyne_rpy[0],r_velodyne_rpy[1],r_velodyne_rpy[2]);
    r_Velo_Transform.setRotation(r_velodyne_qt);
    br.sendTransform(tf::StampedTransform(r_Velo_Transform, msg->header.stamp, "vehicle_frame", R_velodyne_frame_id));

    //camera tf
    tf::Transform CameraTransform;
    CameraTransform.setOrigin(tf::Vector3(camera_x,camera_y,camera_z));
    tf::Quaternion camera_qt;
    camera_qt.setRPY(camera_roll,camera_pitch,camera_yaw);
    CameraTransform.setRotation(camera_qt);
    br.sendTransform(tf::StampedTransform(CameraTransform, msg->header.stamp, "vehicle_frame", camera_frame_id));

    //top velodyne tf
    tf::Transform transform_t;
    transform_t.setOrigin( tf::Vector3(T_velodyne_x, T_velodyne_y, T_velodyne_z) );
    tf::Quaternion t_qt;
    t_qt.setRPY(0,0,0);
    transform_t.setRotation(t_qt);
    br.sendTransform(tf::StampedTransform(transform_t, msg->header.stamp, "vehicle_frame", "top_velodyne2"));

    //left velodyne tf
    tf::Transform transform_l;
    transform_l.setOrigin( tf::Vector3(L_velodyne_x, L_velodyne_y, L_velodyne_z) );
    tf::Quaternion l_qt;
    l_qt.setRPY(0,0,0);
    transform_l.setRotation(l_qt);
    br.sendTransform(tf::StampedTransform(transform_l, msg->header.stamp, "vehicle_frame", "left_velodyne2"));

    //right velodyne tf
    tf::Transform transform_r;
    transform_r.setOrigin( tf::Vector3(R_velodyne_x, R_velodyne_y, R_velodyne_z) );
    tf::Quaternion r_qt;
    r_qt.setRPY(0,0,0);
    transform_r.setRotation(r_qt);
    br.sendTransform(tf::StampedTransform(transform_r, msg->header.stamp, "vehicle_frame", "right_velodyne2"));

}

void VehicleOdomCB(const nav_msgs::OdometryConstPtr &msg){
    Vehicle_Odom_ = *msg;
    Vehicle = true;
}
void NovatelOdomCB(const nav_msgs::OdometryConstPtr &msg){
    Novatel_Odom_ = *msg;
    Novatel = true;
}
void UbloxOdomCB(const nav_msgs::OdometryConstPtr &msg){
    Ublox_Odom_ = *msg;
    Ublox = true;
}
void MCLOdomCB(const nav_msgs::OdometryConstPtr &msg){
    MCL_Odom_ = *msg;
    MCL = true;
}
void EKFOdomCB(const nav_msgs::OdometryConstPtr &msg){
    EKF_Odom_ = *msg;
    EKF = true;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "pharos_tf_broadcaster");

    ros::NodeHandle node;
    ros::NodeHandle pnode("~");

    node.param<std::string>("vehicle_frame_id", vehicle_frame, "vehicle_frame");

    pnode.param<std::string>("front_bumper_frame_id", front_bumper_frame_id, "front_bumper");
    pnode.param<std::string>("rear_bumper_frame_id", rear_bumper_frame_id, "rear_bumper");
    pnode.param<std::string>("T_velodyne_frame_id", T_velodyne_frame_id, "top_veldoyne");
    pnode.param<std::string>("L_velodyne_frame_id", L_velodyne_frame_id, "left_veldoyne");
    pnode.param<std::string>("R_velodyne_frame_id", R_velodyne_frame_id, "right_veldoyne");
    pnode.param<std::string>("camera_frame_id", camera_frame_id, "camera");

    pnode.param<float>("front_bumper_roll", front_bumper_roll, 0.0);
    pnode.param<float>("front_bumper_pitch", front_bumper_pitch, 0.0);
    pnode.param<float>("front_bumper_yaw", front_bumper_yaw, 0.0);
    pnode.param<float>("front_bumper_x", front_bumper_x, 0.0);
    pnode.param<float>("front_bumper_y", front_bumper_y, 0.0);
    pnode.param<float>("front_bumper_z", front_bumper_z, 0.0);

    pnode.param<float>("rear_bumper_roll", rear_bumper_roll, 0.0);
    pnode.param<float>("rear_bumper_pitch", rear_bumper_pitch, 0.0);
    pnode.param<float>("rear_bumper_yaw", rear_bumper_yaw, 0.0);
    pnode.param<float>("rear_bumper_x", rear_bumper_x, 0.0);
    pnode.param<float>("rear_bumper_y", rear_bumper_y, 0.0);
    pnode.param<float>("rear_bumper_z", rear_bumper_z, 0.0);

    pnode.param<float>("T_velodyne_roll", T_velodyne_roll, 0.0);
    pnode.param<float>("T_velodyne_pitch", T_velodyne_pitch, 0.0);
    pnode.param<float>("T_velodyne_yaw", T_velodyne_yaw, 0.0);
    pnode.param<float>("T_velodyne_x", T_velodyne_x, 0.0);
    pnode.param<float>("T_velodyne_y", T_velodyne_y, 0.0);
    pnode.param<float>("T_velodyne_z", T_velodyne_z, 0.0);

    pnode.param<float>("L_velodyne_roll", L_velodyne_roll, 0.0);
    pnode.param<float>("L_velodyne_pitch", L_velodyne_pitch, 0.0);
    pnode.param<float>("L_velodyne_yaw", L_velodyne_yaw, 0.0);
    pnode.param<float>("L_velodyne_x", L_velodyne_x, 0.0);
    pnode.param<float>("L_velodyne_y", L_velodyne_y, 0.0);
    pnode.param<float>("L_velodyne_z", L_velodyne_z, 0.0);

    pnode.param<float>("R_velodyne_roll", R_velodyne_roll, 0.0);
    pnode.param<float>("R_velodyne_pitch", R_velodyne_pitch, 0.0);
    pnode.param<float>("R_velodyne_yaw", R_velodyne_yaw, 0.0);
    pnode.param<float>("R_velodyne_x", R_velodyne_x, 0.0);
    pnode.param<float>("R_velodyne_y", R_velodyne_y, 0.0);
    pnode.param<float>("R_velodyne_z", R_velodyne_z, 0.0);

    pnode.param<float>("camera_x", camera_x, 0.0);
    pnode.param<float>("camera_y", camera_y, 0.0);
    pnode.param<float>("camera_z", camera_z, 0.0);
    pnode.param<float>("camera_roll", camera_roll, 0.0);
    pnode.param<float>("camera_pitch", camera_pitch, 0.0);
    pnode.param<float>("camera_yaw", camera_yaw, 0.0);

    ros::Subscriber VehicleState_sub = node.subscribe("/vehicle/state2016", 10, &VehicleStateCB);
    ros::Subscriber Vehicle_Odom_sub = node.subscribe("/odom/vehicle", 10, &VehicleOdomCB);
    ros::Subscriber Novatel_Odom_sub = node.subscribe("/odom/novatel", 10, &NovatelOdomCB);
    ros::Subscriber Ublox_Odom_sub = node.subscribe("/odom/ublox", 10, &UbloxOdomCB);
    ros::Subscriber MCL_Odom_sub = node.subscribe("/odom/mcl", 10, &MCLOdomCB);
    ros::Subscriber EKF_Odom_sub = node.subscribe("/odom/ekf", 10, &EKFOdomCB);

    ROS_INFO("vehicle_frame : '%s'",vehicle_frame.c_str());



    ros::spin();
    return 0;
};

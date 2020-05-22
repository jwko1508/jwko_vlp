//
// Created by jwkolab on 19. 5. 21.
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pharos_vlp_tilt/vector_perfect_array.h>
#include <pharos_vlp_tilt/VehiclePoseArray.h>
#include <pharos_msgs/ObjectInfoArray.h>
#include <nav_msgs/Odometry.h>
#include <pcl/conversions.h>
///*my custom msg
#include <velodyne_msgs/info.h>
#include <velodyne_msgs/custompoint.h>
#include <velodyne_msgs/cpoint.h>
///
#include <velodyne_msgs/VelodyneScan.h>
#include <pharos_msgs/StateStamped2016.h>

#define L 2.695
#define steer_const 15.14
#define steer_bias 15

bool isTimeCenter;
double diff_value;

pharos_vlp_tilt::VehiclePoseArrayPtr vehicle_vec(new pharos_vlp_tilt::VehiclePoseArray);

std::string sub_center_left;
std::string sub_center_right;
std::string sub_center_top;
std::string sub_center_vehicle;
std::string pub_center;

bool isLeftInit = false;
bool isRightInit = false;
bool isTopInit = false;
bool isVehicleInit = false;

bool isLeftCheck = false;
bool isRightCheck = false;
bool isTopCheck = false;

int vehicle_stack;

float T_velodyne_x , T_velodyne_y , T_velodyne_z;

class vlp_
{

public:

    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;

    vlp_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        ////////////param/////////////
        ///MID
        nh->param<bool>("isTimeCenter",isTimeCenter, false);
        nh->param<int>("vehicle_stack",vehicle_stack, 25);

        /////////////sub/////////////
        pnh->param<std::string>("sub_center_left",sub_center_left, "left_my_msg");
        pnh->param<std::string>("sub_center_right",sub_center_right, "right_my_msg");
        pnh->param<std::string>("sub_center_top",sub_center_top, "top_my_msg");
        pnh->param<std::string>("sub_center_vehicle",sub_center_vehicle, "/vehicle/state2016");

        pnh->param<std::string>("pub_center",pub_center, "center_sync_msg");

        sub_left = nh->subscribe(sub_center_left,1 , &vlp_::Left_VLP_CB,this );
        sub_right = nh->subscribe(sub_center_right,1 , &vlp_::Right_VLP_CB,this );
        sub_top = nh->subscribe(sub_center_top,1 , &vlp_::Top_VLP_CB,this );
        sub_vehicle = nh->subscribe(sub_center_vehicle,10 , &vlp_::Vehicle_State_CB,this );

        pub_for_sync = nh->advertise<pharos_vlp_tilt::VehiclePoseArray>(pub_center,10);
    }

    void bicycle_model (double &x, double &y, double &theta,
                        const double &vel, const double &wheel_angle, const double &dt)
    {
        double ratio = (steer_const - steer_bias) * 3 / M_PI * wheel_angle *steer_const + steer_bias;

        double d = vel*dt;
        double beta = d / L * tan(wheel_angle*steer_const/ratio);

        x = d * cos(theta);
        y = d * sin(theta);

        theta = beta;
        double x_dot = -d * sin(theta);
        double y_dot = d * cos(theta);
    }

    void Top_VLP_CB (const velodyne_msgs::custompoint &input)
    {

        diff_value = ros::Time::now().toSec();


        ros::Time begin = ros::Time::now();

        isTopInit = true;
//
//        for (int j = 0; j < vehicle_vec.size(); j++)
//        {
//            std::cout<<vehicle_vec[j].x<<"\t"<<vehicle_vec[j].y<<"\t"<<vehicle_vec[j].theta<<std::endl;
//        }

        if(!isLeftInit || !isRightInit || !isVehicleInit)
        {
            return;
        }

//        std::cout<<isLeftInit<<"\t"<<isRightInit<<"\t"<<isVehicleInit<<std::endl;

        pharos_vlp_tilt::VehiclePoseArrayPtr store_vehicle(new pharos_vlp_tilt::VehiclePoseArray);

        store_vehicle = vehicle_vec;

        store_vehicle->header.stamp = input.header.stamp;

        pub_for_sync.publish(store_vehicle);

        ros::Time last = ros::Time::now();

        diff_value = last.toSec() - begin.toSec();

        if(isTimeCenter)
        {
            ROS_INFO("Center time : %lf", diff_value);
        }

    }

    void Left_VLP_CB (const velodyne_msgs::custompoint &input)
    {
        isLeftInit = true;
    }

    void Right_VLP_CB (const velodyne_msgs::custompoint &input)
    {
        isRightInit = true;
    }

    void Vehicle_State_CB (const pharos_msgs::StateStamped2016Ptr &input)
    {
        pharos_vlp_tilt::VehiclePosePtr vehicle(new pharos_vlp_tilt::VehiclePose);

        bicycle_model(vehicle->x, vehicle->y, vehicle->theta ,input->state.velocity, input->state.wheel_angle, 0.01);

        vehicle->stamp.data = input->header.stamp;

        if(vehicle_vec->vehicles.size() == 0){
            for(int i=0;i<vehicle_stack;i++){
                vehicle_vec->vehicles.push_back(*vehicle);
            }
        }

        vehicle_vec->vehicles.push_back(*vehicle);

        if(vehicle_vec->vehicles.size() > 1
        && (input->header.stamp.toSec() < vehicle_vec->vehicles[vehicle_vec->vehicles.size() - 2].stamp.data.toSec()
          ||  fabs(input->header.stamp.toSec() - vehicle_vec->vehicles[vehicle_vec->vehicles.size() - 2].stamp.data.toSec()) > 1))
        {
//            pharos_vlp_tilt::VehiclePoseArrayPtr vehicle_init(new pharos_vlp_tilt::VehiclePoseArray);

            std::cout << "Time dismatched!!!! Vehicle Array clear!!!!" << "\n";
            vehicle_vec->vehicles.clear();
            isLeftInit = false;
            isRightInit = false;
            isVehicleInit = false;
        }


        if(vehicle_vec->vehicles.size() > vehicle_stack)
        {
            isVehicleInit = true;

            vehicle_vec->vehicles.erase(vehicle_vec->vehicles.begin() + 0);
        }

//        std::cout << vehicle_vec[0].x << "\t" << vehicle_vec->back().x << "\n";
        Eigen::Matrix2d rotation2Dmatrix;
        rotation2Dmatrix << cos(vehicle_vec->vehicles.back().theta), -sin(vehicle_vec->vehicles.back().theta),
                sin(vehicle_vec->vehicles.back().theta), cos(vehicle_vec->vehicles.back().theta);

        for(int i=0; i<vehicle_vec->vehicles.size(); i++)
        {
            vehicle_vec->vehicles[i].x -= vehicle_vec->vehicles.back().x;

            Eigen::Vector2d position;
            position.x() = vehicle_vec->vehicles[i].x;
            position.y() = vehicle_vec->vehicles[i].y;

            position = rotation2Dmatrix * position;

            vehicle_vec->vehicles[i].x = position.x();
            vehicle_vec->vehicles[i].y = position.y();

            vehicle_vec->vehicles[i].theta -= vehicle_vec->vehicles.back().theta;
        }

    }

protected:
    ros::Subscriber sub_left;
    ros::Subscriber sub_right;
    ros::Subscriber sub_top;
    ros::Subscriber sub_vehicle;

    ros::Publisher pub_for_sync;
    ros::Publisher pub_top;
    ros::Publisher pub_top_pcl;

};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vlpt_sync_center");       //노드명 초기화

    ROS_INFO("started vlpt_sync_center");

    vlp_ hello;

    ros::spin();
    return 0;

}

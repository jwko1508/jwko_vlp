#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pharos_msgs/VehiclePoseArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

pharos_msgs::VehiclePoseArrayPtr vehicle_array(new pharos_msgs::VehiclePoseArray);
std::vector<pcl::PointCloud<pcl::PointXYZI>> cloud_roadinfo_array;

int decay;

double diff_value;

class del_
{
public:
    del_()
    {
        nh = ros::NodeHandlePtr(new ros::NodeHandle(""));
        pnh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

        pnh->param<std::string>("frame_id", frame_id, "/vehicle_frame");
        pnh->param<float>("min_intensity", min_intensity, 50.0);
        pnh->param<int>("decay", decay, 1);
        printf("road info decay step: %d\n", decay);

        pcl_sub = nh->subscribe("/vlp_cloud/mcl/road", 1, &del_::del_CB, this);
        vehicle_array_sub = nh->subscribe("/vlp_msg/center", 1, &del_::Vehicle_Array, this);

        pcl_roadinfo = nh->advertise<sensor_msgs::PointCloud2>("/vlp_cloud/mcl/roadinfo", 1);
    }

    void Vehicle_Array(const pharos_msgs::VehiclePoseArrayPtr &input)
    {
        vehicle_array->vehicles.clear();
        for(int i=decay-1; i>=0; i--){
            vehicle_array->vehicles.push_back(input->vehicles[input->vehicles.size()-1 - i*10]);
        }
    }

    void Decay(pcl::PointCloud<pcl::PointXYZI>::Ptr &info)
    {
        diff_value = ros::Time::now().toSec();

        if(cloud_roadinfo_array.size() > decay-1) cloud_roadinfo_array.erase(cloud_roadinfo_array.begin());
        cloud_roadinfo_array.push_back(*info);

        if(cloud_roadinfo_array.size()>1){
            for(int i=cloud_roadinfo_array.size()-2; i>=0; i--){
                pcl::PointXYZI pt;
                Eigen::Vector2d position;
                Eigen::Matrix2d rotation2Dmatrix;
                rotation2Dmatrix << cos(vehicle_array->vehicles[i].theta), -sin(vehicle_array->vehicles[i].theta),
                        sin(vehicle_array->vehicles[i].theta), cos(vehicle_array->vehicles[i].theta);

                for(int j=0; j<cloud_roadinfo_array[i].size(); j++){
                    position.x() = cloud_roadinfo_array[i].points[j].x + vehicle_array->vehicles[i].x;
                    position.y() = cloud_roadinfo_array[i].points[j].y + vehicle_array->vehicles[i].y;
                    position = rotation2Dmatrix * position;
                    pt.x = position.x();
                    pt.y = position.y();
                    info->push_back(pt);
                }
            }
        }
        // printf("decay_t : %lf\n",ros::Time::now().toSec()-diff_value);
    }

    void del_CB(const sensor_msgs::PointCloud2Ptr& input)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roadinfo(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(*input, *cloud);

        float sum_intensity = 0;

        for(int i=0;i<cloud->size();i++)
        {
            sum_intensity+=cloud->points[i].intensity;
        }

        for(int i=0;i<cloud->size();i++){
            if(cloud->at(i).intensity > sum_intensity/cloud->size() && cloud->at(i).intensity > min_intensity )
                cloud_roadinfo->push_back(cloud->at(i));
        }


        Decay(cloud_roadinfo);


        sensor_msgs::PointCloud2Ptr pc_roadinfo(new sensor_msgs::PointCloud2);

        pcl::toROSMsg(*cloud_roadinfo, *pc_roadinfo);

        // double now_time = ros::Time::now().toSec();
        // std::cout << "road info dt : " << now_time-input->header.stamp.toSec() << std::endl;

        pc_roadinfo->header = input->header;
//        pc_roadinfo->header.frame_id = frame_id;
        pcl_roadinfo.publish(pc_roadinfo);
    }

protected:
    ros::NodeHandlePtr nh;
    ros::NodeHandlePtr pnh;

    ros::Subscriber pcl_sub;
    ros::Subscriber vehicle_array_sub;

    ros::Publisher pcl_roadinfo;

    std::string frame_id;
    float min_intensity;
};



int main(int argc, char **argv)
{

    ros::init(argc, argv, "pharos_velodyne_roadinfo");

    ROS_INFO("started pharos_velodyne_roadinfo node");
    del_ carframe1;


    ros::spin();

    return 0;
}
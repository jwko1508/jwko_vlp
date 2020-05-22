#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>

#define x_len 80//m
#define y_len 80//m
#define x_cell x_len*10
#define y_cell y_len*10

class delll
{
public:
    delll()
    {
        pnh = ros::NodeHandle("~");

        pcl_sub = nh.subscribe("/vlp_cloud/mcl/vertical", 1, &delll::del_CB, this);
        pcl_verticalinfo = nh.advertise<sensor_msgs::PointCloud2>("/vlp_cloud/mcl/verticalinfo", 1);

        pnh.param<std::string>("frame_id", frame_id_, "/vehicle_frame");
        pnh.param<float>("vertical_offset", offset_, 0.0);

        printf("offset : %f\n",offset_);

    }
    void del_CB(const sensor_msgs::PointCloud2Ptr& input)
    {
//        std::cout << "a" << std::endl;
        int bd[x_cell][y_cell]={0,};


        if(input->data.size() !=0){
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::fromROSMsg(*input, *cloud);
            std::vector<std::pair<int, int> > Index;
            Index.clear();

//            std::cout << "b" << std::endl;
            for(int i=0;i<cloud->size();i++){
                int xIndex=0, yIndex=0;
                xIndex = (int)(floor(cloud->at(i).x*5+0.5)) + x_cell/2;
                yIndex = (int)(floor(cloud->at(i).y*5+0.5)) + y_cell/2;
//                xIndex = 0;
//                yIndex = 0;
                if(xIndex > x_cell-1 || yIndex > y_cell-1 || xIndex < 0 || yIndex < 0)
                    continue;
                else{
//                    std::cout << xIndex << "  " << yIndex << std::endl;
                    if(cloud->at(i).z >0+offset_ && cloud->at(i).z<=0.5+offset_){
                        // bd[xIndex][yIndex] = bd[xIndex][yIndex] | 128;
                        // Index.emplace_back(std::make_pair(xIndex,yIndex));
                    }
                    else if(cloud->at(i).z >.5+offset_ && cloud->at(i).z<=1.0+offset_){
                        bd[xIndex][yIndex] = bd[xIndex][yIndex] | 64;
                        Index.emplace_back(std::make_pair(xIndex,yIndex));
                    }
                    else if(cloud->at(i).z >1.0+offset_ && cloud->at(i).z<=1.5+offset_){
                        bd[xIndex][yIndex] = bd[xIndex][yIndex] | 32;
                        Index.emplace_back(std::make_pair(xIndex,yIndex));
                    }
                    else if(cloud->at(i).z >1.5+offset_ && cloud->at(i).z<=2.0+offset_){
                        bd[xIndex][yIndex] = bd[xIndex][yIndex] | 16;
                        Index.emplace_back(std::make_pair(xIndex,yIndex));
                    }
                    else if(cloud->at(i).z >2.0+offset_ && cloud->at(i).z<=2.5+offset_){
                        bd[xIndex][yIndex] = bd[xIndex][yIndex] | 8;
                        Index.emplace_back(std::make_pair(xIndex,yIndex));
                    }
                    else if(cloud->at(i).z >2.5+offset_ && cloud->at(i).z<=3.0+offset_){
                        bd[xIndex][yIndex] = bd[xIndex][yIndex] | 4;
                        Index.emplace_back(std::make_pair(xIndex,yIndex));
                    }
                    else if(cloud->at(i).z >3.0+offset_ && cloud->at(i).z<=3.5+offset_){
                        bd[xIndex][yIndex] = bd[xIndex][yIndex] | 2;
                        Index.emplace_back(std::make_pair(xIndex,yIndex));
                    }
                    else if(cloud->at(i).z >3.5+offset_ && cloud->at(i).z<=4.0+offset_){
                        bd[xIndex][yIndex] = bd[xIndex][yIndex] | 1;
                        Index.emplace_back(std::make_pair(xIndex,yIndex));
                    }
                }
            }
            if(!Index.empty()){

                std::sort(Index.begin(),Index.end());

                for(int i=0;i<Index.size();i++){
                    pcl::PointXYZI points;
                    if(i==0){
                        points.x = (Index[0].first-x_cell/2)/5.0;
                        points.y = (Index[0].second-y_cell/2)/5.0;
                        points.z = std::bitset<8>(bd[Index[0].first][Index[0].second]&255).count();
                        points.intensity = bd[Index[0].first][Index[0].second];
                        cloud_filtered->push_back(points);
                    }
                    else{
                        if(Index[i-1]!=Index[i]){
                            points.x = (Index[i].first-x_cell/2)/5.0;
                            points.y = (Index[i].second-y_cell/2)/5.0;
                            points.z = std::bitset<8>(bd[Index[i].first][Index[i].second]&255).count();
                            points.intensity = bd[Index[i].first][Index[i].second];
                            cloud_filtered->push_back(points);
                        }
                    }
                }
            }
            // double now_time = ros::Time::now().toSec();
            // std::cout << "vertical info dt : " << now_time-input->header.stamp.toSec() << std::endl;

            sensor_msgs::PointCloud2Ptr pc_verticalinfo(new sensor_msgs::PointCloud2);
            pcl::toROSMsg(*cloud_filtered, *pc_verticalinfo);
            pc_verticalinfo->header = input->header;
//            pc_verticalinfo->header.frame_id = frame_id_;
            pcl_verticalinfo.publish(pc_verticalinfo);
        }

    }

protected:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber pcl_sub;

    ros::Publisher pcl_verticalinfo;

    std::string frame_id_;

    float offset_;
};



int main(int argc, char **argv)
{

    ros::init(argc, argv, "pharos_velodyne_verticalinfo");
    ros::NodeHandle nh;

    ROS_INFO("started pharos_velodyne_verticalinfo node");


    delll carframe2;
    ros::spin();

    return 0;
}
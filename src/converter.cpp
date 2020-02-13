#include "pcl_converter/converter.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>

void Converter::convertCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
    float x = cloud -> points[0].intensity;
    pub_.publish(cloud);
    cout << x << endl;
    
}
void Converter::points_Callback(const sensor_msgs::PointCloud2ConstPtr &msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    /*vg.setInputCloud(cloud);
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_filtered);
    cout << "after : " << cloud_filtered->points.size() << "////// before: " << cloud->points.size() << endl;*/
    pcl::PointXYZI point;
    
    for (std::size_t i = 0; i < cloud->points.size(); ++i){
        if (cloud -> points[i].intensity >= 20 && cloud -> points[i].intensity <= 70){
            point = cloud -> points[i];
            cloud_filtered-> points.push_back(point);
        }
    }
    cloud_filtered->header.frame_id = "velodyne";
    pub_.publish(cloud_filtered);
    /*pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("intensity");
    pass.setFilterLimits(20.0, 100.0);
    pass.filter (*cloud_filtered2);
    pub_.publish(cloud_filtered2);*/    
}
void Converter::initSetup(){
    sub_ = nh_.subscribe("velodyne_points",100,&Converter::points_Callback,this);
    pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("filter_point", 1);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pcl_converter_node");
    Converter converter;
    converter.initSetup();
    ros::spin();
}
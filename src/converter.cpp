#include "pcl_converter/converter.h"
#include <velodyne_pointcloud/point_types.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

void Converter::convertCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
    pcl::PCLPointCloud2 pcl_pc2;
    //pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointXYZI point;
    point.x = point.y = point.z = point.intensity = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::fromROSMsg(*input, *cloud_XYZIR);
    //fromPCLPointCloud2(pcl_pc2, *cloud_XYZIR);
    cloud_filtered->header.frame_id = "velodyne";
    for (size_t i = 0; i < cloud_XYZIR -> points.size(); i++){
        if (cloud_XYZIR -> points[i].ring >= 0 && cloud_XYZIR -> points[i].ring <= 5){
            point.x = cloud_XYZIR -> points[i].x;
            point.y = cloud_XYZIR -> points[i].y;
            point.z = cloud_XYZIR -> points[i].z;
            point.intensity = cloud_XYZIR -> points[i].intensity;
            cloud_filtered -> points.push_back(point);
        }
    }
    float x = cloud_XYZIR -> points[0].ring;
    pub_.publish(cloud_filtered);
    cout << x << endl;
    
}
void Converter::points_Callback(const sensor_msgs::PointCloud2ConstPtr &msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointXYZI point;
    point.x = point.y = point.z = point.intensity = 0;
    pcl::PointXYZI point1; 
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr container (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered4 (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass;
    visualization_msgs::Marker mark_points;
    mark_points.header.frame_id = "velodyne";
    mark_points.header.stamp = ros::Time::now();
    mark_points.ns = "mark_points";
    mark_points.action = visualization_msgs::Marker::ADD;
    mark_points.pose.orientation.w = 1.0;
    mark_points.id = 0;
    mark_points.type = visualization_msgs::Marker::POINTS;
    mark_points.scale.x = 0.2, mark_points.scale.y = 0.2;
    mark_points.color.g = 1.0f, mark_points.color.a = 1.0;
    pcl::fromROSMsg(*msg, *cloud_XYZIR);
    for (size_t i = 0; i < cloud_XYZIR->points.size(); i++){
        if (cloud_XYZIR->points[i].ring >= 0 && cloud_XYZIR->points[i].ring <= 7 ){
            point.x = cloud_XYZIR->points[i].x;
            point.y = cloud_XYZIR->points[i].y;
            point.z = cloud_XYZIR->points[i].z;
            point.intensity = cloud_XYZIR->points[i].intensity;
            cloud_filtered->points.push_back(point);
        }
    }
    /*vg.setInputCloud(cloud_filtered);  // voxel grid//
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_filtered2);*/
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");  
    pass.setFilterLimits(0.0, 5.0);
    pass.filter (*cloud_filtered2);
    pass.setInputCloud(cloud_filtered2);
    pass.setFilterFieldName("x");  
    pass.setFilterLimits(0.7, 100.0);
    pass.filter (*cloud_filtered3);

    ROS_INFO("after : %d --------------> before : %d",cloud_filtered3->points.size(), cloud_filtered->points.size());
    /*for (size_t i = 0; i < cloud_filtered2->points.size(); i++){
        if (cloud_filtered2->points[i].intensity >= 25 && cloud_filtered2->points[i].intensity <= 75){
            point1.x = cloud_filtered2->points[i].x;
            point1.y = cloud_filtered2->points[i].y;
            point1.z = cloud_filtered2->points[i].z;
            point1.intensity = cloud_filtered2->points[i].intensity;
            container->points.push_back(point1);    
        }   
    }
    int index = container->points.size()/2;
    geometry_msgs::Point p;
    p.x = container->points[index].x;
    p.y = container->points[index].y;
    p.z = container->points[index].z;
    mark_points.points.push_back(p);
    marker_pub_.publish(mark_points);
*/
    cloud_filtered->header.frame_id = "velodyne";
    cloud_filtered2->header.frame_id = "velodyne";
    cloud_filtered3->header.frame_id = "velodyne";
    cloud_filtered4->header.frame_id = "velodyne";
    pub_.publish(cloud_filtered3);
        
}
void Converter::initSetup(){
    sub_ = nh_.subscribe("velodyne_points",100,&Converter::points_Callback,this);
    pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZI>>("filter_point", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pcl_converter_node");
    Converter converter;
    converter.initSetup();
    ros::spin();
}

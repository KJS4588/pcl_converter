#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//pcl_ros
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//filter
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/Marker.h>
#include <velodyne_pointcloud/point_types.h>
#include <cmath>
#include <vector>
#include <math.h>
//ros_package
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

using namespace std;

class Converter{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_, marker_pub_1, marker_pub_2, waypoint_pub, ackermann_pub_, pub;
    ros::Subscriber sub_;
    
    double steer_angle_;
    float ld = 1.5;
    
    geometry_msgs::Point Point;
    ackermann_msgs::AckermannDriveStamped ackerData_;

public: 
    void initSetup();
    void convertCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    void points_Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
    void visualize(vector<pcl::PointXYZ> left_point, vector<pcl::PointXYZ> right_point, vector<geometry_msgs::Point> waypoint);
    float calcPlaneDist(geometry_msgs::Point point);
    double calcSteeringAngle(vector<geometry_msgs::Point> waypoint);
    void process();
    //void cubicFormula( double a , double b, double c, double d, double realRoot[], double complRoot [], int& nRealRootCount);
};

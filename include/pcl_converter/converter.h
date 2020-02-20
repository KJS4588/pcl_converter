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
//#include <visualization_msgs/Marker.h>

#include <vector>
using namespace std;
class Converter{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_, marker_pub_;
    ros::Subscriber sub_;
    
   
public: 
    void initSetup();
    void convertCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    void points_Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
};

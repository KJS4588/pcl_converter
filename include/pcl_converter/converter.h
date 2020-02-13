#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

using namespace std;
class Converter{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

public:
    void initSetup();
    void convertCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    void points_Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
};

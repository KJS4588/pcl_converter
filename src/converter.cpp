#include "pcl_converter/converter.h"

void Converter::points_Callback(const sensor_msgs::PointCloud2ConstPtr &msg){
    
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PassThrough<pcl::PointXYZI> pass; 
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PointXYZI point;
    visualization_msgs::Marker mark_points_left, mark_points_right;
 //--------------------------------------------------------------------------------------------------//
    mark_points_left.header.frame_id = mark_points_right.header.frame_id = "velodyne";
    mark_points_left.header.stamp = mark_points_right.header.stamp = ros::Time::now();
    mark_points_left.ns = mark_points_right.ns = "mark_points";
    mark_points_left.action = mark_points_right.action = visualization_msgs::Marker::ADD;
    mark_points_left.type = mark_points_right.type = visualization_msgs::Marker::POINTS;
    mark_points_left.pose.orientation.w = mark_points_right.pose.orientation.w = 1.0;
    mark_points_left.id = mark_points_right.id = 0;
    mark_points_left.color.g = mark_points_right.color.r = 1.0f; mark_points_left.color.a = mark_points_right.color.a = 1.0;
    mark_points_right.scale.x = mark_points_left.scale.x = 0.2;
    mark_points_right.scale.y = mark_points_left.scale.y = 0.2; 
    
 //--------------------------------------------------------------------------------------------------//
    pcl::fromROSMsg(*msg, *cloud_XYZIR);
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vect_cloud;
    //vect_cloud.resize(16);
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vect_cloud_pre;
    //vect_cloud_pre.resize(16);
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vect_laneleft;
    //vect_laneleft.resize(16);
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vect_laneright;
    //vect_laneright.resize(16);
    for (size_t i = 0; i < 16; i++){
        vect_cloud_pre.push_back (pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>()));
        vect_cloud.push_back (pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>()));
        vect_laneleft.push_back (pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>()));
        vect_laneright.push_back (pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>()));

    }

    for (size_t i = 0; i < cloud_XYZIR->points.size(); i++){
        cout << i << endl;
        if (cloud_XYZIR->points[i].ring == 2 && cloud_XYZIR->points[i].intensity >= 25 && cloud_XYZIR->points[i].intensity <= 70){
            point.x = cloud_XYZIR->points[i].x;
            point.y = cloud_XYZIR->points[i].y;
            point.z = cloud_XYZIR->points[i].z;
            point.intensity = cloud_XYZIR->points[i].intensity;
            vect_cloud_pre[2]->points.push_back(point);
            pass.setInputCloud(vect_cloud_pre[2]);
            pass.setFilterFieldName("x");  
            pass.setFilterLimits(0.7, 30.0);
            pass.filter (*vect_cloud[2]);   
        }
        
        if (cloud_XYZIR->points[i].ring == 3 && cloud_XYZIR->points[i].intensity >= 25 && cloud_XYZIR->points[i].intensity <= 70){
            point.x = cloud_XYZIR->points[i].x;
            point.y = cloud_XYZIR->points[i].y;
            point.z = cloud_XYZIR->points[i].z;
            point.intensity = cloud_XYZIR->points[i].intensity;
            vect_cloud_pre[3]->points.push_back(point);
            pass.setInputCloud(vect_cloud_pre[3]);
            pass.setFilterFieldName("x");  
            pass.setFilterLimits(0.7, 30.0);
            pass.filter (*vect_cloud[3]);
        }  
        
        if (cloud_XYZIR->points[i].ring == 4 && cloud_XYZIR->points[i].intensity >= 25 && cloud_XYZIR->points[i].intensity <= 70){
            point.x = cloud_XYZIR->points[i].x;
            point.y = cloud_XYZIR->points[i].y;
            point.z = cloud_XYZIR->points[i].z;
            point.intensity = cloud_XYZIR->points[i].intensity;
            vect_cloud_pre[4]->points.push_back(point);
            pass.setInputCloud(vect_cloud_pre[4]);
            pass.setFilterFieldName("x");  
            pass.setFilterLimits(0.7, 30.0);
            pass.filter (*vect_cloud[4]);
        }

        if (cloud_XYZIR->points[i].ring == 5 && cloud_XYZIR->points[i].intensity >= 25 && cloud_XYZIR->points[i].intensity <= 70){
            point.x = cloud_XYZIR->points[i].x;
            point.y = cloud_XYZIR->points[i].y;
            point.z = cloud_XYZIR->points[i].z;
            point.intensity = cloud_XYZIR->points[i].intensity;
            vect_cloud_pre[5]->points.push_back(point);
            pass.setInputCloud(vect_cloud_pre[5]);
            pass.setFilterFieldName("x");  
            pass.setFilterLimits(0.7, 30.0);
            pass.filter (*vect_cloud[5]);
        }
        
        if (cloud_XYZIR->points[i].ring == 6 && cloud_XYZIR->points[i].intensity >= 25 && cloud_XYZIR->points[i].intensity <= 70){
            point.x = cloud_XYZIR->points[i].x;
            point.y = cloud_XYZIR->points[i].y;
            point.z = cloud_XYZIR->points[i].z;
            point.intensity = cloud_XYZIR->points[i].intensity;
            vect_cloud_pre[6]->points.push_back(point);
            pass.setInputCloud(vect_cloud_pre[6]);
            pass.setFilterFieldName("x");  
            pass.setFilterLimits(0.7, 30.0);
            pass.filter (*vect_cloud[6]);
        }
        
        if (cloud_XYZIR->points[i].ring == 7 && cloud_XYZIR->points[i].intensity >= 25 && cloud_XYZIR->points[i].intensity <= 70){
            point.x = cloud_XYZIR->points[i].x;
            point.y = cloud_XYZIR->points[i].y;
            point.z = cloud_XYZIR->points[i].z;
            point.intensity = cloud_XYZIR->points[i].intensity;
            vect_cloud_pre[7]->points.push_back(point);
            pass.setInputCloud(vect_cloud_pre[7]);
            pass.setFilterFieldName("x");  
            pass.setFilterLimits(0.7, 30.0);
            pass.filter (*vect_cloud[7]);
        }
        if (cloud_XYZIR->points[i].ring == 8 && cloud_XYZIR->points[i].intensity >= 25 && cloud_XYZIR->points[i].intensity <= 70){
        point.x = cloud_XYZIR->points[i].x;
        point.y = cloud_XYZIR->points[i].y;
        point.z = cloud_XYZIR->points[i].z;
        point.intensity = cloud_XYZIR->points[i].intensity;
        vect_cloud_pre[8]->points.push_back(point);
        pass.setInputCloud(vect_cloud_pre[8]);
        pass.setFilterFieldName("x");  
        pass.setFilterLimits(0.7, 30.0);
        pass.filter (*vect_cloud[8]);
        }
        if (cloud_XYZIR->points[i].ring == 9 && cloud_XYZIR->points[i].intensity >= 25 && cloud_XYZIR->points[i].intensity <= 70){
        point.x = cloud_XYZIR->points[i].x;
        point.y = cloud_XYZIR->points[i].y;
        point.z = cloud_XYZIR->points[i].z;
        point.intensity = cloud_XYZIR->points[i].intensity;
        vect_cloud_pre[9]->points.push_back(point);
        pass.setInputCloud(vect_cloud_pre[8]);
        pass.setFilterFieldName("x");  
        pass.setFilterLimits(0.7, 30.0);
        pass.filter (*vect_cloud[9]);
        }
        else   
            continue;
    }
    
    for (size_t i = 0; i < vect_cloud[2]->points.size(); i++){
        if (vect_cloud[2]->points[i].y >= 1 && vect_cloud[2]->points[i].y <= 5){
            pcl::PointXYZ point1(vect_cloud[2]->points[i].x, vect_cloud[2]->points[i].y, vect_cloud[2]->points[i].z);
            vect_laneleft[2]->points.push_back(point1);    
        }
        else if (vect_cloud[2]->points[i].y <= -1 && vect_cloud[2]->points[i].y >= -3){
            pcl::PointXYZ point1(vect_cloud[2]->points[i].x, vect_cloud[2]->points[i].y, vect_cloud[2]->points[i].z);
            vect_laneright[2]->points.push_back(point1);
        }
        else
            continue;
    }

    for (size_t i = 0; i < vect_cloud[3]->points.size(); i++){
        if (vect_cloud[3]->points[i].y >= 1 && vect_cloud[3]->points[i].y <= 5){
            pcl::PointXYZ point1(vect_cloud[3]->points[i].x, vect_cloud[3]->points[i].y, vect_cloud[3]->points[i].z);
            vect_laneleft[3]->points.push_back(point1);    
        }
        else if (vect_cloud[3]->points[i].y <= -1 && vect_cloud[3]->points[i].y >= -3){
            pcl::PointXYZ point1(vect_cloud[3]->points[i].x, vect_cloud[3]->points[i].y, vect_cloud[3]->points[i].z);
            vect_laneright[3]->points.push_back(point1);    
        }
        else
            continue;
    }

    for (size_t i = 0; i < vect_cloud[4]->points.size(); i++){
        if (vect_cloud[4]->points[i].y >= 1 && vect_cloud[4]->points[i].y <= 5){
            pcl::PointXYZ point1(vect_cloud[4]->points[i].x, vect_cloud[4]->points[i].y, vect_cloud[4]->points[i].z);
            vect_laneleft[4]->points.push_back(point1);    
        }
        else if (vect_cloud[4]->points[i].y <= -1 && vect_cloud[4]->points[i].y >= -3){
            pcl::PointXYZ point1(vect_cloud[4]->points[i].x, vect_cloud[4]->points[i].y, vect_cloud[4]->points[i].z);
            vect_laneright[4]->points.push_back(point1);    
        }
        else
            continue;
    }

     for (size_t i = 0; i < vect_cloud[5]->points.size(); i++){
        if (vect_cloud[5]->points[i].y >= 1 && vect_cloud[5]->points[i].y <= 5){
            pcl::PointXYZ point1(vect_cloud[5]->points[i].x, vect_cloud[5]->points[i].y, vect_cloud[5]->points[i].z);
            vect_laneleft[5]->points.push_back(point1);    
        }
        else if (vect_cloud[5]->points[i].y <= -1 && vect_cloud[5]->points[i].y >= -3){
            pcl::PointXYZ point1(vect_cloud[5]->points[i].x, vect_cloud[5]->points[i].y, vect_cloud[5]->points[i].z);
            vect_laneright[5]->points.push_back(point1);    
        }
        else
            continue;
    }
    
    for (size_t i = 0; i < vect_cloud[6]->points.size(); i++){
        if (vect_cloud[6]->points[i].y >= 1 && vect_cloud[6]->points[i].y <= 5){
            pcl::PointXYZ point1(vect_cloud[6]->points[i].x, vect_cloud[6]->points[i].y, vect_cloud[6]->points[i].z);
            vect_laneleft[6]->points.push_back(point1);    
        }
        else if (vect_cloud[6]->points[i].y <= -1 && vect_cloud[6]->points[i].y >= -3){
            pcl::PointXYZ point1(vect_cloud[6]->points[i].x, vect_cloud[6]->points[i].y, vect_cloud[6]->points[i].z);
            vect_laneright[6]->points.push_back(point1);    
        }
        else
            continue;
    }
    for (size_t i = 0; i < vect_cloud[7]->points.size(); i++){
        if (vect_cloud[7]->points[i].y >= 1 && vect_cloud[7]->points[i].y <= 5){
            pcl::PointXYZ point1(vect_cloud[7]->points[i].x, vect_cloud[7]->points[i].y, vect_cloud[7]->points[i].z);
            vect_laneleft[7]->points.push_back(point1);    
        }
        else if (vect_cloud[7]->points[i].y <= -1 && vect_cloud[7]->points[i].y >= -3){
            pcl::PointXYZ point1(vect_cloud[7]->points[i].x, vect_cloud[7]->points[i].y, vect_cloud[7]->points[i].z);
            vect_laneright[7]->points.push_back(point1);    
        }
        else
            continue;
    }
    for (size_t i = 0; i < vect_cloud[8]->points.size(); i++){
        if (vect_cloud[8]->points[i].y >= 1 && vect_cloud[8]->points[i].y <= 5){
            pcl::PointXYZ point1(vect_cloud[8]->points[i].x, vect_cloud[8]->points[i].y, vect_cloud[8]->points[i].z);
            vect_laneleft[8]->points.push_back(point1);    
        }
        else if (vect_cloud[8]->points[i].y <= -1 && vect_cloud[8]->points[i].y >= -3){
            pcl::PointXYZ point1(vect_cloud[8]->points[i].x, vect_cloud[8]->points[i].y, vect_cloud[8]->points[i].z);
            vect_laneright[8]->points.push_back(point1);    
        }
        else
            continue;
    }
    for (size_t i = 0; i < vect_cloud[9]->points.size(); i++){
        if (vect_cloud[9]->points[i].y >= 1 && vect_cloud[9]->points[i].y <= 5){
            pcl::PointXYZ point1(vect_cloud[9]->points[i].x, vect_cloud[9]->points[i].y, vect_cloud[9]->points[i].z);
            vect_laneleft[6]->points.push_back(point1);    
        }
        else if (vect_cloud[9]->points[i].y <= -1 && vect_cloud[9]->points[i].y >= -3){
            pcl::PointXYZ point1(vect_cloud[9]->points[i].x, vect_cloud[9]->points[i].y, vect_cloud[9]->points[i].z);
            vect_laneright[9]->points.push_back(point1);    
        }
        else
            continue;
    }
    vect_cloud.clear();
    vect_cloud_pre.clear();
    ROS_INFO("----------------------------------------------------------------------------");
    geometry_msgs::Point p_l, p_r;
    
    vector<pcl::PointXYZ> left_point(16);
    vector<pcl::PointXYZ> right_point(16); 
    pcl::PointXYZ point1; 
  
    for (size_t i = 0; i < vect_laneleft[2]->points.size(); i++){
        if (vect_laneleft[2]->points.size() != 0){
            left_point[2].x += vect_laneleft[2]->points[i].x;
            left_point[2].y += vect_laneleft[2]->points[i].y;
            left_point[2].z += vect_laneleft[2]->points[i].z;
        }
        else
            continue;
    }

    for (size_t i = 0; i < vect_laneright[2]->points.size(); i++){
        if (vect_laneright[2]->points.size() != 0){
            right_point[2].x += vect_laneright[2]->points[i].x;
            right_point[2].y += vect_laneright[2]->points[i].y;
            right_point[2].z += vect_laneright[2]->points[i].z;
        }
        else 
            continue;
    }

    for (size_t i = 0; i < vect_laneleft[3]->points.size(); i++){
        if (vect_laneleft[3]->points.size() != 0){
            left_point[3].x += vect_laneleft[3]->points[i].x;
            left_point[3].y += vect_laneleft[3]->points[i].y;
            left_point[3].z += vect_laneleft[3]->points[i].z;
        }
        else
            continue;
    }

    for (size_t i = 0; i < vect_laneright[3]->points.size(); i++){
        if (vect_laneright[3]->points.size() != 0){
            right_point[3].x += vect_laneright[3]->points[i].x;
            right_point[3].y += vect_laneright[3]->points[i].y;
            right_point[3].z += vect_laneright[3]->points[i].z;
        }
        else 
            continue;
    }

    for (size_t i = 0; i < vect_laneleft[4]->points.size(); i++){
        if (vect_laneleft[4]->points.size() != 0){
            left_point[4].x += vect_laneleft[4]->points[i].x;
            left_point[4].y += vect_laneleft[4]->points[i].y;
            left_point[4].z += vect_laneleft[4]->points[i].z;
        }
        else
            continue;
    }

    for (size_t i = 0; i < vect_laneright[5]->points.size(); i++){
        if (vect_laneright[5]->points.size() != 0){
            right_point[5].x += vect_laneright[5]->points[i].x;
            right_point[5].y += vect_laneright[5]->points[i].y;
            right_point[5].z += vect_laneright[5]->points[i].z;
        }
        else 
            continue;
    }
    for (size_t i = 0; i < vect_laneleft[6]->points.size(); i++){
        if (vect_laneleft[6]->points.size() != 0){
            left_point[6].x += vect_laneleft[6]->points[i].x;
            left_point[6].y += vect_laneleft[6]->points[i].y;
            left_point[6].z += vect_laneleft[6]->points[i].z;
        }
        else
            continue;
    }

    for (size_t i = 0; i < vect_laneright[6]->points.size(); i++){
        if (vect_laneright[6]->points.size() != 0){
            right_point[6].x += vect_laneright[6]->points[i].x;
            right_point[6].y += vect_laneright[6]->points[i].y;
            right_point[6].z += vect_laneright[6]->points[i].z;
        }
        else 
            continue;
    }
    for (size_t i = 0; i < vect_laneleft[7]->points.size(); i++){
        if (vect_laneleft[7]->points.size() != 0){
            left_point[7].x += vect_laneleft[7]->points[i].x;
            left_point[7].y += vect_laneleft[7]->points[i].y;
            left_point[7].z += vect_laneleft[7]->points[i].z;
        }
        else
            continue;
    }

    for (size_t i = 0; i < vect_laneright[7]->points.size(); i++){
        if (vect_laneright[7]->points.size() != 0){
            right_point[7].x += vect_laneright[7]->points[i].x;
            right_point[7].y += vect_laneright[7]->points[i].y;
            right_point[7].z += vect_laneright[7]->points[i].z;
        }
        else 
            continue;
    }
    for (size_t i = 0; i < vect_laneleft[8]->points.size(); i++){
        if (vect_laneleft[8]->points.size() != 0){
            left_point[8].x += vect_laneleft[8]->points[i].x;
            left_point[8].y += vect_laneleft[8]->points[i].y;
            left_point[8].z += vect_laneleft[8]->points[i].z;
        }
        else
            continue;
    }

    for (size_t i = 0; i < vect_laneright[8]->points.size(); i++){
        if (vect_laneright[8]->points.size() != 0){
            right_point[8].x += vect_laneright[8]->points[i].x;
            right_point[8].y += vect_laneright[8]->points[i].y;
            right_point[8].z += vect_laneright[8]->points[i].z;
        }
        else 
            continue;
    }
    for (size_t i = 0; i < vect_laneright[9]->points.size(); i++){
        if (vect_laneright[9]->points.size() != 0){
            right_point[9].x += vect_laneright[9]->points[i].x;
            right_point[9].y += vect_laneright[9]->points[i].y;
            right_point[9].z += vect_laneright[9]->points[i].z;
        }
        else 
            continue;
    }
    for (size_t i = 0; i < vect_laneleft[9]->points.size(); i++){
        if (vect_laneleft[9]->points.size() != 0){
            left_point[9].x += vect_laneleft[9]->points[i].x;
            left_point[9].y += vect_laneleft[9]->points[i].y;
            left_point[9].z += vect_laneleft[9]->points[i].z;
        }
        else
            continue;
    }

    for (size_t i =0; i < left_point.size(); i++){
        if (vect_laneleft[i]->points.size()!= 0){
            p_l.x = left_point[i].x/vect_laneleft[i]->points.size();
            p_l.y = left_point[i].y/vect_laneleft[i]->points.size();
            p_l.z = left_point[i].z/vect_laneleft[i]->points.size();;
            mark_points_left.points.push_back(p_l);}
        else
            continue;
    }
    for (size_t i =0; i < right_point.size(); i++){
        if (vect_laneright[i]->points.size()!= 0){
            p_r.x = right_point[i].x/vect_laneright[i]->points.size();
            p_r.y = right_point[i].y/vect_laneright[i]->points.size();
            p_r.z = right_point[i].z/vect_laneright[i]->points.size();;
            mark_points_right.points.push_back(p_r);}
        else
            continue;
    }
    marker_pub_1.publish(mark_points_left);
    marker_pub_2.publish(mark_points_right);
    vect_laneleft.clear();
    vect_laneright.clear();
    left_point.clear();
    right_point.clear();
  
  
 
    /*pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");  
    pass.setFilterLimits(-5.0, 5.0);
    pass.filter (*cloud_filtered2);
    pass.setInputCloud(cloud_filtered2);
    pass.setFilterFieldName("x");  
    pass.setFilterLimits(0.7, 30.0);
    pass.filter (*cloud_filtered3);
            
    /*vg.setInputCloud(cloud_filtered);  // voxel grid//
    vg.setLeafSize(0.1f, 0.1f, 0.1f);
    vg.filter(*cloud_filtered2);*/
    //ROS_INFO("container_size: %d,   %d" ,containerLeft->points.size(), containerRight->points.size());
    
    //pub_.publish(cloud_XYZIR);*/
}
void Converter::initSetup(){
    sub_ = nh_.subscribe("velodyne_points",100,&Converter::points_Callback,this);
    
    marker_pub_1 = nh_.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
    marker_pub_2 = nh_.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pcl_converter_node");
    Converter converter;
    converter.initSetup();
    ros::spin();
}

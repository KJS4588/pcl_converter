#include "pcl_converter/converter.h"
#include "pcl_converter/polyfit.h"
#include "polyfit.c"

//int ld = 1.5; 

const unsigned int ORDER = 3;
const double ACCEPTABLE_ERROR = 0.01;
 /* to do: 1.using previous lane polynomial
           2.make waypoint of vehicle and visualize
           3.use voxel filter
  */

void Converter::points_Callback(const sensor_msgs::PointCloud2ConstPtr &msg){
    //pcl::VoxelGrid<pcl::PointXYZI> vg;
    //pcl::PassThrough<velodyne_pointcloud::PointXYZIR> pass; 
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_filterd (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);

 //--------------------------------------------------------------------------------------------------//
    pcl::fromROSMsg(*msg, *cloud_XYZIR);
    vector<pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr> vect_cloud;
    vector<pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr> vect_cloud_pre;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vect_laneleft;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vect_laneright;
    for (size_t i = 0; i < 16; i++){
        vect_cloud_pre.push_back (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>()));
        vect_cloud.push_back (pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>()));
        vect_laneleft.push_back (pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>()));
        vect_laneright.push_back (pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>()));
    }
    /*pass.setInputCloud(cloud_XYZIR);
    pass.setFilterFieldName("x");  
    pass.setFilterLimits(0.7, 30.0);
    pass.filter (*cloud_filterd);
    */
    for (auto point : *cloud_XYZIR){
        if (point.intensity >= 25 && point.intensity <= 70){
            vect_cloud[point.ring]->points.push_back(point);
        }else
            continue;
    }

    for (size_t i = 0; i < vect_cloud.size(); i++){         
        for (auto point : *vect_cloud[i]){
            pcl::PointXYZ point1(point.x, point.y, point.z);
            if (point.y >= 1 && point.y <= 5 && point.x >= 0.5 && point.x <= 30){
                vect_laneleft[point.ring]->points.push_back(point1);    
            }
            else if (point.y >= -3 && point.y <= -1 && point.x >= 0.5 && point.x <= 30) {
                vect_laneright[point.ring]->points.push_back(point1);
            }else
                continue;
        }
    }
    vect_cloud.clear();
    ROS_INFO("----------------------------------------------------------------------------");
    
    vector<pcl::PointXYZ> left_point(16);
    vector<pcl::PointXYZ> right_point(16);  
    
    int left_invalidated = 0;
    
    for (size_t i =0; i < vect_laneleft.size(); i++){
        pcl::PointXYZ point1; 
        
        if (vect_laneleft[i]->points.size() != 0){        
            float sum_x = 0.0;
            float sum_y = 0.0;
    
            for (auto point_lf : *vect_laneleft[i]){
                sum_x += point_lf.x;
                sum_y += point_lf.y;
            }
            
            point1.x = sum_x / vect_laneleft[i]->points.size();
            point1.y = sum_y / vect_laneleft[i]->points.size();
            point1.z = 0;
            
            left_point[i] = point1;
        } else{
            
            left_point[i] = pcl::PointXYZ(-1000,-1000,-1000);
            left_invalidated++;
        }
    }

    double left_coef[ORDER + 1];
    
    if (left_invalidated <= 12) { // layers detected 4 or more
        int result;

        double xData[left_point.size()];
        double yData[left_point.size()];

        for (int i = 0 ; i < left_point.size() ; i++) {
            xData[i] = left_point[i].x;
            yData[i] = left_point[i].y;
        }
        result = polyfit(xData, yData, left_point.size(), ORDER, left_coef);

        /*cout << endl;
        cout << "left lane" << endl;
        cout << left_coef[3] << " : " << left_coef[2] << " : " << left_coef[1] << " : " << left_coef[0] << endl;
        cout << endl;*/

    } else { // less than 4 -> use previous poly
        
       /* cout << "#############" << endl;
        cout << "left lane" << endl;
        cout << left_coef[3] << " : " << left_coef[2] << " : " << left_coef[1] << " : " << left_coef[0] << endl;
        cout << "#############" << endl;*/
    }

    int right_invalidated = 0;

    for (size_t i =0; i < vect_laneright.size(); i++){
        pcl::PointXYZ point1;

        if (vect_laneright[i]->points.size() != 0){        
            float sum_x = 0.0;
            float sum_y = 0.0;
            
            for (auto point_rt : *vect_laneright[i]){
                sum_x += point_rt.x;
                sum_y += point_rt.y;
            }
            
            point1.x = sum_x / vect_laneright[i]->points.size();
            point1.y = sum_y / vect_laneright[i]->points.size();
            point1.z = 0;
            
            right_point[i] = point1;
        } else{
            
            right_point[i] = pcl::PointXYZ(-1000,-1000,-1000);
            right_invalidated++;
        }
    }

    double right_coef[ORDER + 1];    
    
    if (right_invalidated <= 12) { // layers detected 4 or more
        int result;

        double xData[right_point.size()];
        double yData[right_point.size()];

        for (int i = 0 ; i < right_point.size() ; i++) {
            xData[i] = right_point[i].x;
            yData[i] = right_point[i].y;
        }

        result = polyfit(xData, yData, right_point.size(), ORDER, right_coef);

        /*cout << endl;
        cout << "right lane" << endl;
        cout << right_coef[3] << " : " << right_coef[2] << " : " << right_coef[1] << " : " << right_coef[0] << endl;
        cout << endl;*/

    } else { // less than 4 -> use previous poly
        
        /*cout << "############" << endl;
        cout << "right lane" << endl;
        cout << right_coef[3] << " : " << right_coef[2] << " : " << right_coef[1] << " : " << right_coef[0] << endl;
        cout << "############" << endl; */   
    }
    
    //cubicFormula(left_coef[3] , left_coef[2], left_coef[1], left_coef[0], double realRoot[], double complRoot [], int& nRealRootCount);
    
    vector<geometry_msgs::Point> waypoint;
    float ld = 1.5;
    for (int i = 0; i < 10; i++){
        
        float waypoint_y_l = left_coef[3]*ld*ld*ld + left_coef[2]*ld*ld + left_coef[1] * ld + left_coef[0];
        float waypoint_y_r = right_coef[3]*ld*ld*ld + right_coef[2]*ld*ld + right_coef[1] * ld + right_coef[0];
        //cout << waypoint_y_l << "            " << waypoint_y_r << endl;
        cout << ld << endl;
        ld += 0.3;
        geometry_msgs::Point p;
        p.x = ld;
        p.y = (waypoint_y_l + waypoint_y_r)/2;
        p.z = 0;
        waypoint.push_back(p);
    }
    
    vect_laneleft.clear();
    vect_laneright.clear();
    visualize(left_point, right_point, waypoint);
    
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
}

void Converter::visualize(vector<pcl::PointXYZ> left_point, vector<pcl::PointXYZ> right_point, vector<geometry_msgs::Point> waypoint) {
    visualization_msgs::Marker mark_points_left, mark_points_right, waypoint_viz;
    geometry_msgs::Point p_l, p_r;

    mark_points_left.header.frame_id = mark_points_right.header.frame_id = waypoint_viz.header.frame_id = "velodyne";
    mark_points_left.header.stamp = mark_points_right.header.stamp = waypoint_viz.header.stamp = ros::Time::now();
    mark_points_left.ns = mark_points_right.ns = waypoint_viz.ns = "mark_points";
    mark_points_left.action = mark_points_right.action = waypoint_viz.action = visualization_msgs::Marker::ADD;
    mark_points_left.type = mark_points_right.type = visualization_msgs::Marker::POINTS;
    waypoint_viz.type = visualization_msgs::Marker::LINE_STRIP;
    mark_points_left.pose.orientation.w = mark_points_right.pose.orientation.w = waypoint_viz.pose.orientation.w = 1.0;
    mark_points_left.id = mark_points_right.id = waypoint_viz.id = 0;
    mark_points_left.color.g = mark_points_right.color.r = waypoint_viz.color.b = 1.0f; 
    mark_points_left.color.a = mark_points_right.color.a = waypoint_viz.color.a = 1.0;
    mark_points_right.scale.x = mark_points_left.scale.x = waypoint_viz.scale.x = 0.1;
    mark_points_right.scale.y = mark_points_left.scale.y = 0.1; 
    
    for (auto point : left_point){
        if (point.x != -1000 && point.y != -1000) {
            //cout << point << endl;
            p_l.x = point.x;
            p_l.y = point.y;
            p_l.z = point.z;
            mark_points_left.points.push_back(p_l);
        }
        else
            continue;
    }
    //cout << "###############################" << endl;
    for (auto point : right_point){
        if (point.x != -1000 && point.y != -1000) {
            p_r.x = point.x;
            p_r.y = point.y;
            p_r.z = point.z;
            mark_points_right.points.push_back(p_r);
        }
        else
            continue;
    }

    for (auto point : waypoint){
        waypoint_viz.points.push_back(point);
    }

    left_point.clear();
    right_point.clear();
    waypoint.clear();
    
    marker_pub_1.publish(mark_points_left);
    marker_pub_2.publish(mark_points_right);
    waypoint_pub.publish(waypoint_viz);
}
/*void Converter::cubicFormula( double a , double b, double c, 
                              double d, double realRoot[], 
                              double complRoot [], int& nRealRootCount) 
{ 
    if (a == 0) return;
    if (d == 0) return; 
    b /= a; 
    c /= a; 
    d /= a; 
    double disc , q, r, dum1, s, t , term1, r13;
    q = (3.0*c - (b*b))/9.0; 
    r = -(27.0*d ) + b*(9.0* c - 2.0*(b *b)); 
    r /= 54.0; disc = q *q*q + r*r; complRoot[0] = 0; 
    //The first root is always real. 
    term1 = (b /3.0); 
    if (disc > 0) { // one root real, two are complex 
        s = r + Sqrt( disc); 
        s = ((s < 0) ? -Pow(- s, (1.0/3.0)) : Pow (s, (1.0/3.0))); 
        t = r - Sqrt( disc); t = ((t < 0) ? -Pow(- t, (1.0/3.0)) : Pow (t, (1.0/3.0))); 
        realRoot[0] = -term1 + s + t; term1 += (s + t)/2.0; 
        realRoot[2] = realRoot [1] = -term1; 
        term1 = BrSqrt (3.0)*(-t + s)/2; 
        complRoot[1] = term1 ; 
        complRoot[2] = -term1 ; 
        nRealRootCount = 1; 
        return; 
    } else if (disc == 0) { 
        // All roots real, at least two are equal. 
        complRoot[2] = complRoot [1] = 0; 
        r13 = ((r < 0) ? -pow(- r,(1.0/3.0)) : pow (r,(1.0/3.0))); 
        realRoot[0] = -term1 + 2.0*r13; 
        realRoot[2] = realRoot [1] = -(r13 + term1); 
        nRealRootCount = 2; 
        return; 
    } else { 
        // Only option left is that all roots are real and unequal (to get here, q < 0) 
        q = -q ; 
        dum1 = q *q*q; 
        dum1 = acos (r/sqrt( dum1)); 
        r13 = 2.0*sqrt (q); 
        realRoot[0] = -term1 + r13*cos( dum1/3.0); 
        realRoot[1] = -term1 + r13*cos(( dum1 + 2.0*PI )/3.0); 
        realRoot[2] = -term1 + r13*cos(( dum1 + 4.0*PI )/3.0); 
        nRealRootCount = 3; 
    } 
    return; 
}*/

void Converter::initSetup(){
    sub_ = nh_.subscribe("velodyne_points",100,&Converter::points_Callback,this);
    marker_pub_1 = nh_.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
    marker_pub_2 = nh_.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
    waypoint_pub = nh_.advertise<visualization_msgs::Marker>("waypoint", 10);
}

int main(int argc, char **argv){
    ros::init(argc,argv,"pcl_converter_node");
    Converter converter;
    converter.initSetup();
    ros::spin();
}

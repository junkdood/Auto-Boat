//#pragma once
#include <pcl/ModelCoefficients.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
//点云下采样
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <iostream>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/segmentation/sac_segmentation.h>
class PclTestCore
{
 
  private:
    int index;
    ros::Subscriber sub_point_cloud_;
    ros::Subscriber sub_the_gps;

    ros::Publisher pub_filtered_points_;
    ros::Publisher pub_filtered_vis_;
    ros::Publisher pub_print_point;

    ros::Publisher pub_object;

    float theta;
    std::vector<float> position;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> scene;

    pcl::PointXYZ Coortrans(pcl::PointXYZ inpoint, float theta, pcl::PointXYZ position);
    pcl::PointXYZ Lidar2MKD(pcl::PointXYZ inpoint, float LItheta, pcl::PointXYZ LIposition, float IMtheta, pcl::PointXYZ IMposition);

    /*发布处理后的点云*/
    //ros::Publisher pub_after_points_;

    void point_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud);

    pcl::PointXYZ translate(float theta, std::vector<float> position, pcl::PointXYZ point);
    void get_the_gps(const sensor_msgs::NavSatFix& msg);
    std::vector<std::vector<float>> point_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const    pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    std::vector<std::vector<float>> point_cluster_new(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const    pcl::PointCloud<pcl::PointXYZ>::Ptr out);
    pcl::PointXYZ ret_the_gps(pcl::PointXYZ point);
    void Cloud_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name);
    
    void Rect_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, std::vector<float> cube, std::string name);
    void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered);
  public:
    PclTestCore(ros::NodeHandle &nh);   //构造函数
    ~PclTestCore();                     //析构函数
    void Spin(); 

};


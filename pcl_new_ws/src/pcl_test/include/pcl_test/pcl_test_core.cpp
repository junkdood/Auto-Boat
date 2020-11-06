#include "pcl_test_core.h"

PclTestCore::PclTestCore(ros::NodeHandle &nh){
	
	//使用类的方法作为回调函数，
    sub_point_cloud_ = nh.subscribe("/velodyne_points",10, &PclTestCore::point_cb, this);
 
    pub_filtered_vis_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_vis", 10);
	pub_filtered_points_ = nh.advertise<std_msgs::Float64MultiArray>("/filtered_points", 10);
    ros::spin();
}
//析构函数
PclTestCore::~PclTestCore(){}
 
void PclTestCore::Spin(){
    
}
 
void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clip_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*in_cloud_ptr,*current_pc_ptr);
	//按点云索引提取点云子集
    pcl::ExtractIndices<pcl::PointXYZ> cliper;
    cliper.setInputCloud(current_pc_ptr);
    pcl::PointIndices indices;
    
#pragma omp for
	for (size_t i = 0; i < current_pc_ptr->points.size(); i++){
			
			//过滤得到特定区域内的点云
			if(current_pc_ptr->points[i].z>0.5||current_pc_ptr->points[i].z<-0.5
				||current_pc_ptr->points[i].x>20||current_pc_ptr->points[i].x<-20
				||current_pc_ptr->points[i].y>20||current_pc_ptr->points[i].y<-20){
					
						indices.indices.push_back(i);
		   		}
		  }
	cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	cliper.setNegative(true);            //ture to remove the indices
	cliper.filter(*clip_pc_ptr);
	
	//将点云的z值全部置为0
	for (size_t i = 0; i < clip_pc_ptr->points.size(); i++){
			clip_pc_ptr->points[i].z = 0;
	}
	
	//该函数实现对障碍物的聚类和包围盒检测
	point_cluster(clip_pc_ptr,cluster_pc_ptr);
	
	std::cout << "OBB points: " << cluster_pc_ptr->points.size () << " data points." << std::endl;
	
	for(size_t i = 0;i<cluster_pc_ptr->points.size();i++){
			std::cout<<"point " << i <<":" << "x: " << cluster_pc_ptr->points[i].x  <<"y: " << cluster_pc_ptr->points[i].y << std::endl;	
		}
	//发布
    sensor_msgs::PointCloud2 pub_pc;
	std_msgs::Float64MultiArray pub_cloud;
    pcl::toROSMsg(*cluster_pc_ptr, pub_pc);
	 for (size_t i = 0; i < cluster_pc_ptr->points.size(); i++) 
	 {
	 	pub_cloud.data.push_back(cluster_pc_ptr->points[i].x);
		pub_cloud.data.push_back(cluster_pc_ptr->points[i].y);
	 }
    pub_pc.header = in_cloud_ptr->header;
    pub_filtered_vis_.publish(pub_pc);
	pub_filtered_points_.publish(pub_cloud);
}

void PclTestCore::point_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const pcl::PointCloud<pcl::PointXYZ>::Ptr out){
	
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;
	tree->setInputCloud (in); 								//创建点云索引向量，用于存储实际的点云信息
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  
	ec.setClusterTolerance (0.15); 							//设置近邻搜索的搜索半径为20cm
	ec.setMinClusterSize (100);								//设置一个聚类需要的最少点数目为100
	ec.setMaxClusterSize (25000);							//设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod (tree);								//设置点云的搜索机制
	ec.setInputCloud (in);
	ec.extract (cluster_indices);							//从点云中提取聚类，并将点云索引保存在cluster_indices中
	
	/*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
		//迭代访问点云索引cluster_indices，直到分割出所有聚类
	int j = 0;
	std::cout << "new obstale" << std::endl;
	for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin (); i != cluster_indices.end (); ++i){
					
					pcl::ConvexHull<pcl::PointXYZ> hull;
					//创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
					for (std::vector<int>::const_iterator pit = i->indices.begin (); pit != i->indices.end (); ++pit){
						cloud_cluster->points.push_back (in->points[*pit]);
						//out->points.push_back(in->points[*pit]);
					}
					hull.setInputCloud(cloud_cluster);                   
					hull.setDimension(2);
					hull.setComputeAreaVolume(true);
					std::vector<pcl::Vertices> polygons;
					pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
					hull.reconstruct(*surface_hull, polygons);
					
					//for (size_t i = 0; i < surface_hull->points.size(); i++){
							//out->points.push_back(surface_hull->points[i]);
					//}
					//std::cout << surface_hull->size() << std::endl;
					std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
					
					pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
					feature_extractor.setInputCloud (surface_hull);
					feature_extractor.compute ();
					
					std::vector <float> moment_of_inertia;  //惯性距
					std::vector <float> eccentricity;  //离心率
					
					pcl::PointXYZ min_point_OBB;
					pcl::PointXYZ max_point_OBB;
					pcl::PointXYZ position_OBB;
					
					Eigen::Matrix3f rotational_matrix_OBB;     //包围盒绕轴旋转的角度
					float major_value, middle_value, minor_value; //eigen是计算矩阵的开源库
					Eigen::Vector3f major_vector, middle_vector, minor_vector;
					Eigen::Vector3f mass_center;   //包围盒的中心点
					
					feature_extractor.getMomentOfInertia (moment_of_inertia); //特征提取获取惯性距
					feature_extractor.getEccentricity (eccentricity);  //特征提取获取离心率
					
					feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB); //特征提取OBB ，position是OBB中心相对AABB中心移动的位移
					feature_extractor.getEigenValues (major_value, middle_value, minor_value); //获取特征值
					feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector); //特征向量
					feature_extractor.getMassCenter (mass_center); //获取最大质心
					
					Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z); //向量位置

					Eigen::Quaternionf quat (rotational_matrix_OBB);  //四元用法
					
					Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
					//Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
					Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
					//Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
					Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
					//Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
					Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
					//Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
					
					  p1 = rotational_matrix_OBB * p1 + position;
					  //p2 = rotational_matrix_OBB * p2 + position;
					  p3 = rotational_matrix_OBB * p3 + position;
					  //p4 = rotational_matrix_OBB * p4 + position;
					  p5 = rotational_matrix_OBB * p5 + position;
					  //p6 = rotational_matrix_OBB * p6 + position;
					  p7 = rotational_matrix_OBB * p7 + position;
					  //p8 = rotational_matrix_OBB * p8 + position;
					
				  pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
				  //pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
				  pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
				  //pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
				  pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
				  //pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
				  pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
				  //pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));
				  
				  out->points.push_back(pt1);
				  //out->points.push_back(pt2);
				  out->points.push_back(pt3);
				  //out->points.push_back(pt4);
				  out->points.push_back(pt5);
				  //out->points.push_back(pt6);
				  out->points.push_back(pt7);
				  //out->points.push_back(pt8);		
			}
}

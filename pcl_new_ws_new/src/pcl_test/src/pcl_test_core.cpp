#include "pcl_test_core.h"
#include <cmath>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

PclTestCore::PclTestCore(ros::NodeHandle &nh){
	
	//使用类的方法作为回调函数，
    sub_point_cloud_ = nh.subscribe("/pandar",100, &PclTestCore::point_cb, this);
	sub_the_gps = nh.subscribe("unionstrong/gpfpd",1000,&PclTestCore::get_the_gps,this);
    pub_filtered_vis_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_vis", 10);
	pub_filtered_points_ = nh.advertise<std_msgs::Float64MultiArray>("/filtered_points", 10);
	pub_print_point = nh.advertise<sensor_msgs::PointCloud2>("/print_point", 10);

	pub_object = nh.advertise<std_msgs::Float64MultiArray>("/object", 10);

	theta = 0;
	position.push_back(0);
	position.push_back(0);
	position.push_back(0);

	//pub_after_points_ = nh.advertise<std_msgs::Float64MultiArray>("/after_points", 10);

	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("pcl_Viewer"));
	index = 0;
    ros::spin();
}
//析构函数
PclTestCore::~PclTestCore(){}
 
void PclTestCore::Spin(){
    
}
void PclTestCore::get_the_gps(const sensor_msgs::NavSatFix& msg){
	position[0] = msg.longitude*20037508.34/180;
	position[1] = log(tan((90+msg.latitude)*M_PI/360))/(M_PI/180);
	position[1] = position[1]*20037508.34/180;
	position[2] = msg.altitude;
	theta = msg.position_covariance[1]/180*M_PI;
}

pcl::PointXYZ PclTestCore::ret_the_gps(pcl::PointXYZ point){
	
	float x=point.x/ 20037508.34 * 180;
    float y=point.y/ 20037508.34 * 180;
    y=180 / M_PI * (2 *atan(exp(y * M_PI / 180)) - M_PI / 2);

	pcl::PointXYZ result(x,y,0);
	return result;
}

pcl::PointXYZ PclTestCore::Coortrans(pcl::PointXYZ inpoint, float theta, pcl::PointXYZ position){
	/*单次坐标转换函数*/
	pcl::PointXYZ outpoint;
	float x = cos(theta) * inpoint.x - sin(theta) * inpoint.y + position.x;
	float y = sin(theta) * inpoint.x + cos(theta) * inpoint.y + position.y;
	outpoint.x = x;
	outpoint.y = y;
	outpoint.z = inpoint.z;
	return outpoint;
}

pcl::PointXYZ PclTestCore::Lidar2MKD(pcl::PointXYZ inpoint, float LItheta, pcl::PointXYZ LIposition, float IMtheta, pcl::PointXYZ IMposition){
	/*Lidar到MKD转换函数*/
	pcl::PointXYZ outpoint;
	outpoint = Coortrans(inpoint, LItheta, LIposition);
	outpoint = Coortrans(outpoint, IMtheta, IMposition);
	return outpoint;
}

pcl::PointXYZ PclTestCore::translate(float theta, std::vector<float> position, pcl::PointXYZ point){

	float costheta = cos(theta);
	float sintheta = sin(theta);

	float nx = costheta * point.x - sintheta * point.y + position[0];
	float ny = sintheta * point.x + costheta * point.y + position[1];

	pcl::PointXYZ npoint(nx, ny, point.z);
	return npoint;
}
 
void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
	pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);	
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);			//接收点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);					
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);				//聚类输入
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);				//聚类输出
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);			//显示场景
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);			//显示地面
	std::string scene_name = "scene";

	pcl::fromROSMsg(*in_cloud_ptr,*raw_pc_ptr);

	//点云滤波
	pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> sor;
  	sor.setInputCloud (raw_pc_ptr);
  	sor.setLeafSize (0.01f, 0.01f, 0.01f);
  	sor.filter (*current_pc_ptr);
	
	//显示场景
	for( int i = 0; i < current_pc_ptr->points.size(); i++){
		if(current_pc_ptr->points[i].x > -30 && current_pc_ptr->points[i].x < 30
		&& current_pc_ptr->points[i].y < 30 && current_pc_ptr->points[i].y > -30)
		{
			scene_pc_ptr->points.push_back(current_pc_ptr->points[i]);
		}
	}
	/*
	Cloud_vis(viewer, scene_pc_ptr, scene_name.append(std::to_string(index)));
	if(index == 0 ){
		index = 1;
		scene_name = "scene";
		viewer->removeShape(scene_name.append(std::to_string(index)),0);
	}
	else{
		index = 0;
		scene_name = "scene";
		viewer->removeShape(scene_name.append(std::to_string(index)),0);
	}*/

	//滤除海面
	detectObjectsOnCloud(scene_pc_ptr, ground_pc_ptr);
	
	/*Cloud_vis(viewer, ground_pc_ptr, scene_name.append(std::to_string(index)));
	if(index == 0 ){
		index = 1;
		scene_name = "scene";
		viewer->removeShape(scene_name.append(std::to_string(index)),0);
	}
	else{
		index = 0;
		scene_name = "scene";
		viewer->removeShape(scene_name.append(std::to_string(index)),0);
	}*/

	/*
	//一定范围内的点
	for( int i = 0; i < current_pc_ptr->points.size(); i++){
		if(current_pc_ptr->points[i].x > -20 && current_pc_ptr->points[i].x < 20 
		&& current_pc_ptr->points[i].y < 20 && current_pc_ptr->points[i].y > -20 
		&& current_pc_ptr->points[i].z > -0.5){
			in_pc_ptr->points.push_back(current_pc_ptr->points[i]);
		}

	}

	//按点云索引提取点云子集
    pcl::ExtractIndices<pcl::PointXYZ> cliper;
    cliper.setInputCloud(in_pc_ptr);
    pcl::PointIndices indices;

//#pragma omp for
	for (size_t i = 0; i < in_pc_ptr->points.size(); i++){
		//过滤得到特定区域内的点云
		if(in_pc_ptr->points[i].x>-20 && in_pc_ptr->points[i].x<20
		&& in_pc_ptr->points[i].y>-20 && in_pc_ptr->points[i].y<20){
			indices.indices.push_back(i);
	   	}
	}

	cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	cliper.setNegative(true);            //ture to remove the indices
	cliper.filter(*tmp_pc_ptr);
	*/

	//将点云的z值全部置为0
	for (size_t i = 0; i < ground_pc_ptr->points.size(); i++){
		ground_pc_ptr->points[i].z = 0;
	}
	
	//该函数实现对障碍物的聚类和包围盒检测
	std::vector<std::vector<float>>  cluster_result = point_cluster_new(ground_pc_ptr,out_pc_ptr);
	
	std::cout << "OBB points: " << out_pc_ptr->points.size () << " data points." << std::endl;
	std::string objectname = "object";
	if(out_pc_ptr->points.size() != 0){
		for(size_t i = 0;i<out_pc_ptr->points.size();i++){
			std::cout<<"point " << i <<":" << "x: " << out_pc_ptr->points[i].x  <<"y: " << out_pc_ptr->points[i].y << std::endl;	
			Rect_vis(viewer, cluster_result[i], objectname.append(std::to_string(i)));
			objectname = "object";

			std_msgs::Float64MultiArray object;
			object.data.push_back(cluster_result[i][0]);
			object.data.push_back(cluster_result[i][1]);
			object.data.push_back(cluster_result[i][2]);
			object.data.push_back(cluster_result[i][3]);
			object.data.push_back(cluster_result[i][4]);
			pub_object.publish(object);
		}
		for(size_t i = 0;i<out_pc_ptr->points.size();i++){
			viewer->removeShape(objectname.append(std::to_string(i)),0);
			objectname = "object";
		}
	}
	
	//发布
    sensor_msgs::PointCloud2 pub_pc;
	std_msgs::Float64MultiArray pub_cloud;
    pcl::toROSMsg(*out_pc_ptr, pub_pc);
	 for (size_t i = 0; i < out_pc_ptr->points.size(); i++) 
	 {
	 	pub_cloud.data.push_back(out_pc_ptr->points[i].x);
		pub_cloud.data.push_back(out_pc_ptr->points[i].y);
	 }
    pub_pc.header = in_cloud_ptr->header;
    pub_filtered_vis_.publish(pub_pc);
	pub_filtered_points_.publish(pub_cloud);
	//pub_after_points_.publish();
}

std::vector<std::vector<float>> PclTestCore::point_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const pcl::PointCloud<pcl::PointXYZ>::Ptr out){
	
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;
	tree->setInputCloud (in); 								//创建点云索引向量，用于存储实际的点云信息
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  
	ec.setClusterTolerance (0.15); 							//设置近邻搜索的搜索半径为20cm
	ec.setMinClusterSize (200);								//设置一个聚类需要的最少点数目为100
	ec.setMaxClusterSize (25000);							//设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod (tree);								//设置点云的搜索机制
	ec.setInputCloud (in);
	ec.extract (cluster_indices);							//从点云中提取聚类，并将点云索引保存在cluster_indices中
	
	/*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
		//迭代访问点云索引cluster_indices，直到分割出所有聚类
	int j = 0;
	std::vector<std::vector<float>> result;
	for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin (); i != cluster_indices.end (); ++i){
					
		std::vector<float> oneobject;
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
		Eigen::Vector3f middle ((max_point_OBB.x + min_point_OBB.x)/2, (max_point_OBB.y + min_point_OBB.y)/2, (max_point_OBB.z + min_point_OBB.z)/2);
					
		p1 = rotational_matrix_OBB * p1 + position;
		p3 = rotational_matrix_OBB * p3 + position;
		p5 = rotational_matrix_OBB * p5 + position;
		p7 = rotational_matrix_OBB * p7 + position;
		middle = rotational_matrix_OBB * middle + position;

		/*pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
		pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
		pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
		pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));*/
				
		pcl::PointXYZ mpoint (middle (0), middle (1), middle (2));
		float length = max_point_OBB.x - min_point_OBB.x;
		float width = max_point_OBB.y - min_point_OBB.y;
		float height = max_point_OBB.z - min_point_OBB.z;

		/*out->points.push_back(pt1);
		out->points.push_back(pt3);
		out->points.push_back(pt5);
		out->points.push_back(pt7);*/
				  	
		out->points.push_back(mpoint);
		oneobject.push_back(mpoint.x); oneobject.push_back(mpoint.y); oneobject.push_back(mpoint.z);
		oneobject.push_back(length); oneobject.push_back(width); oneobject.push_back(height);
		result.push_back(oneobject);
	}
	return result;
}

std::vector<std::vector<float>> PclTestCore::point_cluster_new(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const pcl::PointCloud<pcl::PointXYZ>::Ptr out){
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;
	tree->setInputCloud (in); 								//创建点云索引向量，用于存储实际的点云信息
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.15); 							//设置近邻搜索的搜索半径为20cm
	ec.setMinClusterSize (200);								//设置一个聚类需要的最少点数目为100
	ec.setMaxClusterSize (5000);							//设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod (tree);								//设置点云的搜索机制
	ec.setInputCloud (in);
	ec.extract (cluster_indices);							//从点云中提取聚类，并将点云索引保存在cluster_indices中

	/*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
		//迭代访问点云索引cluster_indices，直到分割出所有聚类
	int j = 0;
	std::vector<std::vector<float>> result;
	for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin (); i != cluster_indices.end (); ++i){

		std::vector<float> oneobject;
		//创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = i->indices.begin (); pit != i->indices.end (); ++pit){
		    cloud_cluster->points.push_back (in->points[*pit]);
		}
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud (cloud_cluster);
		feature_extractor.compute ();

		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;
		Eigen::Vector3f mass_center;   //包围盒的中心点

		feature_extractor.getAABB (min_point_AABB, max_point_AABB); //特征提取AABB
		feature_extractor.getMassCenter (mass_center); //获取最大质心

		pcl::PointXYZ mpoint (mass_center(0), mass_center(1), mass_center(2));
		float length = max_point_AABB.x - min_point_AABB.x;
		float width = max_point_AABB.y - min_point_AABB.y;
		float height = max_point_AABB.z - min_point_AABB.z;

		out->points.push_back(mpoint);
		oneobject.push_back(mpoint.x); oneobject.push_back(mpoint.y); oneobject.push_back(mpoint.z);
		oneobject.push_back(length); oneobject.push_back(width); oneobject.push_back(height);
		result.push_back(oneobject);
	}
	return result;
}

void PclTestCore::Cloud_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name){

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // green

    viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, name);

    viewer->spinOnce(100);
    //boost::this_thread::sleep(boost::posix_time::microseconds(10000));
}

void PclTestCore::Rect_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, std::vector<float> cube, std::string name){

    Eigen::Vector3f center(cube[0], cube[1], cube[2]-0.5);
    Eigen::Quaternionf rotation(1,0,0,0);

    float length = cube[3];
    float width = cube[4];
    float height = cube[5];

	float side = length>width ? length:width;
    viewer->addCube(center,rotation,side,side,1.5,name);

    viewer->spinOnce (100);
    //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
}

void PclTestCore::detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered){

	if (cloud->size() > 0){
    	//创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 创建分割对象
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // 可选择配置，设置模型系数需要优化
        seg.setOptimizeCoefficients(true);
        // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
        seg.setModelType(pcl::SACMODEL_PLANE);//设置模型类型
        seg.setMethodType(pcl::SAC_RANSAC);//设置随机采样一致性方法类型
        // you can modify the parameter below
  		seg.setMaxIterations(2000);//表示点到估计模型的距离最大值，10000 -> 2000
        seg.setDistanceThreshold(0.2);//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件 0.35 -> 0.2
        seg.setInputCloud(cloud);
        //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            cout<<"error! Could not found any inliers!"<<endl;
        }
        // extract ground
        // 从点云中抽取分割的处在平面上的点集
        pcl::ExtractIndices<pcl::PointXYZ> extractor;//点提取对象
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*cloud_filtered);
        // vise-versa, remove the ground not just extract the ground
        // just setNegative to be true
        cout << "filter done."<<endl;
    }
    else{
        cout<<"no data!"<<endl;
    }
}


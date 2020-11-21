#include "pcl_test_core.h"
#include <cmath>

PclTestCore::PclTestCore(ros::NodeHandle &nh){
	
	//使用类的方法作为回调函数，
    sub_point_cloud_ = nh.subscribe("/pandar",1, &PclTestCore::point_cb, this);
	sub_the_gps = nh.subscribe("unionstrong/gpfpd",1000,&PclTestCore::get_the_gps,this);
    pub_filtered_vis_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_vis", 1);
	pub_filtered_points_ = nh.advertise<std_msgs::Float64MultiArray>("/filtered_points", 1);
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);			//接收点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);					
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);				//聚类输入
	pcl::PointCloud<pcl::PointXYZ>::Ptr out_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);				//聚类输出
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);			//显示场景
	pcl::PointCloud<pcl::PointXYZ>::Ptr ground_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);			//显示地面
	std::string scene_name = "scene";

	pcl::fromROSMsg(*in_cloud_ptr,*current_pc_ptr);

    //显示场景
    for( int i = 0; i < current_pc_ptr->points.size(); i++){
        if(current_pc_ptr->points[i].x > -30 && current_pc_ptr->points[i].x < 30
           && current_pc_ptr->points[i].y < 30 && current_pc_ptr->points[i].y > -30)
        {
            scene_pc_ptr->points.push_back(current_pc_ptr->points[i]);
        }
    }

    //down sampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(scene_pc_ptr);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    sor.filter(*cloud_filtered);

	Cloud_vis(viewer, cloud_filtered, scene_name.append(std::to_string(index)));

	detectObjectsOnCloud(cloud_filtered, ground_pc_ptr);
	//该函数实现对障碍物的聚类和包围盒检测
	std::vector<std::vector<float>>  cluster_result = point_cluster(ground_pc_ptr,out_pc_ptr);
	
	std::cout << "AABB points: " << out_pc_ptr->points.size () << " data points." << std::endl;
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

    if (!viewer->wasStopped())
    {
        viewer->spinOnce(50);
    }
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
}

std::vector<std::vector<float>> PclTestCore::point_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr in,const pcl::PointCloud<pcl::PointXYZ>::Ptr out){

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster_indices;
	tree->setInputCloud (in); 								//创建点云索引向量，用于存储实际的点云信息
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;  
	ec.setClusterTolerance (1); 							//设置近邻搜索的搜索半径为20cm
	ec.setMinClusterSize (20);								//设置一个聚类需要的最少点数目为100
	ec.setMaxClusterSize (1000);							//设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod (tree);								//设置点云的搜索机制
	ec.setInputCloud (in);
	ec.extract (cluster_indices);							//从点云中提取聚类，并将点云索引保存在cluster_indices中
	
	/*为了从点云索引向量中分割出每个聚类，必须迭代访问点云索引，每次创建一个新的点云数据集，并且将所有当前聚类的点写入到点云数据集中。*/
	int j = 0;
	std::vector<std::vector<float>> result;
	cout << "num: " << cluster_indices.size() << endl;
	for (std::vector<pcl::PointIndices>::const_iterator i = cluster_indices.begin (); i != cluster_indices.end (); ++i){
		std::vector<float> oneobject;
		//创建新的点云数据集cloud_cluster，将所有当前聚类写入到点云数据集中
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = i->indices.begin (); pit != i->indices.end (); ++pit){
		    cloud_cluster->points.push_back (in->points[*pit]);
		}

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		double x_min=999, x_max=-999, y_min=999, y_max=-999, z_min=999, z_max=-999;
		double mass_x=0, mass_y=0, mass_z=0;

        for (int k = 0; k < cloud_cluster->size(); ++k) {
            if (x_min > cloud_cluster->points[k].x)
                x_min = cloud_cluster->points[k].x;
            if (x_max < cloud_cluster->points[k].x)
                x_max = cloud_cluster->points[k].x;
            if (y_min > cloud_cluster->points[k].y)
                y_min = cloud_cluster->points[k].y;
            if (y_max < cloud_cluster->points[k].y)
                y_max = cloud_cluster->points[k].y;
            if (z_min > cloud_cluster->points[k].z)
                z_min = cloud_cluster->points[k].z;
            if (z_max < cloud_cluster->points[k].z)
                z_max = cloud_cluster->points[k].z;
            mass_x += cloud_cluster->points[k].x;
            mass_y += cloud_cluster->points[k].y;
            mass_z += cloud_cluster->points[k].z;
        }
        mass_x /= cloud_cluster->size();
        mass_y /= cloud_cluster->size();
        mass_z /= cloud_cluster->size();

        /*
        pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud (cloud_cluster);
        feature_extractor.compute ();

        pcl::PointXYZ min_point_AABB;
        pcl::PointXYZ max_point_AABB;
        Eigen::Vector3f mass_center;   //包围盒的中心点

        feature_extractor.getAABB (min_point_AABB, max_point_AABB); //特征提取AABB
        feature_extractor.getMassCenter (mass_center); //获取最大质心
        */

        pcl::PointXYZ mpoint (mass_x, mass_y, mass_z);
        float length = x_max - x_min;
        float width = y_max - y_min;
        float height = z_max - z_min;

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
}

void PclTestCore::Rect_vis(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, std::vector<float> cube, std::string name){

    Eigen::Vector3f center(cube[0], cube[1], cube[2]-0.5);
    Eigen::Quaternionf rotation(1,0,0,0);

    float length = cube[3];
    float width = cube[4];
    float height = cube[5];

	float side = length>width ? length:width;
    viewer->addCube(center,rotation,side,side,1.5,name);

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
  		seg.setMaxIterations(1000);//表示点到估计模型的距离最大值，
        seg.setDistanceThreshold(0.7);//设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
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

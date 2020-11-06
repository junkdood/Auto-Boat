#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>
#include <iomanip>
#include <math.h>

using namespace std;

class send_gps
{
public:
    send_gps(int a)
    {
        pub = nh.advertise <gps_common::GPSFix>("send_gps",1000);
        sub_gps = nh.subscribe("unionstrong/gpfpd",1000,&send_gps::onmsg_gps,this);
        // pub2 = nh.advertise <gps_common::GPSFix>("print_the_obs",1000);         
		// sub_cloud = nh.subscribe("to_vis",1000,&send_gps::onmsg_cloud,this);
        printf("success connected");
    }
    void onmsg_cloud(const std_msgs::Float64MultiArray &msg)  //接收激光雷达信息并计算vo域
	{
		int size= msg.data.end() - msg.data.begin();
		printf("ob_points=　%d\n",size);
		for (size_t i = 0; i < size; i+=4)  //4个点为一个障碍物
		{
            double xt[5],yt[5];
			xt[0]=msg.data[i];
			yt[0]= msg.data[i+1];
            xt[1]=msg.data[i+3];
			yt[1]= msg.data[i+4];
            xt[2]=xt[0];
            yt[2]=yt[1];
            xt[3]=xt[1];
            yt[3]=yt[0];
            xt[4]=(xt[0]+xt[1])/2;
            yt[4]=(yt[0]+yt[1])/2;
            gps_common::GPSFix msg1;
            for(int j = 0 ;j<5;j++){
                x1= x + xt[j]*cos(heading)+yt[j]*sin(heading);
                y1= y + yt[j]*cos(heading)-xt[j]*sin(heading);
                msg1.longitude=x1/ 20037508.34 * 180;
                msg1.latitude=y1/ 20037508.34 * 180;
                msg1.latitude=180 / M_PI * (2 *atan(exp(msg1.latitude * M_PI / 180)) - M_PI / 2);
                msg1.pitch=0;
                msg1.dip=0;
                msg1.roll=0;
                msg1.track=0;
                msg1.header.frame_id="gps";
                msg1.err_speed=1;
                pub2.publish(msg1);
            }
            
            printf("%lf %lf\n",yt[0],xt[0]);
            
		}
	}
    void onmsg_gps(const sensor_msgs::NavSatFix& msg)
    {
		x = msg.longitude*20037508.34/180;
		y = log(tan((90+msg.latitude)*M_PI/360))/(M_PI/180);
		y = y*20037508.34/180;
		heading=msg.position_covariance[1]/180*M_PI;
		if(heading>M_PI)
			heading=heading-2*M_PI;
        gps_common::GPSFix msg1;
        msg1.latitude=msg.latitude;
        msg1.longitude=msg.longitude;
        msg1.pitch=msg.position_covariance[3];
        msg1.dip=msg.position_covariance[1];
        msg1.roll=msg.position_covariance[2];
        msg1.track=msg.position_covariance[1];
        msg1.header.frame_id='gps';
        msg1.err_speed=1;
        pub.publish(msg1);
        printf("%lf\n",msg1.track);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pub2;
	ros::Subscriber sub_cloud;
    ros::Subscriber sub_gps;
    double x = 113.612*20037508.34/180;;
    double y=(log(tan((90+22.374)*M_PI/360))/(M_PI/180))*20037508.34/180;
    double heading=0;
    double x1;
    double y1;
};
int main(int argc,char ** argv){
    ros::init(argc,argv,"send_gps");
    int a=1;
    send_gps send(a);
    ros::spin();
    return 0;
}

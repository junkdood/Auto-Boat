#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;

vector<vector<double> >a;

inline void file_to_string(vector<string> &record, const string& line, char delimiter)
{
    int linepos=0;
    char c;
    int linemax=line.length();
    string curstring;
    record.clear();
    while(linepos<linemax)
    {
        c = line[linepos];
        if(isdigit(c)||c=='.'){
            curstring+=c;
        }
        else if(c==delimiter&&curstring.size()){
            record.push_back(curstring);
            curstring="";
        }
        ++linepos;
    }
    if(curstring.size())
        record.push_back(curstring);
    return;
}
inline double string_to_float(string str){
    int i=0,len=str.length();
    double sum=0;
    while(i<len){
        if(str[i]=='.') break;
        sum=sum*10+str[i]-'0';
        ++i;
    }
    ++i;
    double t=1,d=1;
    while(i<len){
        d*=0.1;
        t=str[i]-'0';
        sum+=t*d;
        ++i;
    }
    return sum;
}


void read()
{

    vector<string> row;
    string line;
    string filename;
	vector<double>b;
	int mod=20;       //取点间隔,从原始gps采集数据取点
	int flag_read=0;  //
    ifstream in("/home/deepdriving/boat_ws/src/track/track.csv");  //读取循迹目标点
    if (in.fail())  { cout << "File not found" <<endl; return ; } 
	string temp;
	getline(in,temp);
    while(getline(in, line)  && in.good() )
    {


			file_to_string(row, line, ',');  //把line里的单元格数字字符提取出来，“,”为单元格分隔符
        	for(int i=0; i<=1; i++){
        	double x;
        	double y=string_to_float(row[i]);
        	if(i==1)
        	{
        		x=y; 
			}
			else if(i==0)
			{
				                
				x=y; 
			}
            b.push_back(x);
       	 	}
			
			
        	a.push_back(b);
        	b.clear();

        flag_read++;
    }
    in.close();
    return ;
}


int main(int argc,char **argv){
    ros::init(argc,argv,"print_the_track");
    // read();
    ros::NodeHandle nh;
    ros::Publisher pub;
    pub = nh.advertise <gps_common::GPSFix>("print_the_track",1000);
    int all=0;
    vector<double>b;
    b.push_back(22.01968414);
    b.push_back(113.70008576);
    a.push_back(b);
    b.clear();
    b.push_back(22.01880615);
    b.push_back(113.69964835);
    a.push_back(b);
    b.clear();
    b.push_back(22.01950419);
    b.push_back(113.69939911);
    a.push_back(b);
    b.clear();
    b.push_back(22.01932766);
    b.push_back(113.69884611);
    a.push_back(b);
    b.clear();
    b.push_back(22.01978416);
    b.push_back(113.69888698);
    a.push_back(b);
    b.clear();
    ros::Rate R(1000);
    while (ros::ok() && all<1000){
        gps_common::GPSFix msg1;
        msg1.latitude=a[all%a.size()][0];
        msg1.longitude=a[all%a.size()][1];
        msg1.pitch=0;
        msg1.dip=0;
        msg1.roll=0;
        msg1.track=0;
        msg1.header.frame_id="gps";
        msg1.err_speed=1;
        pub.publish(msg1);
        printf("%lf %lf %d\n",a[all%3][1],a[all%a.size()][0],all);
        R.sleep();
        all++;
    }
    // ros::Rate R(1000);
    // while (ros::ok() && all<a.size()){
    //     gps_common::GPSFix msg1;
    //     double x=a[all][0];
    //     double y=a[all][1];
    //     x=x/ 20037508.34 * 180;
    //     y=y/ 20037508.34 * 180;
    //     y=180 / M_PI * (2 *atan(exp(y * M_PI / 180)) - M_PI / 2);
    //     msg1.latitude=y;
    //     msg1.longitude=x;
    //     msg1.pitch=0;
    //     msg1.dip=0;
    //     msg1.roll=0;
    //     msg1.track=0;
    //     msg1.header.frame_id="gps";
    //     msg1.err_speed=1;
    //     pub.publish(msg1);
    //     printf("%lf %lf\n",y,x);
    //     R.sleep();
    //     all++;
    // }
}



#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <algorithm>
#include <limits>

#define PI 3.14159

class LaserFilter
{
public:
    LaserFilter(ros::NodeHandle n)
     :node_handle_(n)
    {
        laser_in = node_handle_.subscribe("scan",1,&LaserFilter::laser_in_cb,this);
        laser_out = node_handle_.advertise<sensor_msgs::LaserScan>("filter_scan",50);
    }

    void laser_in_cb(const sensor_msgs::LaserScanConstPtr& laser_msg)
    {
        sensor_msgs::LaserScan scan_after_filter;
        int range_size = laser_msg->ranges.size();          //680
        int center_index = range_size / 2;                  //340
        int filter_angle = 15;                              //过滤激光中心线范围内角度15度的扫描
        double filter_angle_radian = 15.0 / 180 * PI;         //转化为弧度表示
        float inf = std::numeric_limits<float>::infinity();     //无穷大浮点数

        int filter_right_index = center_index - filter_angle_radian / laser_msg->angle_increment / 2; 
        int filter_left_index = center_index + filter_angle_radian / laser_msg->angle_increment / 2; 
        std::cout << "filter_angle_radian : " << filter_angle_radian << std::endl;
        std::cout << "angle_increment : " << laser_msg->angle_increment << std::endl;
        std::cout << "filter_right_index : " << filter_right_index << std::endl;
        std::cout << "filter_left_index : " << filter_left_index << std::endl;
        auto max_range_ptr = std::max_element(laser_msg->ranges.begin(),laser_msg->ranges.end());
        std::cout << "max range : " << *max_range_ptr << std::endl;


        scan_after_filter.header.frame_id = laser_msg->header.frame_id;
        scan_after_filter.angle_min = laser_msg->angle_min;
        scan_after_filter.angle_max = laser_msg->angle_max;
        scan_after_filter.time_increment = laser_msg->time_increment;
        scan_after_filter.angle_increment = laser_msg->angle_increment;
        scan_after_filter.range_min = laser_msg->range_min;
        scan_after_filter.range_max = laser_msg->range_max;
        scan_after_filter.scan_time = laser_msg->scan_time;
        scan_after_filter.header.stamp = ros::Time::now();
        scan_after_filter.ranges = laser_msg->ranges;
        for(int i=filter_right_index; i<=filter_left_index; i++){
            scan_after_filter.ranges[i] = inf;
        }

        laser_out.publish(scan_after_filter);
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber laser_in;
    ros::Publisher laser_out;

};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"laser_filter_leg");
    ros::NodeHandle nh;
    LaserFilter laserfilter(nh);
    ros::spin();
    return 0;
}

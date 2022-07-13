#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Occupancy {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber pcl_sub_;
        ros::Publisher map_pub_;
        ros::Publisher marker_pub_;

        float map_res_;
        float map_x_size_;
        float map_y_size_;
        float height_scale_;
        bool publish_markers_;
        std::string map_frame_;
        std::string pcl_frame_;

        tf::TransformListener tf_listener_;

        nav_msgs::OccupancyGrid occupancy_map_;
        visualization_msgs::MarkerArray marker_array_;

        cv::Mat_<int8_t> map_;
        
        void initializeOccupancyMap();
        void resetOccupancyMap();
        void publishMarker();
        PointCloud projectPCL(const PointCloud::ConstPtr&);

        void pointCloudCallback(const PointCloud::ConstPtr&);
    public:
        Occupancy();
};

Occupancy::Occupancy() : nh_("~") {
    std::string default_map_frame("odom");
    nh_.param("map_frame",map_frame_,default_map_frame);
    nh_.param("map_resolution",map_res_,0.25f);
    nh_.param("map_x_size",map_x_size_,20.f);
    nh_.param("map_y_size",map_y_size_,20.f);
    nh_.param("height_scale",height_scale_,0.04f);
    initializeOccupancyMap();
    pcl_sub_ = nh_.subscribe("pc_in", 1, &Occupancy::pointCloudCallback, this);
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 1, true);
}

PointCloud Occupancy::projectPCL(const PointCloud::ConstPtr& pcl){
    PointCloud pcl_new;
    pcl_ros::transformPointCloud(map_frame_, *pcl, pcl_new, tf_listener_);
    return pcl_new;
}

void Occupancy::initializeOccupancyMap() {
    map_ = cv::Mat::ones((int) map_x_size_/map_res_,(int) map_y_size_/map_res_, CV_8SC1);
    map_.setTo(-1);
    occupancy_map_.header.frame_id = map_frame_;
    occupancy_map_.info.resolution = map_res_;
    occupancy_map_.info.width = (int) map_x_size_ / map_res_;
    occupancy_map_.info.height = (int) map_y_size_ / map_res_;
    occupancy_map_.info.origin.position.x = - map_x_size_ / 2;
    occupancy_map_.info.origin.position.y = 0;
    occupancy_map_.info.origin.position.z = 0;
    occupancy_map_.info.origin.orientation.x = -0.707;
    occupancy_map_.info.origin.orientation.y = -0.707;
    occupancy_map_.info.origin.orientation.z = 0.;
    occupancy_map_.info.origin.orientation.w = 0.;
}

void Occupancy::resetOccupancyMap() {
    map_.setTo(-1);
}

void Occupancy::publishMarker() {
}

void Occupancy::pointCloudCallback(const PointCloud::ConstPtr& msg) {
    int x_index, y_index;
    int scaled_z;
    Occupancy::resetOccupancyMap();
    PointCloud pc;
    pc = Occupancy::projectPCL(msg);
    for (unsigned int i=0;i<pc.size();i++){
        x_index = (int) ((pc.points[i].x / map_res_) + (map_x_size_ / 2  / map_res_));
        y_index = (int) ((pc.points[i].y / map_res_));
        if ((x_index > 0) && (x_index < map_x_size_/map_res_)) {
            if ((y_index > 0) && (y_index < map_y_size_/map_res_)) {
                scaled_z = (int) (pc.points[i].z / height_scale_ + 50);
                if (map_(x_index, y_index) - 5.0 < scaled_z) {
                    if (scaled_z > 100) {
                        scaled_z = 100;
                    } else if (scaled_z < 0) {
                        scaled_z = 0;
                    }
                    map_(x_index, y_index) = scaled_z;
                }
            }
        }
    }
    std::vector<signed char> array;
    array.assign(map_.data, map_.data + map_.total()*map_.channels());
    occupancy_map_.data = array;
    occupancy_map_.header.stamp = ros::Time::now();
    map_pub_.publish(occupancy_map_);
}

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"occupancy_segmentation");
    Occupancy OCC;
    ros::spin();
}

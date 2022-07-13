#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class ImuRollPitch {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber imu_sub_;

        tf::TransformBroadcaster br_;
        std::string fixed_frame_;

        sensor_msgs::Imu imu_;
       
        void imuCallback(const sensor_msgs::ImuPtr&);

    public:
        ImuRollPitch();
};

ImuRollPitch::ImuRollPitch() : nh_("~") {
    std::string default_frame("odom");
    nh_.param("fixed_frame", fixed_frame_, default_frame);
    imu_sub_ = nh_.subscribe("imu_in", 1, &ImuRollPitch::imuCallback, this);
}

void ImuRollPitch::imuCallback(const sensor_msgs::ImuPtr& msg) {
    double q0, q1, q2, q3;
    double roll, pitch, yaw;
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    q.setRPY(roll, pitch, 0.0);
    
    
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = msg->header.frame_id;
    transform.child_frame_id = fixed_frame_;
    transform.transform.rotation.w = q.getW();
    transform.transform.rotation.x = -q.getX();
    transform.transform.rotation.y = -q.getY();
    transform.transform.rotation.z = -q.getZ();
    br_.sendTransform(transform);

}

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"imu_tf");
    ImuRollPitch IRP;
    ros::spin();
}

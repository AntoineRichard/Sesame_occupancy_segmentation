#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuFuse {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber accel_sub_;
        ros::Subscriber gyro_sub_;
        ros::Publisher imu_pub_;

        sensor_msgs::Imu imu_;
        int rate_;

        void accelCallback(const sensor_msgs::ImuPtr&);
        void gyroCallback(const sensor_msgs::ImuPtr&);

    public:
        ImuFuse();
        void run();
};

ImuFuse::ImuFuse() : nh_("~") {
    nh_.param("relay_rate", rate_, 30);
    accel_sub_ = nh_.subscribe("imu_accel", 1, &ImuFuse::accelCallback, this);
    gyro_sub_ = nh_.subscribe("imu_gyro", 1, &ImuFuse::gyroCallback, this);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu_out",1,true);
}

void ImuFuse::accelCallback(const sensor_msgs::ImuPtr& msg) {
    imu_.header = msg->header;
    imu_.linear_acceleration = msg->linear_acceleration;
}

void ImuFuse::gyroCallback(const sensor_msgs::ImuPtr& msg) {
    imu_.angular_velocity = msg->angular_velocity;
}

void ImuFuse::run() {
    ros::Rate rate(rate_);
    while (ros::ok()){
        imu_pub_.publish(imu_);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"imu_fusion");
    ImuFuse IF;
    IF.run();
}

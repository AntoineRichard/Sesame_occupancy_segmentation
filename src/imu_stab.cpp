#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

using Eigen::Vector3f;
using Eigen::Matrix3f;

class RollPitch {
    private:
        ros::NodeHandle nh_;
	    ros::Subscriber accel_sub_;

        std::string orientation_;

	    Vector3f ref_;
        tf::TransformBroadcaster br_;
	    std::string mode_;

	    void GetReferenceVector(std::string mode);
	    void TFPublisher(tf::Quaternion q);
	    Matrix3f RotationMatrixFromAccel(Vector3f);
	    tf::Quaternion RotationMatrixToQuaternions(Matrix3f);
	    void accelCallback(const sensor_msgs::ImuPtr&);

    public:
	    RollPitch();
};

RollPitch::RollPitch() : nh_("~") {
    std::string default_mode("Ydown");
    nh_.param("mode", mode_, default_mode);

    RollPitch::GetReferenceVector(mode_);
    accel_sub_ = nh_.subscribe("/camera/accel/sample", 1, &RollPitch::accelCallback, this);

}

void RollPitch::GetReferenceVector(std::string mode) {
    if (mode == "Zup") {
	//ROS_INFO("Using mode Zup");
        ref_ << 0,0,1;
    } else if (mode == "Zdown") {
	//ROS_INFO("Using mode Zdown");
        ref_ << 0.0,0.0,-1.0;
    } else if (mode == "Yup") {
	//ROS_INFO("Using mode Yup");
        ref_ << 0.0,1.0,0.0;
    } else if (mode == "Ydown") {
	//ROS_INFO("Using mode Ydown");
        ref_ << 0.0,-1.0,0.0;
    } else if (mode == "Xup") {
	//ROS_INFO("Using mode Xup");
        ref_ << 1.0,0.0,0.0;
    } else if (mode == "Xdown") {
	//ROS_INFO("Using mode Xdown");
        ref_ << -1.0,0.0,0.0;
    }
}

Matrix3f RollPitch::RotationMatrixFromAccel(Vector3f acc) {
    float c,s;
    Vector3f v, accn;
    Matrix3f I3, vx, R;

    I3 = Matrix3f::Identity();
    accn = acc/acc.norm();
    //ROS_INFO("accn, x: %f, y:%f, z:%f",accn(0), accn(1), accn(2));
    
    v = ref_.cross(acc);
    //ROS_INFO("v, size: %d, x:%f, y:%f, z:%f",v.size(), v(0), v(1), v(2));
    s = v.norm();
    c = ref_.dot(acc);
    //ROS_INFO("cos: %f, sin:%f", c, s);
    vx << 0, -v(2), v(1),
	  v(2),0,-v(0),
	  -v(1),v(0),0;
    R = I3 + vx + (vx*vx * (1 - c)/(s*s));
    return R;
}

tf::Quaternion RollPitch::RotationMatrixToQuaternions(Matrix3f R) {
    float x,y,z,w;

    w = std::sqrt(1.0 + R(0,0) + R(1,1) + R(2,2)) / 2.0;
    
    float w4 = (4.0 * w);

    x = (R(2,1) - R(1,2)) / w4 ;
    y = (R(0,2) - R(2,0)) / w4 ;
    z = (R(1,0) - R(0,1)) / w4 ;

    //ROS_INFO("quat, x: %f, y:%f, z:%f, w:%f",x,y,z,w);
    tf::Quaternion q(x,y,z,w);
    return q;
}

void RollPitch::TFPublisher(tf::Quaternion q){
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_accel_optical_frame", "camera_accel_gravity_frame"));
}

void RollPitch::accelCallback(const sensor_msgs::ImuPtr& msg) {
    Matrix3f R;
    tf::Quaternion q;

    Vector3f acc(msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z);
    //ROS_INFO("acc, x: %f, y:%f, z:%f",acc(0), acc(1), acc(2));
    //ROS_INFO("ref_, x: %f, y:%f, z:%f",ref_(0), ref_(1), ref_(2));

    R = RollPitch::RotationMatrixFromAccel(acc);
    q = RollPitch::RotationMatrixToQuaternions(R);
    RollPitch::TFPublisher(q);
}

/*RollPitch::getA(r, p, dr, dp, dy) {
    float r_dr, r_dp, p_dr, p_dp;
    r_dr = (dp*std::cos(r) - dy*sin(r))*std::tan(p)
    r_dp = (1/(std::cos(p)*std::cos(p))) * (dp*std::sin(r) + dy*std::cos(r))
    p_dr = - dp*std::sin(r) - dy*std::cos(r)
    p_dp = 0;
    A << r_dr, r_dp,
         p_dr, p_dp;
}

RollPitch::getC(r, p) {
    float c00, c01, c10, c11, c20, c21;
    c00 = 0;
    c01 = g*std::cos(p);
    c10 = -g*std::cos(r)*std::cos(p);
    c11 = g*std::sin(r)*std::sin(p);
    c20 = -g*std::sin(r)*std::cos(p);
    c21 = g*std::cos(p)*std::cos(r);
    C << c00, c01,
         c10, c11,
	 c20, c21;
}

RollPitch::getH(r, p) {
   float h0,h1,h2;
   h0 = g*std::sin(p);
   h1 = -g*std::cos(p)*std::sin(r);
   h2 = -g*std::cos(p)*std::cos(r);
   H << h0,
        h1,
	h2;
}

RollPitch::InitialiazeEKF() {
    Vector2f x(0);
    Matrix2f A;
    Matrix2f H;
}

RollPitch::getF(r, p, dr, dp, dy) {
    float f0, f1;
    f0 = dr + dp*std::cos(dr) + dy*std::cos(r)*std::tan(p);
    f1 = dp*std::cos(r) - dy*std::sin(r);
    F << f0,
         f1;
}

RollPitch::Predict(dr, dp, dy, Ts) {
    RollPitch::getF(X(0),X(1),dr,dp,dy);
    X = X + Ts*F;
    P = P + Ts*(A*P + P*A^T + Q);
}

RollPitch::Update() {
    K = P*C^T*(CPC^T + R).inv();
    X = X + K*(y - H);
    P = (I - K*C)*P;
}*/


int main(int argc, char * argv[]) {
    ros::init(argc,argv,"imu_gravity_aligne_frame");
    RollPitch RP;
    ros::spin();
}


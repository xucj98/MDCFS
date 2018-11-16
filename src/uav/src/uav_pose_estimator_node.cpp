#include <ros/ros.h>
#include <uav/UWB.h>
#include <uav/uav_states.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

// anchor position
const Eigen::Vector3f ap0(   0,    0,  2.4);
const Eigen::Vector3f ap1( 5.1,  0.4,  2.3);
const Eigen::Vector3f ap2( 5.3,  4.3,  2.5);
const Eigen::Vector3f ap3( -0.6,  4,  1.4);

#define sqr(x) ((x)*(x))

static bool initialized = false;
static float last_time;
static Eigen::VectorXf x(25); // uav states
static Eigen::Vector3f p; // uav position in world frame.
static Eigen::Quaternion<float> q; // uav coordinate rotation in world frame.
static Eigen::Quaternion<float> q_init; 
static Eigen::Vector3f v; // uav linear velocity in world frame.
static Eigen::Vector3f a; // uav linear acceleration in world frame.
static Eigen::Vector3f w; // uav coordinate angular velocity in world frame.
static Eigen::Vector3f g; // gravity acceleration in world frame.
static Eigen::Vector3f bw; // the bias of angular velocity
static Eigen::MatrixXf P(3, 3); // covariance matrix of estimate
static Eigen::MatrixXf Q(3, 3); // covariance matrix of inference
// static Eigen::MatrixXf R(10, 10); // covariance matrix of observation

static ros::Publisher states_pub;
static ros::Subscriber imu_sub;
static ros::Subscriber uwb_sub;

static std::vector<sensor_msgs::Imu> imu_datas;
static std::vector<uav::UWB> uwb_datas;

Eigen::Matrix3f crossMatrix(Eigen::Vector3f w)
{
    Eigen::Matrix3f res;
    res <<    0, -w(2),  w(1),
           w(2),     0, -w(0),
          -w(1),  w(0),     0;
    return res;
}

void pose_estimator(
    float curr_time, 
    Eigen::Vector4f uwb_d,
    Eigen::MatrixXf R)
{
    static int uwb_init_cnt = 0;

    // initialized system
    if (!initialized)
    {
        ROS_INFO("initialized %d", uwb_init_cnt);
        last_time = curr_time;

        // estimate initial value of position
        Eigen::Matrix3f A;
        Eigen::Vector3f B;

        if (uwb_init_cnt == 0)
        {
            p << 0, 0, 0;
           
            Q.setZero(3, 3);
            Q = 0.01 * Eigen::MatrixXf::Identity(3, 3);        
            P.setZero(3, 3);
        }

        A << (ap0(0) - ap1(0)) * 2, (ap0(1) - ap1(1)) * 2, (ap0(2) - ap1(2)) * 2,
             (ap0(0) - ap2(0)) * 2, (ap0(1) - ap2(1)) * 2, (ap0(2) - ap2(2)) * 2,
             (ap0(0) - ap3(0)) * 2, (ap0(1) - ap3(1)) * 2, (ap0(2) - ap3(2)) * 2;
        B << sqr(uwb_d(1)) - sqr(uwb_d(0)) + sqr(ap0(0)) - sqr(ap1(0)) + sqr(ap0(1)) - sqr(ap1(1)) + sqr(ap0(2)) - sqr(ap1(2)),
             sqr(uwb_d(2)) - sqr(uwb_d(0)) + sqr(ap0(0)) - sqr(ap2(0)) + sqr(ap0(1)) - sqr(ap2(1)) + sqr(ap0(2)) - sqr(ap2(2)),
             sqr(uwb_d(3)) - sqr(uwb_d(0)) + sqr(ap0(0)) - sqr(ap3(0)) + sqr(ap0(1)) - sqr(ap3(1)) + sqr(ap0(2)) - sqr(ap3(2));

        p += A.inverse() * B;
        uwb_init_cnt += 1;
        
        if (uwb_init_cnt >= 5)
        {
            initialized = true;
            p /= uwb_init_cnt;
            p << 2, 4, 1;
        }
        return;
    }

    
    float dt = curr_time - last_time;

    // get observation zo
    Eigen::VectorXf zo(4);
    zo.segment(0, 4) = uwb_d.array().square();
    
    // std::cout << "==================== observation ========================";
    // std::cout << std::fixed << std::setprecision(2) << zo << std::endl << std::endl;
    
    // estimate xe = f(x)
    Eigen::Vector3f xe_p;   
    
    xe_p = p;
    
    // calculate F = Jacobi(f, x)
    Eigen::MatrixXf F(3, 3);
    F = Eigen::Matrix3f::Identity();
    
    // estimate ze = h(xe)
    Eigen::Vector4f ze_d;
    Eigen::VectorXf ze(4);

    ze_d(0) = (xe_p - ap0).squaredNorm();
    ze_d(1) = (xe_p - ap1).squaredNorm();
    ze_d(2) = (xe_p - ap2).squaredNorm();
    ze_d(3) = (xe_p - ap3).squaredNorm();
   
    ze.segment(0, 4) = ze_d;
   
    // std::cout << "==================== estimate observation ========================" << std::endl;
    // std::cout << std::fixed << std::setprecision(2) << ze << std::endl << std::endl; 

    // calculate H = Jacobi(h, xe)
    Eigen::MatrixXf H(4, 3);
    H.setZero(4, 3);

    // Jacobi(ze_d, xe_p)
    H.block(0, 0, 1, 3) = Eigen::MatrixXf(2 * (xe_p - ap0)).transpose();
    H.block(1, 0, 1, 3) = Eigen::MatrixXf(2 * (xe_p - ap1)).transpose();
    H.block(2, 0, 1, 3) = Eigen::MatrixXf(2 * (xe_p - ap2)).transpose();
    H.block(3, 0, 1, 3) = Eigen::MatrixXf(2 * (xe_p - ap3)).transpose(); 
    // std::cout << "==================== Matrix H ========================" << std::endl;
    // std::cout << std::fixed << std::setprecision(2) << H << std::endl << std::endl;

    Eigen::MatrixXf Pe = F * P * F.transpose() + Q * dt;
    Eigen::VectorXf y = zo - ze;
    Eigen::MatrixXf S = H * Pe * H.transpose() + R;
    Eigen::MatrixXf K = Pe * H.transpose() * S.inverse();
    

    p       = xe_p       +  K.block( 0, 0, 3, 4) * y;
   
    // g << 0, 0, 9.8;

    // std::cout << "==================== Matrix K ========================" << std::endl;
    // std::cout << std::fixed << std::setprecision(2) << K << std::endl << std::endl;

    std::cout << "==================== residue ========================" << std::endl;
    std::cout << std::fixed << std::setprecision(2) << y << std::endl << std::endl;
 
    P = (Eigen::MatrixXf::Identity(3, 3) - K * H) * Pe;

    // std::cout << "==================== Matrix New P ========================" << std::endl;
    // std::cout << std::fixed << std::setprecision(2) << P << std::endl << std::endl;

    ROS_INFO("position: %f %f %f", p.x(), p.y(), p.z());

    uav::uav_states states_msg;
    states_msg.position.x = p.x();
    states_msg.position.y = p.y();
    states_msg.position.z = p.z();

    states_msg.orientation.x = q.x();
    states_msg.orientation.y = q.y();
    states_msg.orientation.z = q.z();
    states_msg.orientation.w = q.w();
    
    states_pub.publish(states_msg);

    last_time = curr_time;

}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu data;

    data = *msg;
    
    imu_datas.push_back(data);
    // ROS_INFO("imu time stamp: %lf", data.header.stamp.toSec());
    // ROS_INFO("imu quaternion: %lf %lf %lf %lf", data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z);
    // ROS_INFO("linear acceleration: %f %f %f", data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
    // ROS_INFO("angular velocity: %f %f %f", data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z);
    
    q.x() = data.orientation.x;
    q.y() = data.orientation.y;
    q.z() = data.orientation.z;
    q.w() = data.orientation.w;

    if (imu_datas.size() == 1) q_init = q;
    q = q_init.inverse() * q;

    Eigen::Vector4f uwb_distance;
    Eigen::MatrixXf R(4, 4);

    if (uwb_datas.size() > 0)
    {
        uwb_distance << uwb_datas[uwb_datas.size() - 1].d0, uwb_datas[uwb_datas.size() - 1].d1, uwb_datas[uwb_datas.size() - 1].d2, uwb_datas[uwb_datas.size() - 1].d3;
 
        R.block(0, 0, 4, 4) = 0.2 * Eigen::MatrixXf::Identity(4, 4);
    
        pose_estimator(data.header.stamp.toSec(), uwb_distance, R);
    }
}

void uwbCallback(const uav::UWB::ConstPtr& msg)
{
    uav::UWB data;
    data = *msg;
    uwb_datas.push_back(data);
    // ROS_INFO("uwb time stamp: %lf", data.header.stamp.toSec());
    // ROS_INFO("uwb distance: %f %f %f %f", data.d0, data.d1, data.d2, data.d3);
     
    Eigen::Vector4f uwb_distance;
    Eigen::MatrixXf R(4, 4);

    uwb_distance << data.d0, data.d1, data.d2, data.d3;
 
    R.block(0, 0, 4, 4) = 0.1 * Eigen::MatrixXf::Identity(4, 4);
   
    pose_estimator(data.header.stamp.toSec(), uwb_distance, R);
}

int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "uav_pose_estimator_node"); 
    ros::NodeHandle n; 
    ros::NodeHandle np("~");

    states_pub = n.advertise<uav::uav_states>("states", 1000);
    imu_sub = n.subscribe("imu", 1000, imuCallback);
    uwb_sub = n.subscribe("uwb", 1000, uwbCallback);

    ros::spin();

    return 0; 
} 

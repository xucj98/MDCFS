#include <ros/ros.h>
#include <uav/UWB.h>
#include <uav/uav_states.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>

// anchor position
const Eigen::Vector3f ap0(   0,    0,  2.4);
const Eigen::Vector3f ap1( 5.1,  0.2,  2.2);
const Eigen::Vector3f ap2( 5.3,  4.3,  2.5);
const Eigen::Vector3f ap3(-0.6,  4.0,  1.4);

#define sqr(x) (x*x)

static bool initialized = false;
static float last_time;
static Eigen::VectorXf x(25); // uav states
static Eigen::Vector3f p; // uav position in world frame.
static Eigen::Quaternion<float> q; // uav coordinate rotation in world frame.
static Eigen::Vector3f v; // uav linear velocity in world frame.
static Eigen::Vector3f a; // uav linear acceleration in world frame.
static Eigen::Vector3f w; // uav coordinate angular velocity in world frame.
static Eigen::Vector3f ba; // imu linear acceleration bias.
static Eigen::Vector3f bw; // imu angular velocity bias.
static Eigen::Vector3f g; // gravity acceleration in world frame.
static Eigen::MatrixXf P(25, 25); // covariance matrix of estimate
static Eigen::MatrixXf Q(25, 25); // covariance matrix of inference
// static Eigen::MatrixXf R(10, 10); // covariance matrix of observation

static ros::Publisher states_pub;
static ros::Subscriber imu_sub;
static ros::Subscriber uwb_sub;

static std::vector<sensor_msgs::Imu> imu_data;
static std::vector<uav::UWB> uwb_data;

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
    Eigen::Vector3f imu_a, 
    Eigen::Vector3f imu_w,
    Eigen::MatrixXf R)
{
    static int imu_init_cnt = 0;
    static int uwb_init_cnt = 0;

    // initialized system
    if (!initialized)
    {
        ROS_INFO("initialized %d %d", imu_init_cnt, uwb_init_cnt);
        last_time = curr_time;

        // estimate initial value of position
        Eigen::Matrix3f A;
        Eigen::Vector3f B;

        if (uwb_init_cnt == 0 && imu_init_cnt == 0)
        {
            p << 0, 0, 0;
            q.w() = 1; q.vec() << 0, 0, 0;
            v << 0, 0, 0;
            a << 0, 0, 0;
            w << 0, 0, 0;
            ba << 0, 0, 0;
            bw << 0, 0, 0;
            g << 0, 0, 9.8;              

            Q.setZero(25, 25);
        }

        if (R(0, 0) < 1) // uwb data is available
        {
            A << (ap0(0) - ap1(0)) * 2, (ap0(1) - ap1(1)) * 2, (ap0(2) - ap1(2)) * 2,
                (ap0(0) - ap2(0)) * 2, (ap0(1) - ap2(1)) * 2, (ap0(2) - ap2(2)) * 2,
                (ap0(0) - ap3(0)) * 2, (ap0(1) - ap3(1)) * 2, (ap0(2) - ap3(2)) * 2;

            B << sqr(uwb_d(1)) - sqr(uwb_d(0)) + sqr(ap0(0)) - sqr(ap1(0)) + sqr(ap0(1)) - sqr(ap1(1)) + sqr(ap0(2)) - sqr(ap1(2)),
                sqr(uwb_d(2)) - sqr(uwb_d(0)) + sqr(ap0(0)) - sqr(ap2(0)) + sqr(ap0(1)) - sqr(ap2(1)) + sqr(ap0(2)) - sqr(ap2(2)),
                sqr(uwb_d(3)) - sqr(uwb_d(0)) + sqr(ap0(0)) - sqr(ap3(0)) + sqr(ap0(1)) - sqr(ap3(1)) + sqr(ap0(2)) - sqr(ap3(2));

            p += A.inverse() * B;
            uwb_init_cnt += 1;
        }
        
        if (R(4, 4) < 1) // imu data is available
        {
            bw += imu_w;
            imu_init_cnt += 1;
        }

        if (uwb_init_cnt >= 10)
        {
            initialized = true;
            bw /= imu_init_cnt;
            p /= uwb_init_cnt;
            ROS_INFO("initial position %f %f %f", p(0), p(1), p(2));
        }
        return;
    }

    
    float dt = curr_time - last_time;

    // get observation zo
    Eigen::VectorXf zo(10);
    zo.segment(0, 4) = uwb_d.array().square();
    zo.segment(4, 3) = imu_a;
    zo.segment(7, 3) = imu_w;

    std::cout << "==================== observation ========================";
    std::cout << std::fixed << std::setprecision(2) << zo << std::endl << std::endl;
    
    // estimate xe = f(x)
    Eigen::Vector3f xe_p, xe_v, xe_a, xe_w, xe_ba, xe_bw, xe_g; 
    Eigen::Quaternion<float> xe_q, xe_dq; 
    
    xe_p = p + dt * v + 0.5 * dt * dt * a;
    
    xe_dq.w() = 1; xe_dq.vec() = 0.5 * dt * w; 
    xe_dq = xe_dq.normalized();
    xe_q = xe_dq * q;

    xe_v = v + dt * a;
    
    xe_a = a;
    xe_w = w;
    xe_ba = ba;
    xe_bw = bw;
    xe_g = g;

    // calculate F = Jacobi(f, x)
    Eigen::MatrixXf F(25, 25);
    F.setZero(25, 25);
    
    // Jacobi(xe_p, (p, v, a))
    F.block(0, 0, 3, 3) = Eigen::Matrix3f::Identity();
    F.block(0, 7, 3, 3) = dt * Eigen::Matrix3f::Identity();
    F.block(0, 10, 3, 3) = 0.5 * dt * dt * Eigen::Matrix3f::Identity();

    // Jacobi(xe_q, q)
    F(3, 3) = 1;
    F.block(3, 4, 1, 3) = -0.5 * dt * Eigen::MatrixXf(w).transpose();
    F.block(4, 3, 3, 1) = 0.5 * dt * Eigen::MatrixXf(w);
    F.block(4, 4, 3, 3) = Eigen::Matrix3f::Identity() + 0.5 * dt * crossMatrix(w);
    
    // Jacobi(xe_q, w)
    F.block(3, 13, 1, 3) << -0.5 * dt * Eigen::MatrixXf(q.vec()).transpose();
    F.block(4, 13, 3, 3) = 0.5 * dt * (q.w() * Eigen::Matrix3f::Identity() - crossMatrix(q.vec()));

    // Jacobi(xe_v, (v, a))
    F.block(7, 7, 3, 3) = Eigen::Matrix3f::Identity();
    F.block(7, 10, 3, 3) = dt * Eigen::Matrix3f::Identity();

    // Jacobi((xe_a, xe_w, xe_ba, xe_bw, xe_g), (a, w, ba, bw, g))
    F.block(10, 10, 15, 15) = Eigen::MatrixXf::Identity(15, 15);

    std::cout << "==================== Matrix F ========================";
    std::cout << std::fixed << std::setprecision(2) << F << std::endl << std::endl;
    
    // estimate ze = h(xe)
    Eigen::Vector4f ze_d;
    Eigen::Vector3f ze_a, ze_w;
    Eigen::VectorXf ze(10);

    ze_d(0) = (xe_p - ap0).squaredNorm();
    ze_d(1) = (xe_p - ap1).squaredNorm();
    ze_d(2) = (xe_p - ap2).squaredNorm();
    ze_d(3) = (xe_p - ap3).squaredNorm();
    
    Eigen::Quaternion<float> tmp_q;
    tmp_q.w() = 0; tmp_q.vec() = xe_a + xe_g;
    tmp_q = xe_q.inverse() * tmp_q * xe_q;
    ze_a = tmp_q.vec() + xe_ba;

    tmp_q.w() = 0; tmp_q.vec() = xe_w;
    tmp_q = xe_q.inverse() * tmp_q * xe_q;
    ze_w = tmp_q.vec() + xe_bw;

    ze.segment(0, 4) = ze_d;
    ze.segment(4, 3) = ze_a;
    ze.segment(7, 3) = ze_w;

    std::cout << "==================== estimate observation ========================";
    std::cout << std::fixed << std::setprecision(2) << ze << std::endl << std::endl;

    // calculate H = Jacobi(h, xe)
    Eigen::MatrixXf H(10, 25);
    H.setZero(10, 25);

    // Jacobi(ze_d, xe_p)
    H.block(0, 0, 1, 3) = Eigen::MatrixXf(2 * (xe_p - ap0)).transpose();
    H.block(1, 0, 1, 3) = Eigen::MatrixXf(2 * (xe_p - ap1)).transpose();
    H.block(2, 0, 1, 3) = Eigen::MatrixXf(2 * (xe_p - ap2)).transpose();
    H.block(3, 0, 1, 3) = Eigen::MatrixXf(2 * (xe_p - ap3)).transpose();

    // rotation matrix of xe_q*
    Eigen::Matrix3f R_xe_q;
    R_xe_q = (sqr(xe_q.w()) - xe_q.vec().squaredNorm()) * Eigen::Matrix3f::Identity() +
        2 * Eigen::MatrixXf(xe_q.vec()) * Eigen::MatrixXf(xe_q.vec()).transpose() +
        - 2 * xe_q.w() * crossMatrix(xe_q.vec());
    // Jacobi(ze_a, xe_a, xe_ba, xe_g)
    H.block(4, 10, 3, 3) = R_xe_q;
    H.block(4, 16, 3, 3) = Eigen::Matrix3f::Identity();
    H.block(4, 22, 3, 3) = R_xe_q;
    
    // Jacobi(ze_a, xe_q)
    H.block(4, 3, 3, 1) = 2 * (xe_q.w() * Eigen::Matrix3f::Identity() - crossMatrix(xe_q.vec())) * (xe_a + xe_ba + xe_g);
    Eigen::Vector3f a_w = xe_a + xe_g;
    H.block(4, 4, 3, 3) = 2 * a_w.dot(xe_q.vec()) * Eigen::Matrix3f::Identity() +
                          2 * xe_q.w() *crossMatrix(a_w) +
                          2 * Eigen::MatrixXf(xe_q.vec()) * Eigen::MatrixXf(a_w).transpose() - 
                          2 * Eigen::MatrixXf(a_w) * Eigen::MatrixXf(xe_q.vec()).transpose();

    // Jacobi(ze_w, (xe_w, we_bw))
    H.block(7, 13, 3, 3) = R_xe_q;
    H.block(7, 19, 3, 3) = Eigen::Matrix3f::Identity();

    // Jacobi(ze_w, xe_q)
    H.block(4, 4, 3, 3) = 2 * xe_w.dot(xe_q.vec()) * Eigen::Matrix3f::Identity() +
                          2 * xe_q.w() *crossMatrix(xe_w) +
                          2 * Eigen::MatrixXf(xe_q.vec()) * Eigen::MatrixXf(xe_w).transpose() - 
                          2 * Eigen::MatrixXf(xe_w) * Eigen::MatrixXf(xe_q.vec()).transpose();
    
    std::cout << "==================== Matrix H ========================";
    std::cout << std::fixed << std::setprecision(2) << H << std::endl << std::endl;


    Eigen::MatrixXf Pe = F * P * F.transpose() + Q * dt;
    Eigen::VectorXf y = zo - ze;
    Eigen::MatrixXf S = H * Pe * H.transpose() + R;
    Eigen::MatrixXf K = Pe * H.transpose() * S.inverse();
    
    p       = xe_p       +  K.block( 0, 0, 3, 10) * y;
    q.w()   = xe_q.w()   + (K.block( 3, 0, 1, 10) * y)(0);
    q.vec() = xe_q.vec() +  K.block( 4, 0, 3, 10) * y;
    v       = xe_v       +  K.block( 7, 0, 3, 10) * y;
    a       = xe_a       +  K.block(10, 0, 3, 10) * y;
    w       = xe_w       +  K.block(13, 0, 3, 10) * y;
    ba      = xe_ba      +  K.block(16, 0, 3, 10) * y;
    bw      = xe_bw      +  K.block(19, 0, 3, 10) * y;
    g       = xe_g       +  K.block(22, 0, 3, 10) * y;
    q = q.normalized();
    
    P = (Eigen::MatrixXf::Identity(25, 25) - K * H) * Pe;

    ROS_INFO("position: %f %f %f", p.x(), p.y(), p.z());
    Eigen::Vector3f eulerAngles = q.toRotationMatrix().eulerAngles(2, 1, 0);
    ROS_INFO("orientation: %f %f %f", eulerAngles(0), eulerAngles(1), eulerAngles(2));

    uav::uav_states states_msg;
    states_msg.position.x = p.x();
    states_msg.position.y = p.y();
    states_msg.position.z = p.z();
    states_msg.orientation.w = q.w();
    states_msg.orientation.x = q.x();
    states_msg.orientation.y = q.y();
    states_msg.orientation.z = q.z();
    states_msg.linear_velocity.x = v.x();
    states_msg.linear_velocity.y = v.y();
    states_msg.linear_velocity.z = v.z();
    states_msg.linear_acceleration.x = a.x();
    states_msg.linear_acceleration.y = a.y();
    states_msg.linear_acceleration.z = a.z();
    states_msg.angular_velocity.x = w.x();
    states_msg.angular_velocity.y = w.y();
    states_msg.angular_velocity.z = w.z();
    states_msg.bias_linear_acceleration.x = ba.x();
    states_msg.bias_linear_acceleration.y = ba.y();
    states_msg.bias_linear_acceleration.z = ba.z();
    states_msg.bias_angular_velocity.x = bw.x();
    states_msg.bias_angular_velocity.y = bw.y();
    states_msg.bias_angular_velocity.z = bw.z();
    states_msg.gravity.x = g.x();
    states_msg.gravity.y = g.y();
    states_msg.gravity.z = g.z();
    
    states_pub.publish(states_msg);

    last_time = curr_time;

    while(ros::ok());
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu data;
    data = *msg;
    imu_data.push_back(data);
    // ROS_INFO("imu time stamp: %lf", data.header.stamp.toSec());
    // ROS_INFO("linear acceleration: %f %f %f", data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z);
    // ROS_INFO("angular velocity: %f %f %f", data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z);
    
    Eigen::Vector4f uwb_distance;
    Eigen::Vector3f linear_acceleration;
    Eigen::Vector3f angular_velocity;
    Eigen::MatrixXf R(10, 10);

    uwb_distance << 0, 0, 0, 0;
    linear_acceleration << data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z;
    angular_velocity << data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z;
    R.setZero(10, 10);
    R.block(0, 0, 4, 4) = 100 * Eigen::MatrixXf::Identity(4, 4);
    R.block(4, 4, 3, 3) = 0.1 * Eigen::MatrixXf::Identity(3, 3);
    R.block(7, 7, 3, 3) = 0.001 * Eigen::MatrixXf::Identity(3, 3);

    pose_estimator(data.header.stamp.toSec(), uwb_distance, linear_acceleration, angular_velocity, R);
}

void uwbCallback(const uav::UWB::ConstPtr& msg)
{
    uav::UWB data;
    data = *msg;
    uwb_data.push_back(data);
    // ROS_INFO("uwb distance: %f %f %f %f", data.d0, data.d1, data.d2, data.d3);
     
    Eigen::Vector4f uwb_distance;
    Eigen::Vector3f linear_acceleration;
    Eigen::Vector3f angular_velocity;
    Eigen::MatrixXf R(10, 10);

    uwb_distance << data.d0, data.d1, data.d2, data.d3;
    linear_acceleration << 0, 0, 0;
    angular_velocity << 0, 0, 0;
    R.setZero(10, 10);
    R.block(0, 0, 4, 4) = 0.01 * Eigen::MatrixXf::Identity(4, 4);
    R.block(4, 4, 3, 3) = 100 * Eigen::MatrixXf::Identity(3, 3);
    R.block(7, 7, 3, 3) = 100 * Eigen::MatrixXf::Identity(3, 3);

    pose_estimator(data.header.stamp.toSec(), uwb_distance, linear_acceleration, angular_velocity, R);
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

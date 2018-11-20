#include <ros/ros.h> 
#include <uav/uav_states.h>
#include <uav/datas_rate.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <pangolin/pangolin.h>
#include <Eigen/Dense>
#include <iostream>

#include <opencv2/opencv.hpp>

const float wheelbase = 0.24;
const float bladelength = 0.14;
const float ctrllength = 0.05;
const float pi = 3.14;

static std::vector<bool> getInof;
static std::vector<Eigen::Vector3f> positions;
static std::vector<Eigen::Quaternion<float> > orientations;
static std::vector<Eigen::Vector3f> colors;
static std::vector<ros::Subscriber> uavs_states_sub;
static std::vector<ros::Subscriber> uavs_states_ref_sub;
static std::vector<ros::Subscriber> uavs_dates_rate_sub;
static std::vector<int> uavs_imu_rate;
static std::vector<int> uavs_uwb_rate;

static cv::Mat infos_img(480, 640, CV_8UC1);

void drawGround()
{
    glColor3f(0.3, 0.3, 0.3);
    for (int i = -40; i <= 40; i++) pangolin::glDrawLine(i * 0.5, -20, 0, i * 0.5, 20, 0);
    for (int i = -40; i <= 40; i++) pangolin::glDrawLine(-20, i * 0.5, 0, 20, i * 0.5, 0);
}

void drawUav(Eigen::Vector3f position, Eigen::Quaternion<float> orientation, Eigen::Vector3f color)
{
    orientation.normalized();

    // do not use orientation
    // orientation.w() = 1;
    // orientation.vec() << 0, 0, 0;

    glColor3f(color(0), color(1), color(2));
    Eigen::Matrix3f R = orientation.toRotationMatrix();
    
    #define rotvec(vec) (R * (vec) + position)(0), (R * (vec) + position)(1), (R * (vec) + position)(2)

    Eigen::Vector3f c1, c2, c3, c4;
    c1 <<  wheelbase / 2.828,  wheelbase / 2.828, 0;
    c2 << -wheelbase / 2.828, -wheelbase / 2.828, 0;
    c3 <<  wheelbase / 2.828, -wheelbase / 2.828, 0;
    c4 << -wheelbase / 2.828,  wheelbase / 2.828, 0;

    Eigen::Vector3f center, head;
    center << 0, 0, 0;
    head << wheelbase, 0, 0;

    pangolin::glDrawLine(rotvec(center), rotvec(head));

    pangolin::glDrawLine(rotvec(c1), rotvec(c2));
    pangolin::glDrawLine(rotvec(c3), rotvec(c4));

    Eigen::Vector3f offset_z;
    offset_z << 0, 0, 0.02;
    pangolin::glDrawLine(rotvec(c1), rotvec(c1 + offset_z));
    pangolin::glDrawLine(rotvec(c2), rotvec(c2 + offset_z));
    pangolin::glDrawLine(rotvec(c3), rotvec(c3 + offset_z));
    pangolin::glDrawLine(rotvec(c4), rotvec(c4 + offset_z));
   
    Eigen::Vector3f ct1, ct2, ct3, ct4;
    ct1 << -ctrllength / 2, -ctrllength / 2, 0.01;
    ct2 <<  ctrllength / 2, -ctrllength / 2, 0.01;
    ct3 <<  ctrllength / 2,  ctrllength / 2, 0.01;
    ct4 << -ctrllength / 2,  ctrllength / 2, 0.01;
    offset_z << 0, 0, -0.02;
    
    pangolin::glDrawLine(rotvec(ct1), rotvec(ct2));
    pangolin::glDrawLine(rotvec(ct2), rotvec(ct3));
    pangolin::glDrawLine(rotvec(ct3), rotvec(ct4));
    pangolin::glDrawLine(rotvec(ct4), rotvec(ct1));
    pangolin::glDrawLine(rotvec(ct1 + offset_z), rotvec(ct2 + offset_z));
    pangolin::glDrawLine(rotvec(ct2 + offset_z), rotvec(ct3 + offset_z));
    pangolin::glDrawLine(rotvec(ct3 + offset_z), rotvec(ct4 + offset_z));
    pangolin::glDrawLine(rotvec(ct4 + offset_z), rotvec(ct1 + offset_z));
    pangolin::glDrawLine(rotvec(ct1), rotvec(ct1 + offset_z));
    pangolin::glDrawLine(rotvec(ct2), rotvec(ct2 + offset_z));
    pangolin::glDrawLine(rotvec(ct3), rotvec(ct3 + offset_z));
    pangolin::glDrawLine(rotvec(ct4), rotvec(ct4 + offset_z));



    for (int i = 0; i < 30; i++)
    {
        Eigen::Vector3f of1, of2;
        of1 << bladelength / 2 * cos(2 * pi * i / 30), bladelength / 2 * sin(2 * pi * i / 30), 0.02;
        of2 << bladelength / 2 * cos(2 * pi * (i + 1) / 30), bladelength / 2 * sin(2 * pi * (i + 1) / 30), 0.02;
        pangolin::glDrawLine(rotvec(c1 + of1), rotvec(c1 + of2));
        pangolin::glDrawLine(rotvec(c2 + of1), rotvec(c2 + of2));
        pangolin::glDrawLine(rotvec(c3 + of1), rotvec(c3 + of2));
        pangolin::glDrawLine(rotvec(c4 + of1), rotvec(c4 + of2)); 
        pangolin::glDrawLine(rotvec(c1), rotvec(c1 + of1));
        pangolin::glDrawLine(rotvec(c2), rotvec(c2 + of1));
        pangolin::glDrawLine(rotvec(c3), rotvec(c3 + of1));
        pangolin::glDrawLine(rotvec(c4), rotvec(c4 + of1));
    
    }    
}

void uavStatesSubCallback(const uav::uav_states::ConstPtr& states, int uav_id)
{
    getInof[uav_id] = true;
    positions[uav_id].x() = states->position.x;
    positions[uav_id].y() = states->position.y;
    positions[uav_id].z() = states->position.z;
    orientations[uav_id].w() = states->orientation.w;
    orientations[uav_id].x() = states->orientation.x;
    orientations[uav_id].y() = states->orientation.y;
    orientations[uav_id].z() = states->orientation.z;
}

void uavDatasRateSubCallback(const uav::datas_rate::ConstPtr &msg, int uav_id)
{
    uavs_imu_rate[uav_id] = msg->imu_rate;
    uavs_uwb_rate[uav_id] = msg->uwb_rate;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_visualization_node"); 
    
    ros::NodeHandle n;
    ros::NodeHandle np("~");

    int uav_count;
    np.param<int>("uav_count", uav_count, 4);

    getInof.resize(uav_count * 2);
    colors.resize(uav_count * 2);
    uavs_states_ref_sub.resize(uav_count);
    uavs_states_sub.resize(uav_count);
    positions.resize(uav_count * 2);
    orientations.resize(uav_count * 2);
    uavs_imu_rate.resize(uav_count);
    uavs_uwb_rate.resize(uav_count);
    uavs_dates_rate_sub.resize(uav_count);

    for (int i = 0; i < uav_count * 2; i++)
    {
        positions[i] << i, 0, 0;
        orientations[i].w() = 1;
        orientations[i].vec() << 0, 0, 0;
        colors[i](0) = (float)(rand() % 256) / 256;
        colors[i](1) = (float)(rand() % 256) / 256;
        colors[i](2) = (float)(rand() % 256) / 256;
        getInof[i] = false;
    }
    for (int i = 0; i < uav_count; i++)
    {
        uavs_states_sub[i] = n.subscribe("uav_" + std::to_string(i) + "/states", 1000, boost::function<void (const uav::uav_states::ConstPtr&)>(boost::bind(uavStatesSubCallback, _1, i)));
        uavs_states_ref_sub[i] = n.subscribe("uav_" + std::to_string(i) + "/states_ref", 1000, boost::function<void (const uav::uav_states::ConstPtr&)>(boost::bind(uavStatesSubCallback, _1, i + uav_count)));
        uavs_dates_rate_sub[i] = n.subscribe("uav_" + std::to_string(i) + "/datas_rate", 1000, boost::function<void (const uav::datas_rate::ConstPtr&)>(boost::bind(uavDatasRateSubCallback, _1, i)));
    } 

    // init pangolin
    pangolin::CreateWindowAndBind("Main", 1080, 720);
    glEnable(GL_DEPTH_TEST);
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1080, 720, 520, 520, 540, 360, 0.2, 100),
        pangolin::ModelViewLookAt(-2, -2, 1, 0, 0, 1.5, pangolin::AxisZ)
    );
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -1080.0f / 720.0f)
            .SetHandler(&handler);

    while(ros::ok())
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        drawGround();
    
        for (int i = 0; i < uav_count * 2; i++)
            if (getInof[i])
                drawUav(positions[i], orientations[i], colors[i]);
        
        
        // Swap frames and Process Events
        pangolin::FinishFrame();

        infos_img.setTo(255);
        for (int i = 0; i < uav_count; i++)
        {
            std::stringstream ss;
            ss << "uav_" << std::to_string(i) << ".position: " << std::fixed << std::setprecision(2) << std::setw(5) << Eigen::MatrixXf(positions[i]).transpose() << ", imu rate: " << uavs_imu_rate[i] << ", uwb rate: " << uavs_uwb_rate[i];
            std::string str = ss.str();
            cv::putText(infos_img, ss.str(), cv::Point(10, i * 30 + 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0), 1, 8);
        }
        cv::imshow("information", infos_img);
        cv::waitKey(1);

        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();
    }
    
    return 0;
}
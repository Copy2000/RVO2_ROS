#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Empty.h>
#include <Eigen/Core>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

ros::Publisher trajectory_pub;
geometry_msgs::PointStamped current_position;

float linear_smoothing_navigation_step = 2;
bool flag_gps_initialized_OK = false;
bool flag_take_off_OK = false;
int flag_tasks_OK = 0;
Eigen::Vector3d home;

/*
Description: 
    updateUavPosition(const geometry_msgs::PointStamped& msg)
    gps数据更新的回调函数.

Parameters:
    msg 位置信息

Return:
    无
*/
void updateUavPosition(const geometry_msgs::PointStamped& msg)
{
    if (!flag_gps_initialized_OK)
    {
        flag_gps_initialized_OK= true;
        home[0] = msg.point.x;
        home[1] = msg.point.y;
        home[2] = msg.point.z;
    }
    current_position = msg;
    // std::cout<<"UAV current position is: "<<msg.point.x<< msg.point.y<< msg.point.z<<std::endl;
}

/*
Description: 
    getDistanceToTarget(const Eigen::Vector3d& target)
    获取当前位置到指定位置位置的距离.

Parameters:
    target 需要飞达的位置点

Return:
    double 当前位置到达目标点的位置
*/
double getDistanceToTarget(const Eigen::Vector3d& target)
{
    double temp = 0;
    temp += pow((target[0] - current_position.point.x), 2);
	temp += pow((target[1] - current_position.point.y), 2);
	temp += pow((target[2] - current_position.point.z), 2);
    temp = sqrt(temp);
    return temp;
}

/*
Description: 
    reachTargetPosition(const Eigen::Vector3d& target, float max_error)
    判定是否到达指定的目标点.

Parameters:
    target 需要飞达的位置点
    max_error 允许的位置误差阈值,当前位置和目标位置小于该阈值时,判定无人机到达目标点

Return:
    bool 到达目标点时返回 true
         未到达目标点时返回 false
*/
bool reachTargetPosition(const Eigen::Vector3d& target, float max_error)
{
    double temp = getDistanceToTarget(target);
	
	if (temp < max_error)
		return true;
	return false;
}

/*
Description: 
    linearSmoothingNavigationTask(const Eigen::Vector3d& target)
    控制无人机从当前位置飞向指定位置.

Parameters:
    target 需要飞达的位置点

Return:
    bool 起飞结束后返回 true
         起飞过程中返回 false
*/
bool linearSmoothingNavigationTask(const Eigen::Vector3d& target)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    if (reachTargetPosition(target,0.2))
        return true;

    double dist = getDistanceToTarget(target);
    Eigen::Vector3d next_step;
    
    if(dist<linear_smoothing_navigation_step)
    {
        next_step = target;
    }
    else
    {
        next_step[0] = current_position.point.x+(target[0]-current_position.point.x)/dist*linear_smoothing_navigation_step;
        next_step[1] = current_position.point.y+(target[1]-current_position.point.y)/dist*linear_smoothing_navigation_step;
        next_step[2] = current_position.point.z+(target[2]-current_position.point.z)/dist*linear_smoothing_navigation_step;
    }
    
    double desired_yaw = 0.0; 
    
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(next_step, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    return false;
}

/*
Description: 
    takeOffTask(float height)
    起飞函数,调用后无人机从起始位置起飞指定高度.

Parameters:
    height 指定的起飞高度

Return:
    bool 起飞结束后返回 true
         起飞过程中返回 false
*/
bool takeOffTask(float height)
{
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();

    static Eigen::Vector3d desired_position(current_position.point.x, current_position.point.y, height);
    double desired_yaw = 0.0;

    if (reachTargetPosition(desired_position,0.2))
        return true;
    
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    return false;
}

/*
Description: 
    gohome()
    反航函数,调用后无人机先沿着当前高度飞到反航点正上方,然后降落.

Parameters:
    无

Return:
    无
*/
void gohome()
{
    static Eigen::Vector3d temp(home[0], home[1], current_position.point.z);
    static bool flag_temp = false;
    
    if (!flag_temp)
    {
        flag_temp = linearSmoothingNavigationTask(temp);
    }
    else
    {
        linearSmoothingNavigationTask(home);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "UAV_Controler");
    ros::NodeHandle nh;
    // Create a private node handle for accessing node parameters.
    ros::NodeHandle nh_private("~");

    std::string uav_name = "";  
    ros::param::get("~mav_name",uav_name);

    // 订阅话题
    // /odometry_sensor1/position   无人机位置信息(包含噪声)
    ros::Subscriber position_sub = nh.subscribe(std::string("/"+uav_name+"/odometry_sensor1/position").c_str(), 10, &updateUavPosition);

    trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    // 等待5s,让Gazebo可以成功启动.
    ros::Duration(5.0).sleep();

    // 创建控制Gazebo自动运行的服务,这里自动运行是指让Gazebo自动Play
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv); 

    // 尝试让Gazebo自动运行
    int i=0;
    while (i <= 10 && !unpaused) {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    // 判断Gazebo有没有自动运行,没有成功Play则退出
    if (!unpaused) {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    } else {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    std::vector<Eigen::Vector3d> path;
path.push_back(Eigen::Vector3d(5.0,0.0,0.6));
path.push_back(Eigen::Vector3d(4.568,-0.012,0.6));
path.push_back(Eigen::Vector3d(4.13631,-0.0556858,0.6));
path.push_back(Eigen::Vector3d(3.7017,-0.0929726,0.6));
path.push_back(Eigen::Vector3d(3.28792,-0.171114,0.6));
path.push_back(Eigen::Vector3d(2.96429,-0.264084,0.6));
path.push_back(Eigen::Vector3d(2.64913,-0.399166,0.6));
path.push_back(Eigen::Vector3d(2.37851,-0.551948,0.6));
path.push_back(Eigen::Vector3d(2.07502,-0.707358,0.6));
path.push_back(Eigen::Vector3d(1.83726,-0.904049,0.6));
path.push_back(Eigen::Vector3d(1.64286,-1.11253,0.6));
path.push_back(Eigen::Vector3d(1.41947,-1.33576,0.6));
path.push_back(Eigen::Vector3d(1.14307,-1.60935,0.6));
path.push_back(Eigen::Vector3d(0.876308,-1.90361,0.6));
path.push_back(Eigen::Vector3d(0.556734,-2.20535,0.6));
path.push_back(Eigen::Vector3d(0.255036,-2.47547,0.6));
path.push_back(Eigen::Vector3d(-0.0409634,-2.7568,0.6));
path.push_back(Eigen::Vector3d(-0.326771,-3.01243,0.6));
path.push_back(Eigen::Vector3d(-0.616932,-3.27271,0.6));
path.push_back(Eigen::Vector3d(-0.913163,-3.55147,0.6));
path.push_back(Eigen::Vector3d(-1.18492,-3.79034,0.6));
path.push_back(Eigen::Vector3d(-1.42311,-4.04397,0.6));
path.push_back(Eigen::Vector3d(-1.70949,-4.29476,0.6));
path.push_back(Eigen::Vector3d(-1.98942,-4.56779,0.6));
path.push_back(Eigen::Vector3d(-2.27256,-4.84986,0.6));
path.push_back(Eigen::Vector3d(-2.49416,-5.07902,0.6));
path.push_back(Eigen::Vector3d(-2.72196,-5.34581,0.6));
path.push_back(Eigen::Vector3d(-2.93521,-5.62034,0.6));
path.push_back(Eigen::Vector3d(-3.09171,-5.9066,0.6));
path.push_back(Eigen::Vector3d(-3.18515,-6.18309,0.6));
path.push_back(Eigen::Vector3d(-3.20029,-6.43661,0.6));
path.push_back(Eigen::Vector3d(-3.17896,-6.65242,0.6));
path.push_back(Eigen::Vector3d(-3.14081,-6.80683,0.6));
path.push_back(Eigen::Vector3d(-3.10952,-6.98762,0.6));
path.push_back(Eigen::Vector3d(-3.07892,-7.18439,0.6));
path.push_back(Eigen::Vector3d(-3.05,-7.42779,0.6));
path.push_back(Eigen::Vector3d(-2.95043,-7.59874,0.6));
path.push_back(Eigen::Vector3d(-2.85388,-7.7424,0.6));
path.push_back(Eigen::Vector3d(-2.7229,-7.87574,0.6));
path.push_back(Eigen::Vector3d(-2.59097,-7.95937,0.6));
path.push_back(Eigen::Vector3d(-2.4278,-7.97654,0.6));
path.push_back(Eigen::Vector3d(-2.26397,-7.93115,0.6));
path.push_back(Eigen::Vector3d(-2.13198,-7.80836,0.6));
path.push_back(Eigen::Vector3d(-2.02292,-7.69942,0.6));
path.push_back(Eigen::Vector3d(-1.9262,-7.56299,0.6));
path.push_back(Eigen::Vector3d(-1.85388,-7.53578,0.6));
path.push_back(Eigen::Vector3d(-1.75886,-7.54713,0.6));
path.push_back(Eigen::Vector3d(-1.69449,-7.57787,0.6));
path.push_back(Eigen::Vector3d(-1.65886,-7.6508,0.6));
path.push_back(Eigen::Vector3d(-1.6125,-7.76725,0.6));
path.push_back(Eigen::Vector3d(-1.60232,-7.92234,0.6));
path.push_back(Eigen::Vector3d(-1.587,-8.09469,0.6));
path.push_back(Eigen::Vector3d(-1.61472,-8.30937,0.6));
path.push_back(Eigen::Vector3d(-1.64924,-8.54076,0.6));
path.push_back(Eigen::Vector3d(-1.73084,-8.72075,0.6));
path.push_back(Eigen::Vector3d(-1.84505,-8.85429,0.6));
path.push_back(Eigen::Vector3d(-1.99857,-8.90993,0.6));
path.push_back(Eigen::Vector3d(-2.19537,-8.93438,0.6));
path.push_back(Eigen::Vector3d(-2.3142,-8.90352,0.6));
path.push_back(Eigen::Vector3d(-2.28704,-8.77277,0.6));
path.push_back(Eigen::Vector3d(-2.37649,-8.50445,0.6));
path.push_back(Eigen::Vector3d(-2.49374,-8.24011,0.6));
path.push_back(Eigen::Vector3d(-2.58584,-7.97559,0.6));
path.push_back(Eigen::Vector3d(-2.67356,-7.72301,0.6));
path.push_back(Eigen::Vector3d(-2.76089,-7.45837,0.6));
path.push_back(Eigen::Vector3d(-2.87996,-7.2217,0.6));
path.push_back(Eigen::Vector3d(-3.00126,-6.96076,0.6));
path.push_back(Eigen::Vector3d(-3.12085,-6.69957,0.6));
path.push_back(Eigen::Vector3d(-3.22667,-6.46213,0.6));
path.push_back(Eigen::Vector3d(-3.32286,-6.21247,0.6));
path.push_back(Eigen::Vector3d(-3.42587,-5.97464,0.6));
path.push_back(Eigen::Vector3d(-3.52713,-5.70059,0.6));
path.push_back(Eigen::Vector3d(-3.63501,-5.45435,0.6));
path.push_back(Eigen::Vector3d(-3.72071,-5.18383,0.6));
path.push_back(Eigen::Vector3d(-3.82148,-4.9292,0.6));
path.push_back(Eigen::Vector3d(-3.91615,-4.68632,0.6));
path.push_back(Eigen::Vector3d(-4.00071,-4.41118,0.6));
path.push_back(Eigen::Vector3d(-4.08798,-4.1359,0.6));
path.push_back(Eigen::Vector3d(-4.17761,-3.86843,0.6));
path.push_back(Eigen::Vector3d(-4.26907,-3.59273,0.6));
path.push_back(Eigen::Vector3d(-4.33803,-3.34477,0.6));
path.push_back(Eigen::Vector3d(-4.42146,-3.06866,0.6));
path.push_back(Eigen::Vector3d(-4.48232,-2.8123,0.6));
path.push_back(Eigen::Vector3d(-4.56195,-2.55582,0.6));
path.push_back(Eigen::Vector3d(-4.61407,-2.27903,0.6));
path.push_back(Eigen::Vector3d(-4.70163,-2.0142,0.6));
path.push_back(Eigen::Vector3d(-4.7513,-1.77289,0.6));
path.push_back(Eigen::Vector3d(-4.82678,-1.52343,0.6));
path.push_back(Eigen::Vector3d(-4.87884,-1.2735,0.6));
path.push_back(Eigen::Vector3d(-4.93507,-1.0348,0.6));
path.push_back(Eigen::Vector3d(-4.97606,-0.867838,0.6));
path.push_back(Eigen::Vector3d(-5.01685,-0.734271,0.6));
path.push_back(Eigen::Vector3d(-5.04948,-0.615417,0.6));
path.push_back(Eigen::Vector3d(-5.08261,-0.520589,0.6));
path.push_back(Eigen::Vector3d(-5.10295,-0.409809,0.6));
path.push_back(Eigen::Vector3d(-5.12856,-0.314757,0.6));
path.push_back(Eigen::Vector3d(-5.15359,-0.223073,0.6));
path.push_back(Eigen::Vector3d(-5.16959,-0.247073,0.6));
path.push_back(Eigen::Vector3d(-5.18959,-0.275073,0.6));
path.push_back(Eigen::Vector3d(-5.17405,-0.245333,0.6));
path.push_back(Eigen::Vector3d(-5.19005,-0.277333,0.6));
path.push_back(Eigen::Vector3d(-5.16804,-0.257866,0.6));
path.push_back(Eigen::Vector3d(-5.18804,-0.265866,0.6));
path.push_back(Eigen::Vector3d(-5.17843,-0.240693,0.6));
path.push_back(Eigen::Vector3d(-5.21443,-0.260693,0.6));
path.push_back(Eigen::Vector3d(-5.21155,-0.244554,0.6));
path.push_back(Eigen::Vector3d(-5.19724,-0.219643,0.6));
path.push_back(Eigen::Vector3d(-5.22924,-0.231643,0.6));
path.push_back(Eigen::Vector3d(-5.21539,-0.225315,0.6));
path.push_back(Eigen::Vector3d(-5.22739,-0.233315,0.6));
path.push_back(Eigen::Vector3d(-5.18991,-0.218652,0.6));
path.push_back(Eigen::Vector3d(-5.21791,-0.222652,0.6));
path.push_back(Eigen::Vector3d(-5.24591,-0.258652,0.6));
path.push_back(Eigen::Vector3d(-5.24479,-0.240897,0.6));
path.push_back(Eigen::Vector3d(-5.25109,-0.197492,0.6));
path.push_back(Eigen::Vector3d(-5.23298,-0.159804,0.6));
path.push_back(Eigen::Vector3d(-5.23698,-0.171804,0.6));
path.push_back(Eigen::Vector3d(-5.24098,-0.183804,0.6));
path.push_back(Eigen::Vector3d(-5.28098,-0.223804,0.6));
path.push_back(Eigen::Vector3d(-5.28339,-0.200981,0.6));
path.push_back(Eigen::Vector3d(-5.29645,-0.181711,0.6));
path.push_back(Eigen::Vector3d(-5.31777,-0.166708,0.6));
path.push_back(Eigen::Vector3d(-5.31179,-0.151924,0.6));
path.push_back(Eigen::Vector3d(-5.34029,-0.133394,0.6));
path.push_back(Eigen::Vector3d(-5.34806,-0.123051,0.6));
path.push_back(Eigen::Vector3d(-5.32866,-0.120921,0.6));
path.push_back(Eigen::Vector3d(-5.28201,-0.113641,0.6));
path.push_back(Eigen::Vector3d(-5.32201,-0.125641,0.6));
path.push_back(Eigen::Vector3d(-5.29435,-0.110202,0.6));
path.push_back(Eigen::Vector3d(-5.31835,-0.114202,0.6));
path.push_back(Eigen::Vector3d(-5.29468,-0.107362,0.6));
path.push_back(Eigen::Vector3d(-5.31868,-0.131362,0.6));
path.push_back(Eigen::Vector3d(-5.29095,-0.12109,0.6));
path.push_back(Eigen::Vector3d(-5.30295,-0.13709,0.6));
path.push_back(Eigen::Vector3d(-5.27436,-0.149672,0.6));
path.push_back(Eigen::Vector3d(-5.29436,-0.185672,0.6));
path.push_back(Eigen::Vector3d(-5.26348,-0.156537,0.6));
path.push_back(Eigen::Vector3d(-5.27148,-0.188537,0.6));
path.push_back(Eigen::Vector3d(-5.24519,-0.17483,0.6));
path.push_back(Eigen::Vector3d(-5.28519,-0.21483,0.6));
path.push_back(Eigen::Vector3d(-5.26815,-0.179864,0.6));
path.push_back(Eigen::Vector3d(-5.23452,-0.179891,0.6));
path.push_back(Eigen::Vector3d(-5.27052,-0.207891,0.6));
path.push_back(Eigen::Vector3d(-5.23242,-0.206313,0.6));
path.push_back(Eigen::Vector3d(-5.23642,-0.222313,0.6));
path.push_back(Eigen::Vector3d(-5.20113,-0.18985,0.6));
path.push_back(Eigen::Vector3d(-5.20913,-0.22985,0.6));
path.push_back(Eigen::Vector3d(-5.24113,-0.23385,0.6));
path.push_back(Eigen::Vector3d(-5.22091,-0.20708,0.6));
path.push_back(Eigen::Vector3d(-5.23691,-0.21508,0.6));
path.push_back(Eigen::Vector3d(-5.20952,-0.212064,0.6));
path.push_back(Eigen::Vector3d(-5.24952,-0.216064,0.6));
path.push_back(Eigen::Vector3d(-5.21562,-0.180851,0.6));
path.push_back(Eigen::Vector3d(-5.24762,-0.196851,0.6));
path.push_back(Eigen::Vector3d(-5.2021,-0.181481,0.6));
path.push_back(Eigen::Vector3d(-5.2301,-0.221481,0.6));
path.push_back(Eigen::Vector3d(-5.19208,-0.217185,0.6));
path.push_back(Eigen::Vector3d(-5.20808,-0.245185,0.6));
path.push_back(Eigen::Vector3d(-5.20246,-0.228148,0.6));
path.push_back(Eigen::Vector3d(-5.22646,-0.244148,0.6));
path.push_back(Eigen::Vector3d(-5.19717,-0.231318,0.6));



    std::cout << path.size() << std::endl;

    ros::Rate loop_rate(10);
    while (ros::ok())
    { 
        if(flag_gps_initialized_OK && !flag_take_off_OK)
        {
            // ROS_INFO("UAV take off task is running...");
            flag_take_off_OK = takeOffTask(3);
        }
        else if(flag_take_off_OK && flag_tasks_OK<path.size())
        {
            if(flag_tasks_OK<path.size())
            {
                
                bool temp = linearSmoothingNavigationTask(path[flag_tasks_OK]);
                if (temp)
                    flag_tasks_OK ++;
            }
        }
      

        ros::spinOnce();
        loop_rate.sleep();
    }
}
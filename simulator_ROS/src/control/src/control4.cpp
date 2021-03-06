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
    path.push_back(Eigen::Vector3d(-5.0,-5.0,0.6));
path.push_back(Eigen::Vector3d(-4.78506,-4.80106,0.6));
path.push_back(Eigen::Vector3d(-4.62431,-4.61936,0.6));
path.push_back(Eigen::Vector3d(-4.50578,-4.51624,0.6));
path.push_back(Eigen::Vector3d(-4.39978,-4.44458,0.6));
path.push_back(Eigen::Vector3d(-4.31518,-4.40372,0.6));
path.push_back(Eigen::Vector3d(-4.32246,-4.43351,0.6));
path.push_back(Eigen::Vector3d(-4.37787,-4.48715,0.6));
path.push_back(Eigen::Vector3d(-4.50414,-4.58393,0.6));
path.push_back(Eigen::Vector3d(-4.68808,-4.72904,0.6));
path.push_back(Eigen::Vector3d(-4.93415,-4.9041,0.6));
path.push_back(Eigen::Vector3d(-5.1977,-5.14806,0.6));
path.push_back(Eigen::Vector3d(-5.46041,-5.38787,0.6));
path.push_back(Eigen::Vector3d(-5.72661,-5.63258,0.6));
path.push_back(Eigen::Vector3d(-6.0139,-5.91013,0.6));
path.push_back(Eigen::Vector3d(-6.31587,-6.20899,0.6));
path.push_back(Eigen::Vector3d(-6.62412,-6.53771,0.6));
path.push_back(Eigen::Vector3d(-6.87703,-6.86997,0.6));
path.push_back(Eigen::Vector3d(-7.15986,-7.21429,0.6));
path.push_back(Eigen::Vector3d(-7.44308,-7.55184,0.6));
path.push_back(Eigen::Vector3d(-7.70523,-7.89597,0.6));
path.push_back(Eigen::Vector3d(-7.92931,-8.23399,0.6));
path.push_back(Eigen::Vector3d(-8.15549,-8.58769,0.6));
path.push_back(Eigen::Vector3d(-8.36711,-8.95999,0.6));
path.push_back(Eigen::Vector3d(-8.56837,-9.33652,0.6));
path.push_back(Eigen::Vector3d(-8.76683,-9.69648,0.6));
path.push_back(Eigen::Vector3d(-8.97216,-10.047,0.6));
path.push_back(Eigen::Vector3d(-9.1592,-10.3889,0.6));
path.push_back(Eigen::Vector3d(-9.33781,-10.73,0.6));
path.push_back(Eigen::Vector3d(-9.4765,-11.1075,0.6));
path.push_back(Eigen::Vector3d(-9.60563,-11.4561,0.6));
path.push_back(Eigen::Vector3d(-9.72387,-11.7451,0.6));
path.push_back(Eigen::Vector3d(-9.85458,-11.9821,0.6));
path.push_back(Eigen::Vector3d(-9.90848,-12.2465,0.6));
path.push_back(Eigen::Vector3d(-9.86261,-12.5602,0.6));
path.push_back(Eigen::Vector3d(-9.7705,-12.8725,0.6));
path.push_back(Eigen::Vector3d(-9.61407,-13.2142,0.6));
path.push_back(Eigen::Vector3d(-9.44451,-13.5422,0.6));
path.push_back(Eigen::Vector3d(-9.30052,-13.8787,0.6));
path.push_back(Eigen::Vector3d(-9.15404,-14.2398,0.6));
path.push_back(Eigen::Vector3d(-8.98649,-14.6007,0.6));
path.push_back(Eigen::Vector3d(-8.75892,-14.8744,0.6));
path.push_back(Eigen::Vector3d(-8.47934,-15.1253,0.6));
path.push_back(Eigen::Vector3d(-8.17687,-15.2765,0.6));
path.push_back(Eigen::Vector3d(-7.87596,-15.47,0.6));
path.push_back(Eigen::Vector3d(-7.57756,-15.6588,0.6));
path.push_back(Eigen::Vector3d(-7.28293,-15.7892,0.6));
path.push_back(Eigen::Vector3d(-6.9742,-15.8539,0.6));
path.push_back(Eigen::Vector3d(-6.70936,-15.8278,0.6));
path.push_back(Eigen::Vector3d(-6.4742,-15.7668,0.6));
path.push_back(Eigen::Vector3d(-6.27243,-15.6101,0.6));
path.push_back(Eigen::Vector3d(-6.08953,-15.406,0.6));
path.push_back(Eigen::Vector3d(-5.9688,-15.1507,0.6));
path.push_back(Eigen::Vector3d(-5.82912,-14.8908,0.6));
path.push_back(Eigen::Vector3d(-5.71741,-14.6588,0.6));
path.push_back(Eigen::Vector3d(-5.5832,-14.4661,0.6));
path.push_back(Eigen::Vector3d(-5.40429,-14.2871,0.6));
path.push_back(Eigen::Vector3d(-5.23386,-14.1069,0.6));
path.push_back(Eigen::Vector3d(-5.05338,-13.9409,0.6));
path.push_back(Eigen::Vector3d(-4.84299,-13.7859,0.6));
path.push_back(Eigen::Vector3d(-4.65113,-13.6326,0.6));
path.push_back(Eigen::Vector3d(-4.44695,-13.4598,0.6));
path.push_back(Eigen::Vector3d(-4.25092,-13.2931,0.6));
path.push_back(Eigen::Vector3d(-4.07929,-13.128,0.6));
path.push_back(Eigen::Vector3d(-3.91685,-12.9527,0.6));
path.push_back(Eigen::Vector3d(-3.75647,-12.7268,0.6));
path.push_back(Eigen::Vector3d(-3.64013,-12.5009,0.6));
path.push_back(Eigen::Vector3d(-3.54029,-12.2419,0.6));
path.push_back(Eigen::Vector3d(-3.41142,-11.9954,0.6));
path.push_back(Eigen::Vector3d(-3.30729,-11.7364,0.6));
path.push_back(Eigen::Vector3d(-3.19482,-11.5055,0.6));
path.push_back(Eigen::Vector3d(-3.06632,-11.2466,0.6));
path.push_back(Eigen::Vector3d(-2.96582,-10.9838,0.6));
path.push_back(Eigen::Vector3d(-2.86487,-10.737,0.6));
path.push_back(Eigen::Vector3d(-2.73559,-10.4743,0.6));
path.push_back(Eigen::Vector3d(-2.61028,-10.2357,0.6));
path.push_back(Eigen::Vector3d(-2.48107,-10.005,0.6));
path.push_back(Eigen::Vector3d(-2.37607,-9.76621,0.6));
path.push_back(Eigen::Vector3d(-2.25885,-9.53551,0.6));
path.push_back(Eigen::Vector3d(-2.12966,-9.29681,0.6));
path.push_back(Eigen::Vector3d(-2.00064,-9.07007,0.6));
path.push_back(Eigen::Vector3d(-1.89989,-8.83925,0.6));
path.push_back(Eigen::Vector3d(-1.77089,-8.5885,0.6));
path.push_back(Eigen::Vector3d(-1.66197,-8.34573,0.6));
path.push_back(Eigen::Vector3d(-1.54883,-8.11102,0.6));
path.push_back(Eigen::Vector3d(-1.42761,-7.86032,0.6));
path.push_back(Eigen::Vector3d(-1.32633,-7.59764,0.6));
path.push_back(Eigen::Vector3d(-1.20048,-7.35111,0.6));
path.push_back(Eigen::Vector3d(-1.10267,-7.11256,0.6));
path.push_back(Eigen::Vector3d(-0.972426,-6.88612,0.6));
path.push_back(Eigen::Vector3d(-0.854508,-6.6596,0.6));
path.push_back(Eigen::Vector3d(-0.744672,-6.42106,0.6));
path.push_back(Eigen::Vector3d(-0.634631,-6.19057,0.6));
path.push_back(Eigen::Vector3d(-0.528465,-5.93211,0.6));
path.push_back(Eigen::Vector3d(-0.425782,-5.68179,0.6));
path.push_back(Eigen::Vector3d(-0.302569,-5.45559,0.6));
path.push_back(Eigen::Vector3d(-0.187545,-5.21335,0.6));
path.push_back(Eigen::Vector3d(-0.0563397,-4.97516,0.6));
path.push_back(Eigen::Vector3d(0.0466238,-4.7129,0.6));
path.push_back(Eigen::Vector3d(0.142315,-4.45483,0.6));
path.push_back(Eigen::Vector3d(0.270899,-4.21699,0.6));
path.push_back(Eigen::Vector3d(0.38332,-3.9831,0.6));
path.push_back(Eigen::Vector3d(0.48394,-3.74527,0.6));
path.push_back(Eigen::Vector3d(0.59714,-3.51158,0.6));
path.push_back(Eigen::Vector3d(0.726545,-3.28195,0.6));
path.push_back(Eigen::Vector3d(0.84765,-3.02824,0.6));
path.push_back(Eigen::Vector3d(0.949041,-2.77061,0.6));
path.push_back(Eigen::Vector3d(1.08339,-2.50922,0.6));
path.push_back(Eigen::Vector3d(1.2018,-2.25985,0.6));
path.push_back(Eigen::Vector3d(1.33259,-2.00258,0.6));
path.push_back(Eigen::Vector3d(1.4355,-1.75734,0.6));
path.push_back(Eigen::Vector3d(1.55129,-1.52033,0.6));
path.push_back(Eigen::Vector3d(1.66741,-1.25941,0.6));
path.push_back(Eigen::Vector3d(1.76834,-1.03071,0.6));
path.push_back(Eigen::Vector3d(1.89808,-0.774216,0.6));
path.push_back(Eigen::Vector3d(2.00812,-0.525807,0.6));
path.push_back(Eigen::Vector3d(2.13114,-0.301661,0.6));
path.push_back(Eigen::Vector3d(2.26608,-0.073496,0.6));
path.push_back(Eigen::Vector3d(2.38451,0.16281,0.6));
path.push_back(Eigen::Vector3d(2.49136,0.403001,0.6));
path.push_back(Eigen::Vector3d(2.62736,0.642879,0.6));
path.push_back(Eigen::Vector3d(2.75509,0.890831,0.6));
path.push_back(Eigen::Vector3d(2.88323,1.14267,0.6));
path.push_back(Eigen::Vector3d(2.99993,1.40236,0.6));
path.push_back(Eigen::Vector3d(3.10628,1.62559,0.6));
path.push_back(Eigen::Vector3d(3.22982,1.88049,0.6));
path.push_back(Eigen::Vector3d(3.33877,2.13899,0.6));
path.push_back(Eigen::Vector3d(3.4787,2.36864,0.6));
path.push_back(Eigen::Vector3d(3.59006,2.61845,0.6));
path.push_back(Eigen::Vector3d(3.70852,2.86335,0.6));
path.push_back(Eigen::Vector3d(3.86173,3.08344,0.6));
path.push_back(Eigen::Vector3d(4.00061,3.31621,0.6));
path.push_back(Eigen::Vector3d(4.1154,3.55702,0.6));
path.push_back(Eigen::Vector3d(4.27049,3.78053,0.6));
path.push_back(Eigen::Vector3d(4.40439,4.00842,0.6));
path.push_back(Eigen::Vector3d(4.51952,4.19474,0.6));
path.push_back(Eigen::Vector3d(4.60361,4.32779,0.6));
path.push_back(Eigen::Vector3d(4.67889,4.43823,0.6));
path.push_back(Eigen::Vector3d(4.70311,4.52659,0.6));
path.push_back(Eigen::Vector3d(4.74249,4.60527,0.6));
path.push_back(Eigen::Vector3d(4.76599,4.66022,0.6));
path.push_back(Eigen::Vector3d(4.80479,4.70817,0.6));
path.push_back(Eigen::Vector3d(4.81983,4.73854,0.6));
path.push_back(Eigen::Vector3d(4.84387,4.77083,0.6));
path.push_back(Eigen::Vector3d(4.83187,4.74683,0.6));
path.push_back(Eigen::Vector3d(4.80387,4.72283,0.6));
path.push_back(Eigen::Vector3d(4.82309,4.73826,0.6));
path.push_back(Eigen::Vector3d(4.78709,4.71026,0.6));
path.push_back(Eigen::Vector3d(4.79368,4.74021,0.6));
path.push_back(Eigen::Vector3d(4.81094,4.78017,0.6));
path.push_back(Eigen::Vector3d(4.80294,4.77217,0.6));
path.push_back(Eigen::Vector3d(4.76294,4.73217,0.6));
path.push_back(Eigen::Vector3d(4.78635,4.76174,0.6));
path.push_back(Eigen::Vector3d(4.81708,4.77339,0.6));
path.push_back(Eigen::Vector3d(4.79708,4.73339,0.6));
path.push_back(Eigen::Vector3d(4.82967,4.76671,0.6));
path.push_back(Eigen::Vector3d(4.80967,4.73871,0.6));
path.push_back(Eigen::Vector3d(4.84373,4.77097,0.6));
path.push_back(Eigen::Vector3d(4.83573,4.73497,0.6));
path.push_back(Eigen::Vector3d(4.82773,4.69897,0.6));
path.push_back(Eigen::Vector3d(4.85019,4.72718,0.6));



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
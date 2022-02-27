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
path.push_back(Eigen::Vector3d(0.0,5.0,0.6));
path.push_back(Eigen::Vector3d(-0.04,4.71316,0.6));
path.push_back(Eigen::Vector3d(-0.0802285,4.29415,0.6));
path.push_back(Eigen::Vector3d(-0.156453,3.88558,0.6));
path.push_back(Eigen::Vector3d(-0.247833,3.472,0.6));
path.push_back(Eigen::Vector3d(-0.373902,3.1338,0.6));
path.push_back(Eigen::Vector3d(-0.513214,2.84158,0.6));
path.push_back(Eigen::Vector3d(-0.663936,2.56385,0.6));
path.push_back(Eigen::Vector3d(-0.841075,2.28862,0.6));
path.push_back(Eigen::Vector3d(-1.02061,2.01911,0.6));
path.push_back(Eigen::Vector3d(-1.2202,1.75078,0.6));
path.push_back(Eigen::Vector3d(-1.43734,1.54647,0.6));
path.push_back(Eigen::Vector3d(-1.68008,1.31032,0.6));
path.push_back(Eigen::Vector3d(-1.95497,1.04881,0.6));
path.push_back(Eigen::Vector3d(-2.25898,0.769962,0.6));
path.push_back(Eigen::Vector3d(-2.54428,0.495004,0.6));
path.push_back(Eigen::Vector3d(-2.84171,0.200967,0.6));
path.push_back(Eigen::Vector3d(-3.13092,-0.0825744,0.6));
path.push_back(Eigen::Vector3d(-3.43112,-0.360236,0.6));
path.push_back(Eigen::Vector3d(-3.72325,-0.616514,0.6));
path.push_back(Eigen::Vector3d(-3.98722,-0.890164,0.6));
path.push_back(Eigen::Vector3d(-4.25258,-1.11987,0.6));
path.push_back(Eigen::Vector3d(-4.51781,-1.38509,0.6));
path.push_back(Eigen::Vector3d(-4.76292,-1.65209,0.6));
path.push_back(Eigen::Vector3d(-4.99751,-1.92077,0.6));
path.push_back(Eigen::Vector3d(-5.25208,-2.20673,0.6));
path.push_back(Eigen::Vector3d(-5.56972,-2.47532,0.6));
path.push_back(Eigen::Vector3d(-5.90676,-2.72925,0.6));
path.push_back(Eigen::Vector3d(-6.31029,-2.91854,0.6));
path.push_back(Eigen::Vector3d(-6.71469,-3.1114,0.6));
path.push_back(Eigen::Vector3d(-7.09279,-3.28241,0.6));
path.push_back(Eigen::Vector3d(-7.40964,-3.54644,0.6));
path.push_back(Eigen::Vector3d(-7.70087,-3.81452,0.6));
path.push_back(Eigen::Vector3d(-7.96208,-4.1077,0.6));
path.push_back(Eigen::Vector3d(-8.15578,-4.4077,0.6));
path.push_back(Eigen::Vector3d(-8.33186,-4.76037,0.6));
path.push_back(Eigen::Vector3d(-8.46538,-5.12166,0.6));
path.push_back(Eigen::Vector3d(-8.59073,-5.49902,0.6));
path.push_back(Eigen::Vector3d(-8.65794,-5.91898,0.6));
path.push_back(Eigen::Vector3d(-8.69323,-6.34858,0.6));
path.push_back(Eigen::Vector3d(-8.70138,-6.74138,0.6));
path.push_back(Eigen::Vector3d(-8.62694,-7.13961,0.6));
path.push_back(Eigen::Vector3d(-8.67893,-7.386,0.6));
path.push_back(Eigen::Vector3d(-8.69452,-7.56452,0.6));
path.push_back(Eigen::Vector3d(-8.66858,-7.61197,0.6));
path.push_back(Eigen::Vector3d(-8.66407,-7.60741,0.6));
path.push_back(Eigen::Vector3d(-8.65043,-7.60865,0.6));
path.push_back(Eigen::Vector3d(-8.63658,-7.61943,0.6));
path.push_back(Eigen::Vector3d(-8.63472,-7.59731,0.6));
path.push_back(Eigen::Vector3d(-8.62713,-7.59406,0.6));
path.push_back(Eigen::Vector3d(-8.65346,-7.5583,0.6));
path.push_back(Eigen::Vector3d(-8.69042,-7.5225,0.6));
path.push_back(Eigen::Vector3d(-8.71927,-7.48049,0.6));
path.push_back(Eigen::Vector3d(-8.77207,-7.43673,0.6));
path.push_back(Eigen::Vector3d(-8.83966,-7.3895,0.6));
path.push_back(Eigen::Vector3d(-8.8913,-7.33322,0.6));
path.push_back(Eigen::Vector3d(-8.96205,-7.26748,0.6));
path.push_back(Eigen::Vector3d(-9.02854,-7.1928,0.6));
path.push_back(Eigen::Vector3d(-9.0841,-7.16008,0.6));
path.push_back(Eigen::Vector3d(-9.12274,-7.12945,0.6));
path.push_back(Eigen::Vector3d(-9.13048,-7.1016,0.6));
path.push_back(Eigen::Vector3d(-9.19102,-7.07046,0.6));
path.push_back(Eigen::Vector3d(-9.24707,-7.07808,0.6));
path.push_back(Eigen::Vector3d(-9.3227,-7.05738,0.6));
path.push_back(Eigen::Vector3d(-9.38799,-7.0529,0.6));
path.push_back(Eigen::Vector3d(-9.4498,-7.0403,0.6));
path.push_back(Eigen::Vector3d(-9.5249,-7.03401,0.6));
path.push_back(Eigen::Vector3d(-9.62114,-7.04557,0.6));
path.push_back(Eigen::Vector3d(-9.70135,-7.06471,0.6));
path.push_back(Eigen::Vector3d(-9.80599,-7.10122,0.6));
path.push_back(Eigen::Vector3d(-9.96312,-7.16262,0.6));
path.push_back(Eigen::Vector3d(-10.124,-7.26942,0.6));
path.push_back(Eigen::Vector3d(-10.2668,-7.37061,0.6));
path.push_back(Eigen::Vector3d(-10.4007,-7.48675,0.6));
path.push_back(Eigen::Vector3d(-10.4806,-7.62093,0.6));
path.push_back(Eigen::Vector3d(-10.5413,-7.79644,0.6));
path.push_back(Eigen::Vector3d(-10.5619,-7.95357,0.6));
path.push_back(Eigen::Vector3d(-10.4936,-8.13005,0.6));
path.push_back(Eigen::Vector3d(-10.4246,-8.26498,0.6));
path.push_back(Eigen::Vector3d(-10.2995,-8.37798,0.6));
path.push_back(Eigen::Vector3d(-10.1201,-8.51322,0.6));
path.push_back(Eigen::Vector3d(-9.94022,-8.60865,0.6));
path.push_back(Eigen::Vector3d(-9.71833,-8.67752,0.6));
path.push_back(Eigen::Vector3d(-9.48562,-8.73925,0.6));
path.push_back(Eigen::Vector3d(-9.28807,-8.78461,0.6));
path.push_back(Eigen::Vector3d(-9.08739,-8.8517,0.6));
path.push_back(Eigen::Vector3d(-8.85857,-8.93025,0.6));
path.push_back(Eigen::Vector3d(-8.61885,-8.97297,0.6));
path.push_back(Eigen::Vector3d(-8.35784,-8.97409,0.6));
path.push_back(Eigen::Vector3d(-8.06941,-8.96484,0.6));
path.push_back(Eigen::Vector3d(-7.81283,-8.88954,0.6));
path.push_back(Eigen::Vector3d(-7.50893,-8.80764,0.6));
path.push_back(Eigen::Vector3d(-7.21433,-8.67763,0.6));
path.push_back(Eigen::Vector3d(-6.92194,-8.51766,0.6));
path.push_back(Eigen::Vector3d(-6.62945,-8.36194,0.6));
path.push_back(Eigen::Vector3d(-6.3361,-8.23111,0.6));
path.push_back(Eigen::Vector3d(-6.06126,-8.11273,0.6));
path.push_back(Eigen::Vector3d(-5.79775,-7.9937,0.6));
path.push_back(Eigen::Vector3d(-5.52108,-7.87706,0.6));
path.push_back(Eigen::Vector3d(-5.2816,-7.81703,0.6));
path.push_back(Eigen::Vector3d(-5.08277,-7.79985,0.6));
path.push_back(Eigen::Vector3d(-4.91385,-7.80361,0.6));
path.push_back(Eigen::Vector3d(-4.76581,-7.85931,0.6));
path.push_back(Eigen::Vector3d(-4.63115,-7.97785,0.6));
path.push_back(Eigen::Vector3d(-4.52219,-8.12593,0.6));
path.push_back(Eigen::Vector3d(-4.43864,-8.27552,0.6));
path.push_back(Eigen::Vector3d(-4.33654,-8.41216,0.6));
path.push_back(Eigen::Vector3d(-4.2066,-8.53836,0.6));
path.push_back(Eigen::Vector3d(-4.05903,-8.68029,0.6));
path.push_back(Eigen::Vector3d(-3.91833,-8.79911,0.6));
path.push_back(Eigen::Vector3d(-3.73384,-8.85291,0.6));
path.push_back(Eigen::Vector3d(-3.54587,-8.85897,0.6));
path.push_back(Eigen::Vector3d(-3.35064,-8.82682,0.6));
path.push_back(Eigen::Vector3d(-3.21149,-8.78102,0.6));
path.push_back(Eigen::Vector3d(-3.08237,-8.72483,0.6));
path.push_back(Eigen::Vector3d(-2.9597,-8.67746,0.6));
path.push_back(Eigen::Vector3d(-2.86647,-8.60721,0.6));
path.push_back(Eigen::Vector3d(-2.75887,-8.50469,0.6));
path.push_back(Eigen::Vector3d(-2.65598,-8.39646,0.6));
path.push_back(Eigen::Vector3d(-2.56486,-8.28782,0.6));
path.push_back(Eigen::Vector3d(-2.46395,-8.17877,0.6));
path.push_back(Eigen::Vector3d(-2.34983,-8.03154,0.6));
path.push_back(Eigen::Vector3d(-2.2138,-7.88373,0.6));
path.push_back(Eigen::Vector3d(-2.08872,-7.71553,0.6));
path.push_back(Eigen::Vector3d(-1.93014,-7.49733,0.6));
path.push_back(Eigen::Vector3d(-1.74595,-7.29206,0.6));
path.push_back(Eigen::Vector3d(-1.56427,-7.05504,0.6));
path.push_back(Eigen::Vector3d(-1.39469,-6.82998,0.6));
path.push_back(Eigen::Vector3d(-1.22493,-6.60899,0.6));
path.push_back(Eigen::Vector3d(-1.04333,-6.37594,0.6));
path.push_back(Eigen::Vector3d(-0.858311,-6.13867,0.6));
path.push_back(Eigen::Vector3d(-0.690649,-5.92694,0.6));
path.push_back(Eigen::Vector3d(-0.592519,-5.77355,0.6));
path.push_back(Eigen::Vector3d(-0.506015,-5.64284,0.6));
path.push_back(Eigen::Vector3d(-0.432812,-5.54627,0.6));
path.push_back(Eigen::Vector3d(-0.37425,-5.44902,0.6));
path.push_back(Eigen::Vector3d(-0.3154,-5.39521,0.6));
path.push_back(Eigen::Vector3d(-0.27632,-5.33217,0.6));
path.push_back(Eigen::Vector3d(-0.237056,-5.30174,0.6));
path.push_back(Eigen::Vector3d(-0.221645,-5.26139,0.6));
path.push_back(Eigen::Vector3d(-0.213316,-5.22911,0.6));
path.push_back(Eigen::Vector3d(-0.229316,-5.26911,0.6));
path.push_back(Eigen::Vector3d(-0.215453,-5.23129,0.6));
path.push_back(Eigen::Vector3d(-0.235453,-5.25129,0.6));
path.push_back(Eigen::Vector3d(-0.204362,-5.24103,0.6));
path.push_back(Eigen::Vector3d(-0.212362,-5.28103,0.6));
path.push_back(Eigen::Vector3d(-0.18589,-5.25682,0.6));
path.push_back(Eigen::Vector3d(-0.188712,-5.24146,0.6));
path.push_back(Eigen::Vector3d(-0.212712,-5.27746,0.6));
path.push_back(Eigen::Vector3d(-0.210169,-5.22997,0.6));
path.push_back(Eigen::Vector3d(-0.230169,-5.23397,0.6));
path.push_back(Eigen::Vector3d(-0.200136,-5.19917,0.6));
path.push_back(Eigen::Vector3d(-0.240136,-5.20317,0.6));
path.push_back(Eigen::Vector3d(-0.252136,-5.22317,0.6));
path.push_back(Eigen::Vector3d(-0.237708,-5.20254,0.6));
path.push_back(Eigen::Vector3d(-0.253708,-5.22654,0.6));
path.push_back(Eigen::Vector3d(-0.242967,-5.21723,0.6));
path.push_back(Eigen::Vector3d(-0.230373,-5.20579,0.6));
path.push_back(Eigen::Vector3d(-0.234373,-5.21379,0.6));
path.push_back(Eigen::Vector3d(-0.195499,-5.18703,0.6));
path.push_back(Eigen::Vector3d(-0.199499,-5.21103,0.6));



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
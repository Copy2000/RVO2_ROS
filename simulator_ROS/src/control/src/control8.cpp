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
path.push_back(Eigen::Vector3d(0.0,-5.0,0.6));
path.push_back(Eigen::Vector3d(-0.028,-4.73316,0.6));
path.push_back(Eigen::Vector3d(-0.0587711,-4.36605,0.6));
path.push_back(Eigen::Vector3d(-0.109698,-3.99236,0.6));
path.push_back(Eigen::Vector3d(-0.205307,-3.61083,0.6));
path.push_back(Eigen::Vector3d(-0.318289,-3.49292,0.6));
path.push_back(Eigen::Vector3d(-0.408807,-3.46368,0.6));
path.push_back(Eigen::Vector3d(-0.540814,-3.50566,0.6));
path.push_back(Eigen::Vector3d(-0.693552,-3.61065,0.6));
path.push_back(Eigen::Vector3d(-0.874502,-3.745,0.6));
path.push_back(Eigen::Vector3d(-1.0678,-3.99134,0.6));
path.push_back(Eigen::Vector3d(-1.29102,-4.28484,0.6));
path.push_back(Eigen::Vector3d(-1.54481,-4.5333,0.6));
path.push_back(Eigen::Vector3d(-1.8093,-4.86883,0.6));
path.push_back(Eigen::Vector3d(-2.10453,-5.19001,0.6));
path.push_back(Eigen::Vector3d(-2.3764,-5.52663,0.6));
path.push_back(Eigen::Vector3d(-2.62101,-5.86915,0.6));
path.push_back(Eigen::Vector3d(-2.88584,-6.21598,0.6));
path.push_back(Eigen::Vector3d(-3.12692,-6.5685,0.6));
path.push_back(Eigen::Vector3d(-3.36616,-6.91863,0.6));
path.push_back(Eigen::Vector3d(-3.61261,-7.25941,0.6));
path.push_back(Eigen::Vector3d(-3.84875,-7.55686,0.6));
path.push_back(Eigen::Vector3d(-4.09777,-7.85362,0.6));
path.push_back(Eigen::Vector3d(-4.3068,-8.14777,0.6));
path.push_back(Eigen::Vector3d(-4.51436,-8.45772,0.6));
path.push_back(Eigen::Vector3d(-4.72742,-8.76816,0.6));
path.push_back(Eigen::Vector3d(-4.91902,-9.06237,0.6));
path.push_back(Eigen::Vector3d(-5.11046,-9.36391,0.6));
path.push_back(Eigen::Vector3d(-5.26513,-9.65659,0.6));
path.push_back(Eigen::Vector3d(-5.39904,-9.93482,0.6));
path.push_back(Eigen::Vector3d(-5.51614,-10.206,0.6));
path.push_back(Eigen::Vector3d(-5.6121,-10.4592,0.6));
path.push_back(Eigen::Vector3d(-5.65517,-10.6503,0.6));
path.push_back(Eigen::Vector3d(-5.65806,-10.7857,0.6));
path.push_back(Eigen::Vector3d(-5.67329,-10.8907,0.6));
path.push_back(Eigen::Vector3d(-5.67599,-10.9848,0.6));
path.push_back(Eigen::Vector3d(-5.69704,-11.1147,0.6));
path.push_back(Eigen::Vector3d(-5.70343,-11.1746,0.6));
path.push_back(Eigen::Vector3d(-5.73224,-11.1918,0.6));
path.push_back(Eigen::Vector3d(-5.77671,-11.1218,0.6));
path.push_back(Eigen::Vector3d(-5.83948,-10.93,0.6));
path.push_back(Eigen::Vector3d(-5.84263,-10.6735,0.6));
path.push_back(Eigen::Vector3d(-5.73777,-10.5729,0.6));
path.push_back(Eigen::Vector3d(-5.62653,-10.423,0.6));
path.push_back(Eigen::Vector3d(-5.52722,-10.2777,0.6));
path.push_back(Eigen::Vector3d(-5.40954,-10.11,0.6));
path.push_back(Eigen::Vector3d(-5.33184,-9.98367,0.6));
path.push_back(Eigen::Vector3d(-5.27284,-9.88715,0.6));
path.push_back(Eigen::Vector3d(-5.23847,-9.83535,0.6));
path.push_back(Eigen::Vector3d(-5.24668,-9.7777,0.6));
path.push_back(Eigen::Vector3d(-5.26929,-9.74703,0.6));
path.push_back(Eigen::Vector3d(-5.32326,-9.72286,0.6));
path.push_back(Eigen::Vector3d(-5.39902,-9.72267,0.6));
path.push_back(Eigen::Vector3d(-5.47798,-9.72077,0.6));
path.push_back(Eigen::Vector3d(-5.6028,-9.75783,0.6));
path.push_back(Eigen::Vector3d(-5.74378,-9.80044,0.6));
path.push_back(Eigen::Vector3d(-5.90238,-9.84172,0.6));
path.push_back(Eigen::Vector3d(-6.07949,-9.9194,0.6));
path.push_back(Eigen::Vector3d(-6.27216,-10.0057,0.6));
path.push_back(Eigen::Vector3d(-6.44051,-10.0872,0.6));
path.push_back(Eigen::Vector3d(-6.50359,-10.0749,0.6));
path.push_back(Eigen::Vector3d(-6.48829,-10.0295,0.6));
path.push_back(Eigen::Vector3d(-6.48323,-9.95784,0.6));
path.push_back(Eigen::Vector3d(-6.45504,-9.87043,0.6));
path.push_back(Eigen::Vector3d(-6.43646,-9.75591,0.6));
path.push_back(Eigen::Vector3d(-6.43028,-9.6512,0.6));
path.push_back(Eigen::Vector3d(-6.42641,-9.53608,0.6));
path.push_back(Eigen::Vector3d(-6.39817,-9.42565,0.6));
path.push_back(Eigen::Vector3d(-6.40425,-9.30642,0.6));
path.push_back(Eigen::Vector3d(-6.42088,-9.19296,0.6));
path.push_back(Eigen::Vector3d(-6.422,-9.06142,0.6));
path.push_back(Eigen::Vector3d(-6.42009,-8.91709,0.6));
path.push_back(Eigen::Vector3d(-6.37462,-8.75735,0.6));
path.push_back(Eigen::Vector3d(-6.33022,-8.60412,0.6));
path.push_back(Eigen::Vector3d(-6.29402,-8.44528,0.6));
path.push_back(Eigen::Vector3d(-6.23732,-8.30328,0.6));
path.push_back(Eigen::Vector3d(-6.1707,-8.15032,0.6));
path.push_back(Eigen::Vector3d(-6.06333,-7.97554,0.6));
path.push_back(Eigen::Vector3d(-5.98415,-7.81813,0.6));
path.push_back(Eigen::Vector3d(-5.91563,-7.658,0.6));
path.push_back(Eigen::Vector3d(-5.82722,-7.50128,0.6));
path.push_back(Eigen::Vector3d(-5.71403,-7.32069,0.6));
path.push_back(Eigen::Vector3d(-5.61175,-7.15643,0.6));
path.push_back(Eigen::Vector3d(-5.53591,-7.00644,0.6));
path.push_back(Eigen::Vector3d(-5.47264,-6.86567,0.6));
path.push_back(Eigen::Vector3d(-5.40898,-6.722,0.6));
path.push_back(Eigen::Vector3d(-5.33074,-6.53562,0.6));
path.push_back(Eigen::Vector3d(-5.23979,-6.3516,0.6));
path.push_back(Eigen::Vector3d(-5.13711,-6.13979,0.6));
path.push_back(Eigen::Vector3d(-5.04472,-5.91073,0.6));
path.push_back(Eigen::Vector3d(-4.93642,-5.68189,0.6));
path.push_back(Eigen::Vector3d(-4.83218,-5.42904,0.6));
path.push_back(Eigen::Vector3d(-4.73164,-5.16426,0.6));
path.push_back(Eigen::Vector3d(-4.61251,-4.93042,0.6));
path.push_back(Eigen::Vector3d(-4.49494,-4.71023,0.6));
path.push_back(Eigen::Vector3d(-4.38144,-4.49834,0.6));
path.push_back(Eigen::Vector3d(-4.28226,-4.31492,0.6));
path.push_back(Eigen::Vector3d(-4.18092,-4.14621,0.6));
path.push_back(Eigen::Vector3d(-4.04998,-4.04775,0.6));
path.push_back(Eigen::Vector3d(-3.9276,-3.98756,0.6));
path.push_back(Eigen::Vector3d(-3.80553,-3.97443,0.6));
path.push_back(Eigen::Vector3d(-3.66815,-4.00166,0.6));
path.push_back(Eigen::Vector3d(-3.57072,-3.98995,0.6));
path.push_back(Eigen::Vector3d(-3.4782,-3.96617,0.6));
path.push_back(Eigen::Vector3d(-3.38431,-3.92043,0.6));
path.push_back(Eigen::Vector3d(-3.26074,-3.84064,0.6));
path.push_back(Eigen::Vector3d(-3.14826,-3.73061,0.6));
path.push_back(Eigen::Vector3d(-3.02467,-3.62745,0.6));
path.push_back(Eigen::Vector3d(-2.87191,-3.50319,0.6));
path.push_back(Eigen::Vector3d(-2.73155,-3.39265,0.6));
path.push_back(Eigen::Vector3d(-2.56465,-3.27122,0.6));
path.push_back(Eigen::Vector3d(-2.41764,-3.16012,0.6));
path.push_back(Eigen::Vector3d(-2.29024,-2.98564,0.6));
path.push_back(Eigen::Vector3d(-2.16433,-2.80359,0.6));
path.push_back(Eigen::Vector3d(-2.03121,-2.6128,0.6));
path.push_back(Eigen::Vector3d(-1.90006,-2.403,0.6));
path.push_back(Eigen::Vector3d(-1.79112,-2.16559,0.6));
path.push_back(Eigen::Vector3d(-1.66371,-1.91985,0.6));
path.push_back(Eigen::Vector3d(-1.53746,-1.70712,0.6));
path.push_back(Eigen::Vector3d(-1.46874,-1.44387,0.6));
path.push_back(Eigen::Vector3d(-1.43474,-1.17718,0.6));
path.push_back(Eigen::Vector3d(-1.37391,-0.922081,0.6));
path.push_back(Eigen::Vector3d(-1.34116,-0.674969,0.6));
path.push_back(Eigen::Vector3d(-1.30723,-0.403995,0.6));
path.push_back(Eigen::Vector3d(-1.27579,-0.129201,0.6));
path.push_back(Eigen::Vector3d(-1.2305,0.121366,0.6));
path.push_back(Eigen::Vector3d(-1.17227,0.383815,0.6));
path.push_back(Eigen::Vector3d(-1.14157,0.626205,0.6));
path.push_back(Eigen::Vector3d(-1.08898,0.86835,0.6));
path.push_back(Eigen::Vector3d(-1.04769,1.11441,0.6));
path.push_back(Eigen::Vector3d(-0.976775,1.36024,0.6));
path.push_back(Eigen::Vector3d(-0.918202,1.61413,0.6));
path.push_back(Eigen::Vector3d(-0.850871,1.88391,0.6));
path.push_back(Eigen::Vector3d(-0.80704,2.14162,0.6));
path.push_back(Eigen::Vector3d(-0.736727,2.40699,0.6));
path.push_back(Eigen::Vector3d(-0.66594,2.64429,0.6));
path.push_back(Eigen::Vector3d(-0.603533,2.88165,0.6));
path.push_back(Eigen::Vector3d(-0.544537,3.13092,0.6));
path.push_back(Eigen::Vector3d(-0.483828,3.38795,0.6));
path.push_back(Eigen::Vector3d(-0.428788,3.65663,0.6));
path.push_back(Eigen::Vector3d(-0.37103,3.9053,0.6));
path.push_back(Eigen::Vector3d(-0.300824,4.08824,0.6));
path.push_back(Eigen::Vector3d(-0.280659,4.25459,0.6));
path.push_back(Eigen::Vector3d(-0.252527,4.39967,0.6));
path.push_back(Eigen::Vector3d(-0.226022,4.49974,0.6));
path.push_back(Eigen::Vector3d(-0.188818,4.59579,0.6));
path.push_back(Eigen::Vector3d(-0.167054,4.65663,0.6));
path.push_back(Eigen::Vector3d(-0.173643,4.71731,0.6));
path.push_back(Eigen::Vector3d(-0.170915,4.74185,0.6));
path.push_back(Eigen::Vector3d(-0.186915,4.70585,0.6));
path.push_back(Eigen::Vector3d(-0.177532,4.74068,0.6));
path.push_back(Eigen::Vector3d(-0.209532,4.70468,0.6));
path.push_back(Eigen::Vector3d(-0.195625,4.74774,0.6));
path.push_back(Eigen::Vector3d(-0.1605,4.77019,0.6));
path.push_back(Eigen::Vector3d(-0.1885,4.75419,0.6));
path.push_back(Eigen::Vector3d(-0.2125,4.74219,0.6));
path.push_back(Eigen::Vector3d(-0.186,4.76975,0.6));
path.push_back(Eigen::Vector3d(-0.226,4.75375,0.6));
path.push_back(Eigen::Vector3d(-0.1888,4.787,0.6));
path.push_back(Eigen::Vector3d(-0.2168,4.767,0.6));
path.push_back(Eigen::Vector3d(-0.17744,4.8096,0.6));



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
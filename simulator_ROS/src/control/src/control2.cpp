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
    path.push_back(Eigen::Vector3d(-5.0,5.0,0.6));
path.push_back(Eigen::Vector3d(-4.78106,4.74906,0.6));
path.push_back(Eigen::Vector3d(-4.64899,4.48103,0.6));
path.push_back(Eigen::Vector3d(-4.5335,4.22254,0.6));
path.push_back(Eigen::Vector3d(-4.48203,3.97096,0.6));
path.push_back(Eigen::Vector3d(-4.45976,3.75006,0.6));
path.push_back(Eigen::Vector3d(-4.49461,3.53866,0.6));
path.push_back(Eigen::Vector3d(-4.58505,3.34593,0.6));
path.push_back(Eigen::Vector3d(-4.71311,3.11905,0.6));
path.push_back(Eigen::Vector3d(-4.88265,2.92055,0.6));
path.push_back(Eigen::Vector3d(-5.09932,2.74117,0.6));
path.push_back(Eigen::Vector3d(-5.307,2.59061,0.6));
path.push_back(Eigen::Vector3d(-5.50205,2.46016,0.6));
path.push_back(Eigen::Vector3d(-5.70963,2.34352,0.6));
path.push_back(Eigen::Vector3d(-5.92653,2.27269,0.6));
path.push_back(Eigen::Vector3d(-6.14786,2.26737,0.6));
path.push_back(Eigen::Vector3d(-6.28593,2.29549,0.6));
path.push_back(Eigen::Vector3d(-6.33633,2.38774,0.6));
path.push_back(Eigen::Vector3d(-6.2622,2.48862,0.6));
path.push_back(Eigen::Vector3d(-6.12676,2.58147,0.6));
path.push_back(Eigen::Vector3d(-5.96902,2.67001,0.6));
path.push_back(Eigen::Vector3d(-5.70416,2.65868,0.6));
path.push_back(Eigen::Vector3d(-5.42083,2.59668,0.6));
path.push_back(Eigen::Vector3d(-5.15615,2.43516,0.6));
path.push_back(Eigen::Vector3d(-4.90226,2.20772,0.6));
path.push_back(Eigen::Vector3d(-4.64182,2.09316,0.6));
path.push_back(Eigen::Vector3d(-4.38201,2.0082,0.6));
path.push_back(Eigen::Vector3d(-4.10568,1.95645,0.6));
path.push_back(Eigen::Vector3d(-3.87343,1.92732,0.6));
path.push_back(Eigen::Vector3d(-3.60262,1.88245,0.6));
path.push_back(Eigen::Vector3d(-3.32367,1.77161,0.6));
path.push_back(Eigen::Vector3d(-3.05121,1.5664,0.6));
path.push_back(Eigen::Vector3d(-2.80518,1.3207,0.6));
path.push_back(Eigen::Vector3d(-2.72143,1.28627,0.6));
path.push_back(Eigen::Vector3d(-2.61545,1.28074,0.6));
path.push_back(Eigen::Vector3d(-2.51149,1.26559,0.6));
path.push_back(Eigen::Vector3d(-2.39958,1.28238,0.6));
path.push_back(Eigen::Vector3d(-2.25355,1.32513,0.6));
path.push_back(Eigen::Vector3d(-2.05751,1.37136,0.6));
path.push_back(Eigen::Vector3d(-1.85774,1.40486,0.6));
path.push_back(Eigen::Vector3d(-1.64476,1.42007,0.6));
path.push_back(Eigen::Vector3d(-1.44905,1.41586,0.6));
path.push_back(Eigen::Vector3d(-1.20771,1.36318,0.6));
path.push_back(Eigen::Vector3d(-0.966819,1.28985,0.6));
path.push_back(Eigen::Vector3d(-0.716097,1.16378,0.6));
path.push_back(Eigen::Vector3d(-0.477534,1.0028,0.6));
path.push_back(Eigen::Vector3d(-0.267429,0.814658,0.6));
path.push_back(Eigen::Vector3d(-0.0587985,0.57887,0.6));
path.push_back(Eigen::Vector3d(0.117107,0.332769,0.6));
path.push_back(Eigen::Vector3d(0.326501,0.0553451,0.6));
path.push_back(Eigen::Vector3d(0.521366,-0.209402,0.6));
path.push_back(Eigen::Vector3d(0.717949,-0.46535,0.6));
path.push_back(Eigen::Vector3d(0.932071,-0.716575,0.6));
path.push_back(Eigen::Vector3d(1.13907,-0.991386,0.6));
path.push_back(Eigen::Vector3d(1.35223,-1.25316,0.6));
path.push_back(Eigen::Vector3d(1.55505,-1.52214,0.6));
path.push_back(Eigen::Vector3d(1.77252,-1.7858,0.6));
path.push_back(Eigen::Vector3d(2.0001,-2.02843,0.6));
path.push_back(Eigen::Vector3d(2.2085,-2.27463,0.6));
path.push_back(Eigen::Vector3d(2.41113,-2.53971,0.6));
path.push_back(Eigen::Vector3d(2.63791,-2.78262,0.6));
path.push_back(Eigen::Vector3d(2.87457,-3.04055,0.6));
path.push_back(Eigen::Vector3d(3.10201,-3.28498,0.6));
path.push_back(Eigen::Vector3d(3.33652,-3.53572,0.6));
path.push_back(Eigen::Vector3d(3.55503,-3.78823,0.6));
path.push_back(Eigen::Vector3d(3.77288,-4.03246,0.6));
path.push_back(Eigen::Vector3d(3.9983,-4.26197,0.6));
path.push_back(Eigen::Vector3d(4.17464,-4.42958,0.6));
path.push_back(Eigen::Vector3d(4.32371,-4.55966,0.6));
path.push_back(Eigen::Vector3d(4.45097,-4.66773,0.6));
path.push_back(Eigen::Vector3d(4.52478,-4.74218,0.6));
path.push_back(Eigen::Vector3d(4.61582,-4.79775,0.6));
path.push_back(Eigen::Vector3d(4.67666,-4.8742,0.6));
path.push_back(Eigen::Vector3d(4.72933,-4.93136,0.6));
path.push_back(Eigen::Vector3d(4.70933,-4.93936,0.6));
path.push_back(Eigen::Vector3d(4.69333,-4.96736,0.6));
path.push_back(Eigen::Vector3d(4.68533,-5.00736,0.6));
path.push_back(Eigen::Vector3d(4.68133,-5.04736,0.6));
path.push_back(Eigen::Vector3d(4.72906,-5.05789,0.6));
path.push_back(Eigen::Vector3d(4.70506,-5.08189,0.6));
path.push_back(Eigen::Vector3d(4.68506,-5.10189,0.6));
path.push_back(Eigen::Vector3d(4.73605,-5.08551,0.6));
path.push_back(Eigen::Vector3d(4.70405,-5.09751,0.6));
path.push_back(Eigen::Vector3d(4.66805,-5.13751,0.6));
path.push_back(Eigen::Vector3d(4.71844,-5.11801,0.6));
path.push_back(Eigen::Vector3d(4.69044,-5.15001,0.6));
path.push_back(Eigen::Vector3d(4.72835,-5.12801,0.6));
path.push_back(Eigen::Vector3d(4.70435,-5.16401,0.6));
path.push_back(Eigen::Vector3d(4.74348,-5.1632,0.6));
path.push_back(Eigen::Vector3d(4.73148,-5.1872,0.6));
path.push_back(Eigen::Vector3d(4.77318,-5.18576,0.6));
path.push_back(Eigen::Vector3d(4.75318,-5.20976,0.6));
path.push_back(Eigen::Vector3d(4.79855,-5.20381,0.6));
path.push_back(Eigen::Vector3d(4.77055,-5.21181,0.6));
path.push_back(Eigen::Vector3d(4.75055,-5.25181,0.6));
path.push_back(Eigen::Vector3d(4.78444,-5.21745,0.6));
path.push_back(Eigen::Vector3d(4.74844,-5.23345,0.6));
path.push_back(Eigen::Vector3d(4.75875,-5.19876,0.6));
path.push_back(Eigen::Vector3d(4.72675,-5.21476,0.6));
path.push_back(Eigen::Vector3d(4.7494,-5.18781,0.6));
path.push_back(Eigen::Vector3d(4.7254,-5.19181,0.6));
path.push_back(Eigen::Vector3d(4.76432,-5.17345,0.6));
path.push_back(Eigen::Vector3d(4.72432,-5.18145,0.6));
path.push_back(Eigen::Vector3d(4.74346,-5.18516,0.6));
path.push_back(Eigen::Vector3d(4.76276,-5.17212,0.6));
path.push_back(Eigen::Vector3d(4.74276,-5.19613,0.6));
path.push_back(Eigen::Vector3d(4.75821,-5.1969,0.6));
path.push_back(Eigen::Vector3d(4.73021,-5.2369,0.6));
path.push_back(Eigen::Vector3d(4.75617,-5.20952,0.6));
path.push_back(Eigen::Vector3d(4.78894,-5.17962,0.6));
path.push_back(Eigen::Vector3d(4.77694,-5.20762,0.6));
path.push_back(Eigen::Vector3d(4.77294,-5.23562,0.6));
path.push_back(Eigen::Vector3d(4.79835,-5.20049,0.6));
path.push_back(Eigen::Vector3d(4.77435,-5.22049,0.6));
path.push_back(Eigen::Vector3d(4.77035,-5.22849,0.6));
path.push_back(Eigen::Vector3d(4.77628,-5.18679,0.6));
path.push_back(Eigen::Vector3d(4.74028,-5.20279,0.6));
path.push_back(Eigen::Vector3d(4.75222,-5.19423,0.6));
path.push_back(Eigen::Vector3d(4.73222,-5.22623,0.6));
path.push_back(Eigen::Vector3d(4.76178,-5.19299,0.6));
path.push_back(Eigen::Vector3d(4.72178,-5.22499,0.6));
path.push_back(Eigen::Vector3d(4.75742,-5.21199,0.6));
path.push_back(Eigen::Vector3d(4.79794,-5.20159,0.6));
path.push_back(Eigen::Vector3d(4.78194,-5.22159,0.6));
path.push_back(Eigen::Vector3d(4.74594,-5.25759,0.6));
path.push_back(Eigen::Vector3d(4.78075,-5.22607,0.6));
path.push_back(Eigen::Vector3d(4.76475,-5.23007,0.6));
path.push_back(Eigen::Vector3d(4.7838,-5.19606,0.6));
path.push_back(Eigen::Vector3d(4.7438,-5.23206,0.6));
path.push_back(Eigen::Vector3d(4.75504,-5.19765,0.6));
path.push_back(Eigen::Vector3d(4.72704,-5.22965,0.6));
path.push_back(Eigen::Vector3d(4.77763,-5.19172,0.6));
path.push_back(Eigen::Vector3d(4.76963,-5.23172,0.6));
path.push_back(Eigen::Vector3d(4.82077,-5.23247,0.6));
path.push_back(Eigen::Vector3d(4.80077,-5.26447,0.6));
path.push_back(Eigen::Vector3d(4.81786,-5.23274,0.6));
path.push_back(Eigen::Vector3d(4.78586,-5.26874,0.6));
path.push_back(Eigen::Vector3d(4.82469,-5.25099,0.6));
path.push_back(Eigen::Vector3d(4.80069,-5.26299,0.6));
path.push_back(Eigen::Vector3d(4.82455,-5.21439,0.6));
path.push_back(Eigen::Vector3d(4.80455,-5.23039,0.6));
path.push_back(Eigen::Vector3d(4.78055,-5.27039,0.6));
path.push_back(Eigen::Vector3d(4.82044,-5.25631,0.6));
path.push_back(Eigen::Vector3d(4.81244,-5.29631,0.6));
path.push_back(Eigen::Vector3d(4.80995,-5.26105,0.6));
path.push_back(Eigen::Vector3d(4.83996,-5.22484,0.6));
path.push_back(Eigen::Vector3d(4.81196,-5.22884,0.6));
path.push_back(Eigen::Vector3d(4.78396,-5.25284,0.6));
path.push_back(Eigen::Vector3d(4.81517,-5.21027,0.6));
path.push_back(Eigen::Vector3d(4.81117,-5.24627,0.6));
path.push_back(Eigen::Vector3d(4.77917,-5.26627,0.6));
path.push_back(Eigen::Vector3d(4.80334,-5.24102,0.6));
path.push_back(Eigen::Vector3d(4.79534,-5.26502,0.6));
path.push_back(Eigen::Vector3d(4.83227,-5.23201,0.6));
path.push_back(Eigen::Vector3d(4.82027,-5.23601,0.6));
path.push_back(Eigen::Vector3d(4.78827,-5.24801,0.6));
path.push_back(Eigen::Vector3d(4.82262,-5.23041,0.6));
path.push_back(Eigen::Vector3d(4.79462,-5.26641,0.6));
path.push_back(Eigen::Vector3d(4.83169,-5.23713,0.6));
path.push_back(Eigen::Vector3d(4.79169,-5.26913,0.6));
path.push_back(Eigen::Vector3d(4.80135,-5.2393,0.6));



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
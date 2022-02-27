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
    path.push_back(Eigen::Vector3d(5.0,-5.0,0.6));
path.push_back(Eigen::Vector3d(4.74106,-4.78506,0.6));
path.push_back(Eigen::Vector3d(4.47844,-4.59826,0.6));
path.push_back(Eigen::Vector3d(4.2196,-4.45014,0.6));
path.push_back(Eigen::Vector3d(3.97408,-4.36045,0.6));
path.push_back(Eigen::Vector3d(3.75506,-4.30635,0.6));
path.push_back(Eigen::Vector3d(3.54736,-4.2985,0.6));
path.push_back(Eigen::Vector3d(3.36237,-4.35419,0.6));
path.push_back(Eigen::Vector3d(3.15701,-4.52008,0.6));
path.push_back(Eigen::Vector3d(3.00972,-4.67819,0.6));
path.push_back(Eigen::Vector3d(2.87304,-4.85719,0.6));
path.push_back(Eigen::Vector3d(2.78457,-5.06203,0.6));
path.push_back(Eigen::Vector3d(2.72094,-5.27208,0.6));
path.push_back(Eigen::Vector3d(2.69394,-5.47077,0.6));
path.push_back(Eigen::Vector3d(2.692,-5.63,0.6));
path.push_back(Eigen::Vector3d(2.73989,-5.70815,0.6));
path.push_back(Eigen::Vector3d(2.79237,-5.64946,0.6));
path.push_back(Eigen::Vector3d(2.80904,-5.53707,0.6));
path.push_back(Eigen::Vector3d(2.79934,-5.36498,0.6));
path.push_back(Eigen::Vector3d(2.78214,-5.1715,0.6));
path.push_back(Eigen::Vector3d(2.69538,-4.9291,0.6));
path.push_back(Eigen::Vector3d(2.5359,-4.7056,0.6));
path.push_back(Eigen::Vector3d(2.34716,-4.49273,0.6));
path.push_back(Eigen::Vector3d(2.13512,-4.26461,0.6));
path.push_back(Eigen::Vector3d(1.97722,-4.03768,0.6));
path.push_back(Eigen::Vector3d(1.80512,-3.80272,0.6));
path.push_back(Eigen::Vector3d(1.66548,-3.56276,0.6));
path.push_back(Eigen::Vector3d(1.56,-3.35436,0.6));
path.push_back(Eigen::Vector3d(1.43605,-3.12654,0.6));
path.push_back(Eigen::Vector3d(1.25588,-2.87887,0.6));
path.push_back(Eigen::Vector3d(1.05605,-2.66824,0.6));
path.push_back(Eigen::Vector3d(0.849027,-2.46129,0.6));
path.push_back(Eigen::Vector3d(0.619102,-2.21791,0.6));
path.push_back(Eigen::Vector3d(0.583538,-2.14548,0.6));
path.push_back(Eigen::Vector3d(0.549101,-2.09366,0.6));
path.push_back(Eigen::Vector3d(0.493304,-2.06622,0.6));
path.push_back(Eigen::Vector3d(0.445569,-2.06544,0.6));
path.push_back(Eigen::Vector3d(0.354525,-2.08268,0.6));
path.push_back(Eigen::Vector3d(0.207097,-2.12494,0.6));
path.push_back(Eigen::Vector3d(0.0345714,-2.18568,0.6));
path.push_back(Eigen::Vector3d(-0.123744,-2.27956,0.6));
path.push_back(Eigen::Vector3d(-0.2027,-2.39854,0.6));
path.push_back(Eigen::Vector3d(-0.338566,-2.52791,0.6));
path.push_back(Eigen::Vector3d(-0.528725,-2.68047,0.6));
path.push_back(Eigen::Vector3d(-0.713258,-2.83622,0.6));
path.push_back(Eigen::Vector3d(-0.934538,-2.96042,0.6));
path.push_back(Eigen::Vector3d(-1.12264,-3.09186,0.6));
path.push_back(Eigen::Vector3d(-1.2602,-3.23858,0.6));
path.push_back(Eigen::Vector3d(-1.35818,-3.39242,0.6));
path.push_back(Eigen::Vector3d(-1.43018,-3.57242,0.6));
path.push_back(Eigen::Vector3d(-1.4349,-3.77523,0.6));
path.push_back(Eigen::Vector3d(-1.36354,-4.02288,0.6));
path.push_back(Eigen::Vector3d(-1.25839,-4.25394,0.6));
path.push_back(Eigen::Vector3d(-1.16692,-4.43116,0.6));
path.push_back(Eigen::Vector3d(-1.1416,-4.55369,0.6));
path.push_back(Eigen::Vector3d(-1.16373,-4.61354,0.6));
path.push_back(Eigen::Vector3d(-1.23287,-4.63178,0.6));
path.push_back(Eigen::Vector3d(-1.31651,-4.53875,0.6));
path.push_back(Eigen::Vector3d(-1.44076,-4.28811,0.6));
path.push_back(Eigen::Vector3d(-1.52749,-4.01335,0.6));
path.push_back(Eigen::Vector3d(-1.64463,-3.75245,0.6));
path.push_back(Eigen::Vector3d(-1.78929,-3.49546,0.6));
path.push_back(Eigen::Vector3d(-1.91656,-3.24221,0.6));
path.push_back(Eigen::Vector3d(-2.05885,-2.98877,0.6));
path.push_back(Eigen::Vector3d(-2.18362,-2.75105,0.6));
path.push_back(Eigen::Vector3d(-2.31916,-2.50111,0.6));
path.push_back(Eigen::Vector3d(-2.45717,-2.25889,0.6));
path.push_back(Eigen::Vector3d(-2.58534,-2.00834,0.6));
path.push_back(Eigen::Vector3d(-2.70002,-1.75754,0.6));
path.push_back(Eigen::Vector3d(-2.81762,-1.50254,0.6));
path.push_back(Eigen::Vector3d(-2.93798,-1.24335,0.6));
path.push_back(Eigen::Vector3d(-3.06095,-0.991914,0.6));
path.push_back(Eigen::Vector3d(-3.18617,-0.736198,0.6));
path.push_back(Eigen::Vector3d(-3.30146,-0.47217,0.6));
path.push_back(Eigen::Vector3d(-3.42721,-0.223904,0.6));
path.push_back(Eigen::Vector3d(-3.5505,0.0127393,0.6));
path.push_back(Eigen::Vector3d(-3.65902,0.253792,0.6));
path.push_back(Eigen::Vector3d(-3.74139,0.499153,0.6));
path.push_back(Eigen::Vector3d(-3.85098,0.736624,0.6));
path.push_back(Eigen::Vector3d(-3.92986,0.982466,0.6));
path.push_back(Eigen::Vector3d(-4.0279,1.25642,0.6));
path.push_back(Eigen::Vector3d(-4.12414,1.49861,0.6));
path.push_back(Eigen::Vector3d(-4.23381,1.75713,0.6));
path.push_back(Eigen::Vector3d(-4.31172,2.01211,0.6));
path.push_back(Eigen::Vector3d(-4.38003,2.27127,0.6));
path.push_back(Eigen::Vector3d(-4.48348,2.53053,0.6));
path.push_back(Eigen::Vector3d(-4.546,2.77033,0.6));
path.push_back(Eigen::Vector3d(-4.611,3.03829,0.6));
path.push_back(Eigen::Vector3d(-4.68255,3.28639,0.6));
path.push_back(Eigen::Vector3d(-4.7505,3.52684,0.6));
path.push_back(Eigen::Vector3d(-4.81806,3.79967,0.6));
path.push_back(Eigen::Vector3d(-4.85845,4.01174,0.6));
path.push_back(Eigen::Vector3d(-4.91876,4.18939,0.6));
path.push_back(Eigen::Vector3d(-4.96301,4.31551,0.6));
path.push_back(Eigen::Vector3d(-4.99841,4.44441,0.6));
path.push_back(Eigen::Vector3d(-5.01073,4.52353,0.6));
path.push_back(Eigen::Vector3d(-5.01258,4.59482,0.6));
path.push_back(Eigen::Vector3d(-5.03006,4.66386,0.6));
path.push_back(Eigen::Vector3d(-5.06405,4.71509,0.6));
path.push_back(Eigen::Vector3d(-5.08005,4.67509,0.6));
path.push_back(Eigen::Vector3d(-5.07204,4.72007,0.6));
path.push_back(Eigen::Vector3d(-5.09604,4.71607,0.6));
path.push_back(Eigen::Vector3d(-5.12804,4.68007,0.6));
path.push_back(Eigen::Vector3d(-5.13043,4.72806,0.6));
path.push_back(Eigen::Vector3d(-5.15843,4.70406,0.6));
path.push_back(Eigen::Vector3d(-5.13075,4.75124,0.6));
path.push_back(Eigen::Vector3d(-5.15875,4.74724,0.6));
path.push_back(Eigen::Vector3d(-5.18275,4.72724,0.6));
path.push_back(Eigen::Vector3d(-5.1862,4.7738,0.6));
path.push_back(Eigen::Vector3d(-5.2262,4.7378,0.6));
path.push_back(Eigen::Vector3d(-5.20896,4.77424,0.6));
path.push_back(Eigen::Vector3d(-5.21696,4.75824,0.6));
path.push_back(Eigen::Vector3d(-5.20557,4.80259,0.6));
path.push_back(Eigen::Vector3d(-5.20957,4.77059,0.6));
path.push_back(Eigen::Vector3d(-5.23357,4.75859,0.6));
path.push_back(Eigen::Vector3d(-5.19485,4.77087,0.6));
path.push_back(Eigen::Vector3d(-5.19885,4.76287,0.6));
path.push_back(Eigen::Vector3d(-5.20285,4.74287,0.6));
path.push_back(Eigen::Vector3d(-5.17028,4.7543,0.6));
path.push_back(Eigen::Vector3d(-5.21028,4.7143,0.6));
path.push_back(Eigen::Vector3d(-5.20023,4.73944,0.6));
path.push_back(Eigen::Vector3d(-5.18018,4.77955,0.6));
path.push_back(Eigen::Vector3d(-5.18818,4.74755,0.6));
path.push_back(Eigen::Vector3d(-5.21618,4.74355,0.6));
path.push_back(Eigen::Vector3d(-5.19694,4.76684,0.6));
path.push_back(Eigen::Vector3d(-5.20494,4.75484,0.6));
path.push_back(Eigen::Vector3d(-5.17996,4.79187,0.6));
path.push_back(Eigen::Vector3d(-5.19596,4.77187,0.6));
path.push_back(Eigen::Vector3d(-5.22796,4.76387,0.6));
path.push_back(Eigen::Vector3d(-5.21836,4.7711,0.6));
path.push_back(Eigen::Vector3d(-5.19469,4.78488,0.6));
path.push_back(Eigen::Vector3d(-5.20269,4.78088,0.6));
path.push_back(Eigen::Vector3d(-5.23869,4.77688,0.6));
path.push_back(Eigen::Vector3d(-5.21495,4.8175,0.6));
path.push_back(Eigen::Vector3d(-5.23495,4.7775,0.6));
path.push_back(Eigen::Vector3d(-5.21196,4.806,0.6));
path.push_back(Eigen::Vector3d(-5.21596,4.786,0.6));
path.push_back(Eigen::Vector3d(-5.21996,4.75,0.6));
path.push_back(Eigen::Vector3d(-5.19997,4.788,0.6));
path.push_back(Eigen::Vector3d(-5.22397,4.772,0.6));
path.push_back(Eigen::Vector3d(-5.19918,4.7776,0.6));
path.push_back(Eigen::Vector3d(-5.23518,4.7416,0.6));
path.push_back(Eigen::Vector3d(-5.22014,4.76928,0.6));
path.push_back(Eigen::Vector3d(-5.18011,4.79943,0.6));
path.push_back(Eigen::Vector3d(-5.21611,4.77943,0.6));
path.push_back(Eigen::Vector3d(-5.22411,4.74743,0.6));
path.push_back(Eigen::Vector3d(-5.19129,4.78994,0.6));
path.push_back(Eigen::Vector3d(-5.20729,4.77394,0.6));
path.push_back(Eigen::Vector3d(-5.24329,4.75794,0.6));
path.push_back(Eigen::Vector3d(-5.20663,4.79435,0.6));
path.push_back(Eigen::Vector3d(-5.23063,4.77435,0.6));
path.push_back(Eigen::Vector3d(-5.20051,4.78748,0.6));
path.push_back(Eigen::Vector3d(-5.22451,4.76348,0.6));
path.push_back(Eigen::Vector3d(-5.2076,4.80279,0.6));
path.push_back(Eigen::Vector3d(-5.2116,4.79879,0.6));
path.push_back(Eigen::Vector3d(-5.2356,4.76679,0.6));
path.push_back(Eigen::Vector3d(-5.20848,4.78543,0.6));
path.push_back(Eigen::Vector3d(-5.22848,4.77743,0.6));
path.push_back(Eigen::Vector3d(-5.20279,4.78594,0.6));
path.push_back(Eigen::Vector3d(-5.21079,4.75394,0.6));
path.push_back(Eigen::Vector3d(-5.18063,4.77915,0.6));


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
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
    path.push_back(Eigen::Vector3d(-5.0,0.0,0.6));
path.push_back(Eigen::Vector3d(-4.604,-0.036,0.6));
path.push_back(Eigen::Vector3d(-4.23715,-0.106374,0.6));
path.push_back(Eigen::Vector3d(-3.84421,-0.18368,0.6));
path.push_back(Eigen::Vector3d(-3.48219,-0.292219,0.6));
path.push_back(Eigen::Vector3d(-3.35791,-0.399866,0.6));
path.push_back(Eigen::Vector3d(-3.35407,-0.490161,0.6));
path.push_back(Eigen::Vector3d(-3.41447,-0.624707,0.6));
path.push_back(Eigen::Vector3d(-3.56336,-0.744443,0.6));
path.push_back(Eigen::Vector3d(-3.7361,-0.899659,0.6));
path.push_back(Eigen::Vector3d(-3.94849,-1.08152,0.6));
path.push_back(Eigen::Vector3d(-4.24031,-1.27627,0.6));
path.push_back(Eigen::Vector3d(-4.51069,-1.50804,0.6));
path.push_back(Eigen::Vector3d(-4.78669,-1.76651,0.6));
path.push_back(Eigen::Vector3d(-5.11921,-2.0261,0.6));
path.push_back(Eigen::Vector3d(-5.43958,-2.3273,0.6));
path.push_back(Eigen::Vector3d(-5.76387,-2.6446,0.6));
path.push_back(Eigen::Vector3d(-6.08312,-2.93096,0.6));
path.push_back(Eigen::Vector3d(-6.41311,-3.2422,0.6));
path.push_back(Eigen::Vector3d(-6.71563,-3.53731,0.6));
path.push_back(Eigen::Vector3d(-6.93271,-3.90604,0.6));
path.push_back(Eigen::Vector3d(-7.12206,-4.29503,0.6));
path.push_back(Eigen::Vector3d(-7.26834,-4.67594,0.6));
path.push_back(Eigen::Vector3d(-7.40593,-5.057,0.6));
path.push_back(Eigen::Vector3d(-7.56421,-5.4119,0.6));
path.push_back(Eigen::Vector3d(-7.7124,-5.76171,0.6));
path.push_back(Eigen::Vector3d(-7.87571,-6.08934,0.6));
path.push_back(Eigen::Vector3d(-8.03022,-6.44056,0.6));
path.push_back(Eigen::Vector3d(-8.19046,-6.79634,0.6));
path.push_back(Eigen::Vector3d(-8.30794,-7.11429,0.6));
path.push_back(Eigen::Vector3d(-8.41175,-7.37439,0.6));
path.push_back(Eigen::Vector3d(-8.48594,-7.6,0.6));
path.push_back(Eigen::Vector3d(-8.59157,-7.87535,0.6));
path.push_back(Eigen::Vector3d(-8.7086,-8.14889,0.6));
path.push_back(Eigen::Vector3d(-8.85424,-8.46817,0.6));
path.push_back(Eigen::Vector3d(-9.00363,-8.79316,0.6));
path.push_back(Eigen::Vector3d(-9.1705,-9.13521,0.6));
path.push_back(Eigen::Vector3d(-9.33483,-9.50314,0.6));
path.push_back(Eigen::Vector3d(-9.52358,-9.8719,0.6));
path.push_back(Eigen::Vector3d(-9.67928,-10.2652,0.6));
path.push_back(Eigen::Vector3d(-9.86566,-10.6518,0.6));
path.push_back(Eigen::Vector3d(-10.0121,-11.0418,0.6));
path.push_back(Eigen::Vector3d(-10.123,-11.4092,0.6));
path.push_back(Eigen::Vector3d(-10.0879,-11.7701,0.6));
path.push_back(Eigen::Vector3d(-9.94563,-12.0404,0.6));
path.push_back(Eigen::Vector3d(-9.73713,-12.1707,0.6));
path.push_back(Eigen::Vector3d(-9.49936,-12.1978,0.6));
path.push_back(Eigen::Vector3d(-9.27169,-12.168,0.6));
path.push_back(Eigen::Vector3d(-9.06453,-12.139,0.6));
path.push_back(Eigen::Vector3d(-8.94654,-12.0869,0.6));
path.push_back(Eigen::Vector3d(-8.86842,-12.0433,0.6));
path.push_back(Eigen::Vector3d(-8.87869,-11.9744,0.6));
path.push_back(Eigen::Vector3d(-8.93858,-11.8897,0.6));
path.push_back(Eigen::Vector3d(-9.06438,-11.7894,0.6));
path.push_back(Eigen::Vector3d(-9.21624,-11.6983,0.6));
path.push_back(Eigen::Vector3d(-9.41009,-11.6242,0.6));
path.push_back(Eigen::Vector3d(-9.61559,-11.5188,0.6));
path.push_back(Eigen::Vector3d(-9.84974,-11.4254,0.6));
path.push_back(Eigen::Vector3d(-10.0919,-11.2865,0.6));
path.push_back(Eigen::Vector3d(-10.3547,-11.1775,0.6));
path.push_back(Eigen::Vector3d(-10.621,-11.0586,0.6));
path.push_back(Eigen::Vector3d(-10.7806,-10.9896,0.6));
path.push_back(Eigen::Vector3d(-10.8161,-10.9912,0.6));
path.push_back(Eigen::Vector3d(-10.7968,-11.0319,0.6));
path.push_back(Eigen::Vector3d(-10.7393,-11.0803,0.6));
path.push_back(Eigen::Vector3d(-10.6281,-11.1106,0.6));
path.push_back(Eigen::Vector3d(-10.541,-11.1441,0.6));
path.push_back(Eigen::Vector3d(-10.4318,-11.1634,0.6));
path.push_back(Eigen::Vector3d(-10.3252,-11.2009,0.6));
path.push_back(Eigen::Vector3d(-10.205,-11.2332,0.6));
path.push_back(Eigen::Vector3d(-10.1164,-11.2869,0.6));
path.push_back(Eigen::Vector3d(-10.0214,-11.325,0.6));
path.push_back(Eigen::Vector3d(-9.88185,-11.3655,0.6));
path.push_back(Eigen::Vector3d(-9.67361,-11.4056,0.6));
path.push_back(Eigen::Vector3d(-9.39926,-11.4709,0.6));
path.push_back(Eigen::Vector3d(-9.08693,-11.507,0.6));
path.push_back(Eigen::Vector3d(-8.77903,-11.5464,0.6));
path.push_back(Eigen::Vector3d(-8.45855,-11.537,0.6));
path.push_back(Eigen::Vector3d(-8.14185,-11.559,0.6));
path.push_back(Eigen::Vector3d(-7.80414,-11.5449,0.6));
path.push_back(Eigen::Vector3d(-7.47196,-11.5117,0.6));
path.push_back(Eigen::Vector3d(-7.15426,-11.4973,0.6));
path.push_back(Eigen::Vector3d(-6.83804,-11.4455,0.6));
path.push_back(Eigen::Vector3d(-6.52319,-11.3893,0.6));
path.push_back(Eigen::Vector3d(-6.21299,-11.3467,0.6));
path.push_back(Eigen::Vector3d(-5.88966,-11.2801,0.6));
path.push_back(Eigen::Vector3d(-5.57102,-11.1994,0.6));
path.push_back(Eigen::Vector3d(-5.25323,-11.1448,0.6));
path.push_back(Eigen::Vector3d(-4.94662,-11.0774,0.6));
path.push_back(Eigen::Vector3d(-4.65843,-10.9933,0.6));
path.push_back(Eigen::Vector3d(-4.34626,-10.9313,0.6));
path.push_back(Eigen::Vector3d(-4.06606,-10.8055,0.6));
path.push_back(Eigen::Vector3d(-3.78531,-10.6821,0.6));
path.push_back(Eigen::Vector3d(-3.48398,-10.5703,0.6));
path.push_back(Eigen::Vector3d(-3.19123,-10.4485,0.6));
path.push_back(Eigen::Vector3d(-2.92338,-10.3133,0.6));
path.push_back(Eigen::Vector3d(-2.63795,-10.1735,0.6));
path.push_back(Eigen::Vector3d(-2.34697,-10.0477,0.6));
path.push_back(Eigen::Vector3d(-2.0701,-9.91449,0.6));
path.push_back(Eigen::Vector3d(-1.79022,-9.7775,0.6));
path.push_back(Eigen::Vector3d(-1.53236,-9.647,0.6));
path.push_back(Eigen::Vector3d(-1.2761,-9.50915,0.6));
path.push_back(Eigen::Vector3d(-1.04123,-9.32113,0.6));
path.push_back(Eigen::Vector3d(-0.838041,-9.15312,0.6));
path.push_back(Eigen::Vector3d(-0.639775,-8.96151,0.6));
path.push_back(Eigen::Vector3d(-0.468679,-8.75684,0.6));
path.push_back(Eigen::Vector3d(-0.323096,-8.5221,0.6));
path.push_back(Eigen::Vector3d(-0.189487,-8.30337,0.6));
path.push_back(Eigen::Vector3d(-0.0557998,-8.08467,0.6));
path.push_back(Eigen::Vector3d(0.101968,-7.86599,0.6));
path.push_back(Eigen::Vector3d(0.235157,-7.63513,0.6));
path.push_back(Eigen::Vector3d(0.384645,-7.41236,0.6));
path.push_back(Eigen::Vector3d(0.460564,-7.18083,0.6));
path.push_back(Eigen::Vector3d(0.521639,-6.94459,0.6));
path.push_back(Eigen::Vector3d(0.547676,-6.71181,0.6));
path.push_back(Eigen::Vector3d(0.554941,-6.46335,0.6));
path.push_back(Eigen::Vector3d(0.546751,-6.20612,0.6));
path.push_back(Eigen::Vector3d(0.536716,-5.9267,0.6));
path.push_back(Eigen::Vector3d(0.554182,-5.66044,0.6));
path.push_back(Eigen::Vector3d(0.553333,-5.42871,0.6));
path.push_back(Eigen::Vector3d(0.566531,-5.14248,0.6));
path.push_back(Eigen::Vector3d(0.567724,-4.87555,0.6));
path.push_back(Eigen::Vector3d(0.638832,-4.61376,0.6));
path.push_back(Eigen::Vector3d(0.744072,-4.34787,0.6));
path.push_back(Eigen::Vector3d(0.833983,-4.0873,0.6));
path.push_back(Eigen::Vector3d(0.940315,-3.80203,0.6));
path.push_back(Eigen::Vector3d(1.05471,-3.52542,0.6));
path.push_back(Eigen::Vector3d(1.20015,-3.26001,0.6));
path.push_back(Eigen::Vector3d(1.35059,-2.99881,0.6));
path.push_back(Eigen::Vector3d(1.52998,-2.76545,0.6));
path.push_back(Eigen::Vector3d(1.71382,-2.52717,0.6));
path.push_back(Eigen::Vector3d(1.94198,-2.32788,0.6));
path.push_back(Eigen::Vector3d(2.18735,-2.15189,0.6));
path.push_back(Eigen::Vector3d(2.44684,-1.96418,0.6));
path.push_back(Eigen::Vector3d(2.70953,-1.79194,0.6));
path.push_back(Eigen::Vector3d(2.97374,-1.5941,0.6));
path.push_back(Eigen::Vector3d(3.21313,-1.41574,0.6));
path.push_back(Eigen::Vector3d(3.47547,-1.22072,0.6));
path.push_back(Eigen::Vector3d(3.73225,-1.03671,0.6));
path.push_back(Eigen::Vector3d(3.9618,-0.857367,0.6));
path.push_back(Eigen::Vector3d(4.12944,-0.693893,0.6));
path.push_back(Eigen::Vector3d(4.29155,-0.563115,0.6));
path.push_back(Eigen::Vector3d(4.40124,-0.470492,0.6));
path.push_back(Eigen::Vector3d(4.49699,-0.400393,0.6));
path.push_back(Eigen::Vector3d(4.56959,-0.360315,0.6));
path.push_back(Eigen::Vector3d(4.61567,-0.324252,0.6));
path.push_back(Eigen::Vector3d(4.66054,-0.283401,0.6));
path.push_back(Eigen::Vector3d(4.68843,-0.234721,0.6));
path.push_back(Eigen::Vector3d(4.74275,-0.191777,0.6));
path.push_back(Eigen::Vector3d(4.7862,-0.165422,0.6));
path.push_back(Eigen::Vector3d(4.7622,-0.193422,0.6));
path.push_back(Eigen::Vector3d(4.7582,-0.201422,0.6));
path.push_back(Eigen::Vector3d(4.7222,-0.205422,0.6));
path.push_back(Eigen::Vector3d(4.77376,-0.172337,0.6));
path.push_back(Eigen::Vector3d(4.74976,-0.208337,0.6));
path.push_back(Eigen::Vector3d(4.76381,-0.20267,0.6));
path.push_back(Eigen::Vector3d(4.75181,-0.22667,0.6));
path.push_back(Eigen::Vector3d(4.78144,-0.205336,0.6));
path.push_back(Eigen::Vector3d(4.74144,-0.213336,0.6));
path.push_back(Eigen::Vector3d(4.77316,-0.186669,0.6));
path.push_back(Eigen::Vector3d(4.73316,-0.222669,0.6));



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
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
    path.push_back(Eigen::Vector3d(5.0,5.0,0.6));
path.push_back(Eigen::Vector3d(4.74106,4.74906,0.6));
path.push_back(Eigen::Vector3d(4.47567,4.51126,0.6));
path.push_back(Eigen::Vector3d(4.21991,4.26811,0.6));
path.push_back(Eigen::Vector3d(3.95247,4.01606,0.6));
path.push_back(Eigen::Vector3d(3.69399,3.77991,0.6));
path.push_back(Eigen::Vector3d(3.45493,3.56441,0.6));
path.push_back(Eigen::Vector3d(3.2287,3.32392,0.6));
path.push_back(Eigen::Vector3d(3.02432,3.12485,0.6));
path.push_back(Eigen::Vector3d(2.82901,2.92778,0.6));
path.push_back(Eigen::Vector3d(2.64486,2.72181,0.6));
path.push_back(Eigen::Vector3d(2.41122,2.49589,0.6));
path.push_back(Eigen::Vector3d(2.17068,2.25191,0.6));
path.push_back(Eigen::Vector3d(1.91775,1.9829,0.6));
path.push_back(Eigen::Vector3d(1.68425,1.73924,0.6));
path.push_back(Eigen::Vector3d(1.43057,1.49167,0.6));
path.push_back(Eigen::Vector3d(1.18509,1.252,0.6));
path.push_back(Eigen::Vector3d(0.923804,1.00024,0.6));
path.push_back(Eigen::Vector3d(0.686837,0.73232,0.6));
path.push_back(Eigen::Vector3d(0.429124,0.492769,0.6));
path.push_back(Eigen::Vector3d(0.195978,0.252937,0.6));
path.push_back(Eigen::Vector3d(-0.0692838,0.00116217,0.6));
path.push_back(Eigen::Vector3d(-0.33804,-0.254862,0.6));
path.push_back(Eigen::Vector3d(-0.606257,-0.519152,0.6));
path.push_back(Eigen::Vector3d(-0.912534,-0.685434,0.6));
path.push_back(Eigen::Vector3d(-1.26031,-0.960027,0.6));
path.push_back(Eigen::Vector3d(-1.58626,-1.21074,0.6));
path.push_back(Eigen::Vector3d(-1.95035,-1.46118,0.6));
path.push_back(Eigen::Vector3d(-2.28686,-1.71377,0.6));
path.push_back(Eigen::Vector3d(-2.62352,-1.97398,0.6));
path.push_back(Eigen::Vector3d(-2.93168,-2.2187,0.6));
path.push_back(Eigen::Vector3d(-3.19591,-2.43592,0.6));
path.push_back(Eigen::Vector3d(-3.43321,-2.71141,0.6));
path.push_back(Eigen::Vector3d(-3.64158,-2.96449,0.6));
path.push_back(Eigen::Vector3d(-3.83648,-3.24947,0.6));
path.push_back(Eigen::Vector3d(-4.03862,-3.50945,0.6));
path.push_back(Eigen::Vector3d(-4.2206,-3.77879,0.6));
path.push_back(Eigen::Vector3d(-4.38048,-4.05103,0.6));
path.push_back(Eigen::Vector3d(-4.51638,-4.27683,0.6));
path.push_back(Eigen::Vector3d(-4.64511,-4.46146,0.6));
path.push_back(Eigen::Vector3d(-4.74009,-4.60917,0.6));
path.push_back(Eigen::Vector3d(-4.83207,-4.71934,0.6));
path.push_back(Eigen::Vector3d(-4.87765,-4.81547,0.6));
path.push_back(Eigen::Vector3d(-4.88165,-4.82347,0.6));
path.push_back(Eigen::Vector3d(-4.90965,-4.84747,0.6));
path.push_back(Eigen::Vector3d(-4.92165,-4.88347,0.6));
path.push_back(Eigen::Vector3d(-4.96165,-4.90747,0.6));
path.push_back(Eigen::Vector3d(-4.98165,-4.91147,0.6));
path.push_back(Eigen::Vector3d(-5.01765,-4.94347,0.6));
path.push_back(Eigen::Vector3d(-5.04565,-4.95547,0.6));
path.push_back(Eigen::Vector3d(-5.05365,-4.97147,0.6));
path.push_back(Eigen::Vector3d(-5.08965,-4.98347,0.6));
path.push_back(Eigen::Vector3d(-5.10165,-5.02347,0.6));
path.push_back(Eigen::Vector3d(-5.11765,-5.04747,0.6));
path.push_back(Eigen::Vector3d(-5.12965,-5.08347,0.6));
path.push_back(Eigen::Vector3d(-5.16565,-5.11147,0.6));
path.push_back(Eigen::Vector3d(-5.18965,-5.14747,0.6));
path.push_back(Eigen::Vector3d(-5.22165,-5.17947,0.6));
path.push_back(Eigen::Vector3d(-5.23765,-5.21947,0.6));
path.push_back(Eigen::Vector3d(-5.48073,-4.89485,0.6));
path.push_back(Eigen::Vector3d(-5.63028,-4.74958,0.6));
path.push_back(Eigen::Vector3d(-5.75337,-4.65774,0.6));
path.push_back(Eigen::Vector3d(-5.85565,-4.57141,0.6));
path.push_back(Eigen::Vector3d(-5.96003,-4.48484,0.6));
path.push_back(Eigen::Vector3d(-6.07837,-4.40781,0.6));
path.push_back(Eigen::Vector3d(-6.22376,-4.3605,0.6));
path.push_back(Eigen::Vector3d(-6.35919,-4.29318,0.6));
path.push_back(Eigen::Vector3d(-6.51686,-4.25963,0.6));
path.push_back(Eigen::Vector3d(-6.7213,-4.22768,0.6));
path.push_back(Eigen::Vector3d(-6.92017,-4.22322,0.6));
path.push_back(Eigen::Vector3d(-7.11441,-4.26031,0.6));
path.push_back(Eigen::Vector3d(-7.29141,-4.27839,0.6));
path.push_back(Eigen::Vector3d(-7.47116,-4.29353,0.6));
path.push_back(Eigen::Vector3d(-7.66677,-4.29121,0.6));
path.push_back(Eigen::Vector3d(-7.83809,-4.34274,0.6));
path.push_back(Eigen::Vector3d(-7.975,-4.4013,0.6));
path.push_back(Eigen::Vector3d(-8.11372,-4.43525,0.6));
path.push_back(Eigen::Vector3d(-8.22639,-4.46444,0.6));
path.push_back(Eigen::Vector3d(-8.2905,-4.47055,0.6));
path.push_back(Eigen::Vector3d(-8.34832,-4.44003,0.6));
path.push_back(Eigen::Vector3d(-8.38734,-4.40455,0.6));
path.push_back(Eigen::Vector3d(-8.42085,-4.35466,0.6));
path.push_back(Eigen::Vector3d(-8.42063,-4.31417,0.6));
path.push_back(Eigen::Vector3d(-8.43364,-4.24852,0.6));
path.push_back(Eigen::Vector3d(-8.41931,-4.17483,0.6));
path.push_back(Eigen::Vector3d(-8.41131,-4.05775,0.6));
path.push_back(Eigen::Vector3d(-8.39724,-3.9664,0.6));
path.push_back(Eigen::Vector3d(-8.37429,-3.86443,0.6));
path.push_back(Eigen::Vector3d(-8.34508,-3.74104,0.6));
path.push_back(Eigen::Vector3d(-8.31938,-3.60379,0.6));
path.push_back(Eigen::Vector3d(-8.3096,-3.52,0.6));
path.push_back(Eigen::Vector3d(-8.32678,-3.46529,0.6));
path.push_back(Eigen::Vector3d(-8.35872,-3.45607,0.6));
path.push_back(Eigen::Vector3d(-8.40312,-3.4938,0.6));
path.push_back(Eigen::Vector3d(-8.43803,-3.65855,0.6));
path.push_back(Eigen::Vector3d(-8.44434,-3.89031,0.6));
path.push_back(Eigen::Vector3d(-8.39963,-4.11346,0.6));
path.push_back(Eigen::Vector3d(-8.30626,-4.28609,0.6));
path.push_back(Eigen::Vector3d(-8.17792,-4.41221,0.6));
path.push_back(Eigen::Vector3d(-8.02281,-4.5337,0.6));
path.push_back(Eigen::Vector3d(-7.8798,-4.648,0.6));
path.push_back(Eigen::Vector3d(-7.74877,-4.7572,0.6));
path.push_back(Eigen::Vector3d(-7.60258,-4.87341,0.6));
path.push_back(Eigen::Vector3d(-7.47376,-4.98595,0.6));
path.push_back(Eigen::Vector3d(-7.34409,-5.12996,0.6));
path.push_back(Eigen::Vector3d(-7.20847,-5.25912,0.6));
path.push_back(Eigen::Vector3d(-7.04948,-5.38653,0.6));
path.push_back(Eigen::Vector3d(-6.85733,-5.4995,0.6));
path.push_back(Eigen::Vector3d(-6.65572,-5.55593,0.6));
path.push_back(Eigen::Vector3d(-6.4564,-5.55014,0.6));
path.push_back(Eigen::Vector3d(-6.27692,-5.53449,0.6));
path.push_back(Eigen::Vector3d(-6.09419,-5.47375,0.6));
path.push_back(Eigen::Vector3d(-5.9118,-5.40414,0.6));
path.push_back(Eigen::Vector3d(-5.76544,-5.36331,0.6));
path.push_back(Eigen::Vector3d(-5.63235,-5.31465,0.6));
path.push_back(Eigen::Vector3d(-5.53388,-5.29172,0.6));
path.push_back(Eigen::Vector3d(-5.45111,-5.24137,0.6));
path.push_back(Eigen::Vector3d(-5.37289,-5.2251,0.6));
path.push_back(Eigen::Vector3d(-5.33831,-5.21208,0.6));
path.push_back(Eigen::Vector3d(-5.29065,-5.19766,0.6));
path.push_back(Eigen::Vector3d(-5.25652,-5.17813,0.6));
path.push_back(Eigen::Vector3d(-5.28852,-5.18613,0.6));
path.push_back(Eigen::Vector3d(-5.26934,-5.14033,0.6));
path.push_back(Eigen::Vector3d(-5.28134,-5.16433,0.6));
path.push_back(Eigen::Vector3d(-5.27664,-5.1276,0.6));
path.push_back(Eigen::Vector3d(-5.31264,-5.1636,0.6));
path.push_back(Eigen::Vector3d(-5.26448,-5.14623,0.6));
path.push_back(Eigen::Vector3d(-5.28048,-5.15023,0.6));
path.push_back(Eigen::Vector3d(-5.24439,-5.15219,0.6));
path.push_back(Eigen::Vector3d(-5.28439,-5.16019,0.6));
path.push_back(Eigen::Vector3d(-5.23951,-5.15615,0.6));
path.push_back(Eigen::Vector3d(-5.24351,-5.18415,0.6));
path.push_back(Eigen::Vector3d(-5.26751,-5.18815,0.6));
path.push_back(Eigen::Vector3d(-5.23801,-5.17452,0.6));
path.push_back(Eigen::Vector3d(-5.24201,-5.18652,0.6));
path.push_back(Eigen::Vector3d(-5.28201,-5.21052,0.6));
path.push_back(Eigen::Vector3d(-5.25361,-5.20042,0.6));
path.push_back(Eigen::Vector3d(-5.22288,-5.20033,0.6));
path.push_back(Eigen::Vector3d(-5.25888,-5.22433,0.6));
path.push_back(Eigen::Vector3d(-5.22311,-5.19147,0.6));
path.push_back(Eigen::Vector3d(-5.23911,-5.20347,0.6));
path.push_back(Eigen::Vector3d(-5.27511,-5.23947,0.6));
path.push_back(Eigen::Vector3d(-5.25609,-5.21557,0.6));
path.push_back(Eigen::Vector3d(-5.21287,-5.18846,0.6));
path.push_back(Eigen::Vector3d(-5.21687,-5.20446,0.6));
path.push_back(Eigen::Vector3d(-5.22487,-5.22446,0.6));
path.push_back(Eigen::Vector3d(-5.19189,-5.21157,0.6));
path.push_back(Eigen::Vector3d(-5.22789,-5.22357,0.6));
path.push_back(Eigen::Vector3d(-5.19032,-5.21085,0.6));
path.push_back(Eigen::Vector3d(-5.21432,-5.25085,0.6));
path.push_back(Eigen::Vector3d(-5.17545,-5.22868,0.6));
path.push_back(Eigen::Vector3d(-5.20745,-5.26868,0.6));
path.push_back(Eigen::Vector3d(-5.16996,-5.22295,0.6));
path.push_back(Eigen::Vector3d(-5.20996,-5.25895,0.6));
path.push_back(Eigen::Vector3d(-5.17997,-5.23516,0.6));
path.push_back(Eigen::Vector3d(-5.19597,-5.27116,0.6));
path.push_back(Eigen::Vector3d(-5.16878,-5.24493,0.6));
path.push_back(Eigen::Vector3d(-5.20078,-5.27693,0.6));
path.push_back(Eigen::Vector3d(-5.18462,-5.23754,0.6));
path.push_back(Eigen::Vector3d(-5.20862,-5.24154,0.6));
path.push_back(Eigen::Vector3d(-5.2069,-5.22523,0.6));



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
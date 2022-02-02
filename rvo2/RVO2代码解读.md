# RVO2代码解读

## file:example.cpp

主函数部分(不是全部的代码，完全版请对照源代码)：

```c++
ros::init(argc, argv, "rvo2_example");//init
ros::NodeHandle n;
q0_sub = n.subscribe("/q0/ground_truth_to_tf/pose", 0, q0_pose_callback);
//topicname:/q0/ground_truth_to_tf/pose,就是订阅当前位置

q0_cmd_vel = n.advertise<geometry_msgs::Twist>("/q0/pref_vel", 0);
//topicname:/q0/pref_vel,就是发布一个msg（Twist），q0_cmd_vel在之前就已经定义过ros::Publisher
//但是我不知道为什么后面是0？

while (ros::ok()) {
    ros::spinOnce();
    
    //q0_pos_set是bool型，是为了确定位置是否设定，初始为false，但是经过之前的/q0/ground_truth_to_tf/pose的订阅中的回调函数q0_pose_callback
    if (q0_pos_set and q1_pos_set and q2_pos_set) 
    {
        RVO::RVOSimulator *sim = new RVO::RVOSimulator();
        
        /*
        1.RVO_API RVOSimulator(float timeStep, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity = Vector3());
        
        2.(1)RVO_API size_t addAgent(const Vector3 &position);
          (2)RVO_API size_t addAgent(const Vector3 &position, float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity = Vector3());
        */
        sim->setTimeStep(1.0 / 30);//步长，跟rvo2有关系
        //param:float neighborDist, size_t maxNeighbors, float timeHorizon, float radius, float maxSpeed, const Vector3 &velocity
        sim->setAgentDefaults(2, 2, 10, 2, 1);
        //pose_to_vector是把位置变成vector的函数
        sim->addAgent(pose_to_vector(q0_pose));
        
        //param:size_t agentNo, const Vector3 &prefVelocity(The replacement of the three-dimensional preferred velocity)
        sim->setAgentPrefVelocity(0, RVO::Vector3(1, 1, 1));
        sim->setAgentPrefVelocity(1, RVO::Vector3(-1, -1, 1));
        sim->setAgentPrefVelocity(2, RVO::Vector3(-1, 1, 1));
        ros::Rate r(30);
        ros::Time st = ros::Time::now();
        while (ros::ok()) {
            sim->globalTime_ = ros::Time::now().toSec();
            sim->setAgentPosition(0, pose_to_vector(q0_pose));
            sim->setAgentPosition(1, pose_to_vector(q1_pose));
            sim->setAgentPosition(2, pose_to_vector(q2_pose));
            sim->setAgentVelocity(0, q0_vel);
            sim->setAgentVelocity(1, q1_vel);
            sim->setAgentVelocity(2, q2_vel);
            sim->doStep();
			
            //RVO::normalize是归一化的意思
            // Sends commands to the quads
            q0_cmd_vel.publish(vector_to_twist(sim->getAgentVelocity(0)));
            q1_cmd_vel.publish(vector_to_twist(sim->getAgentVelocity(1)));
            q2_cmd_vel.publish(vector_to_twist(sim->getAgentVelocity(2)));
            q0_cmd_vel.publish(vector_to_twist(sim->getAgentPrefVelocity(0)));
            q1_cmd_vel.publish(vector_to_twist(sim->getAgentPrefVelocity(1)));
            q2_cmd_vel.publish(vector_to_twist(sim->getAgentPrefVelocity(2)));
            q0_cmd_vel.publish(vector_to_twist(
                RVO::normalize(RVO::Vector3(1, 1, 1))));
            q1_cmd_vel.publish(vector_to_twist(
                RVO::normalize(RVO::Vector3(-1, -1, 1))));
            q2_cmd_vel.publish(vector_to_twist(
                RVO::normalize(RVO::Vector3(-1, 1, 1))));
            q3_cmd_vel.publish(vector_to_twist(
                RVO::normalize(RVO::Vector3(1, -1, 1))));

            r.sleep();
            ros::spinOnce();
        }
    }
}

```

## file:example.launch

```javascript
<?xml version="1.0"?>

<launch>

    <!-- Start Gazebo with wg world running in (max) realtime -->
    <include file="$(find hector_gazebo_worlds)/launch/start.launch"/>

    <!-- Spawn simulated quadrotor uav -->
    <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch" ns="q0">
        <arg name="model"
            value="$(find hector_quadrotor_description)/urdf/quadrotor_hokuyo_utm30lx.gazebo.xacro"/>
        <arg name="x" value="0.0"/>
        <arg name="y" value="0.0"/>
        <arg name="z" value="0.3"/>
        <arg name="name" value="q0"/>
    </include>

    <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch" ns="q1">
        <arg name="model"
            value="$(find hector_quadrotor_description)/urdf/quadrotor_hokuyo_utm30lx.gazebo.xacro"/>
        <arg name="x" value="5.0"/>
        <arg name="y" value="5.0"/>
        <arg name="z" value="0.3"/>
        <arg name="name" value="q1"/>
    </include>

    <include file="$(find hector_quadrotor_gazebo)/launch/spawn_quadrotor.launch" ns="q2">
        <arg name="model"
            value="$(find hector_quadrotor_description)/urdf/quadrotor_hokuyo_utm30lx.gazebo.xacro"/>
        <arg name="x" value="5.0"/>
        <arg name="y" value="0"/>
        <arg name="z" value="0.3"/>
        <arg name="name" value="q2"/>
    </include>

    <node pkg="rvo2" type="example" name="rvo2_example"/>
</launch>

```

### 初始运行example

```
Warning：
1.Calling service /gazebo/spawn_urdf model 
QinotifyFileSystemWatchCherEngine::addPaths: inotify_add_watch failed:设备上没有空间


2.Desired controller update period(0.01) is slower than the gazebo simulation period(0.001s)
```


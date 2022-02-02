



# geometry_msgs

## [geometry_msgs::PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html)

``` 
Raw Message Definition:
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose 

Compact Message Definition:
std_msgs/Header header
geometry_msgs/Pose pose


```



### [std_msgs](http://docs.ros.org/en/api/std_msgs/html/index-msg.html)/Header Message

引用：[here](https://blog.csdn.net/qq_18676517/article/details/109270525)

```
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 

uint32 seq

#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library

time stamp

#Frame this data is associated with

string frame_id
```



### [geometry_msgs](http://docs.ros.org/en/api/geometry_msgs/html/index-msg.html)/Pose Message

```
Raw Message Definition
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

Compact Message Definition
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
```

#### [geometry_msgs](http://docs.ros.org/en/api/geometry_msgs/html/index-msg.html)/Point Message

```
# This contains the position of a point in free space
float64 x
float64 y
float64 z
```

#### [geometry_msgs](http://docs.ros.org/en/api/geometry_msgs/html/index-msg.html)/Quaternion Message

```
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
```

## [geometry_msgs](http://docs.ros.org/en/melodic/api/geometry_msgs/html/index-msg.html)/Twist Message

cite:[here](https://blog.csdn.net/ktigerhero3/article/details/80740920)

``` 
# This expresses velocity in free space broken into its linear and angular parts.
Vector3 linear
Vector3 angular 
```

### [geometry_msgs](http://docs.ros.org/en/melodic/api/geometry_msgs/html/index-msg.html)/Vector3 Message

```
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
```

使用方法：

```c++
int main(int argc,char** argv)
{
    ros::init(argc,argv,"init");
    ros::NodeHandle nh;
    ros::Publisher Pub = nh.advertise<geometry_msgs::Twist>("/hector1/cmd_vel",10)
    
// 初始化geometry_msgs::Twist类型的消息    
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x=0.2;
    vel_msg.angular.z=0.2;
    
// 设置循环的频率
    ros::Rate loop_rate(10);
    
    while(ros::ok)
    {
        ros::spinonce();
        Pub.publish(Twist);
        
        ROS_INFO("Publsh velocity command[%0.2f m/s, %0.2f rad/s]", 
				vel_msg.linear.x, vel_msg.angular.z);
        
        loop_rate.sleep();
    }
    return 0;
}
```


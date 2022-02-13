# ROS学习:RVO2之**orca_pkg代码解读**

## act_orca_test.cpp

```c++
orca_radius = para['R'];
orca_max_spead = para['V'];
orca_max_error = para['E'];
orca_rand_para = para['P'];
safe_radius = para['S'];
orca_time_step = para['T'];
orca_neighbor_dist = para['D'];
```

首先我们先确定参数：

> \* @param[in] a map para contain key and value we need to set .
>
>​    \* etc:
>
>​    \* AGENT_ACCOUNT---the number of swarms  						数量 A
>
>​    \* orca_radius----Aircraft radius     												飞行器半径R
>
>​    \* orca_max_error---max error distance from goal point 		最大目标误差E
>
>​    \* orca_rand_para---noise ,avoid deadlock      							噪音P					
>
>​    \* safe_radius--avoid radius =orca_radius(Aircraft radius)+safe_radius  安全半径S
>
>​    \* orca_time_step----step size 														步长T
>
>​    \* orca_neighbor_dist----min dis between the two neighbors ,suggested:more than double of  Aircraft radius																									邻居距离D
>
>orca_max_spead = para['V']; 															最大速度V


/*
作用：用于飞行器队形形成、编队运动。
包含函数：
*/
#ifndef ORCA_API_H
#define ORCA_API_H


#include <string>
#include <stdio.h>
#include<stdlib.h>
#include "orca.h"
#include <iostream>
#include <math.h>
#include <cmath>
#include <time.h>
#include "UAV_STATE.h"

using namespace std;

/*
用于计算飞行器的期望速度
计算方式：分布式。（每一架飞机需要单独调用此函数，以获得自身的期望速度）
输入：
	uav--UAV类数组  调用函数前需要更新全部的uav类里面的velocity,position，以及自身uav[myself_id].goalPosition.
	agent_id--调用此函数的飞行器id
	Goal_velocity--目标速度
    orca_time_step--仿真时间步长
	orca_neighbor_dist--相邻飞行器的中心点最小距离。（需大于两倍半径）
	orca_radius--飞行器半径（需预留安全距离）
	orca_rand_para--速度增加的随机值（建议大于50）
	orca_max_error--判断飞行器是否到达目标点的最大误差，小于这个值，判定飞行器到达目标位置
*/
void ORCA_COMPUTE_VELOCITY(UAV *uav, int agent_id, int AGENT_ACCOUNT,double*Goal_velocity,double orca_time_step, double orca_neighbor_dist, double orca_radius, double orca_max_spead,double orca_rand_para, double orca_max_error);

#endif

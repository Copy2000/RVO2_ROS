/*
���ã����ڷ����������γɡ�����˶���
����������
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
���ڼ���������������ٶ�
���㷽ʽ���ֲ�ʽ����ÿһ�ܷɻ���Ҫ�������ô˺������Ի������������ٶȣ�
���룺
	uav--UAV������  ���ú���ǰ��Ҫ����ȫ����uav�������velocity,position���Լ�����uav[myself_id].goalPosition.
	agent_id--���ô˺����ķ�����id
	Goal_velocity--Ŀ���ٶ�
    orca_time_step--����ʱ�䲽��
	orca_neighbor_dist--���ڷ����������ĵ���С���롣������������뾶��
	orca_radius--�������뾶����Ԥ����ȫ���룩
	orca_rand_para--�ٶ����ӵ����ֵ���������50��
	orca_max_error--�жϷ������Ƿ񵽴�Ŀ���������С�����ֵ���ж�����������Ŀ��λ��
*/
void ORCA_COMPUTE_VELOCITY(UAV *uav, int agent_id, int AGENT_ACCOUNT,double*Goal_velocity,double orca_time_step, double orca_neighbor_dist, double orca_radius, double orca_max_spead,double orca_rand_para, double orca_max_error);

#endif

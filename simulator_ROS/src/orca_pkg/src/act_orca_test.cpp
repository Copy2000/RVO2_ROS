/* ==================================================================
* Copyright (c) 2018, micROS Group, TAIIC, NIIDT & HPCL.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* 1. Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the
* distribution.
* 3. All advertising materials mentioning features or use of this software
* must display the following acknowledgement:
* This product includes software developed by the micROS Group. and
* its contributors.
* 4. Neither the name of the Group nor the names of its contributors may
* be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY MICROS,GROUP AND CONTRIBUTORS
* ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
* NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
* FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
* THE MICROS,GROUP OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
* OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
* OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* ===================================================================
* Author: Ren Xiaoguang, Wu Yunlong, Li Jinghua, micROS-DA Team, TAIIC.
*/
#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include "ros/ros.h"
#include "msg_base.h"
#include "qrotor_control_orca_impl.h"
//#include <trajectory_msgs/MultiDOFJointTrajectory.h>

bool whether_reach_goal_position(const std::vector<msg_base::QRotorStateStruct> &swarms,
	const std::vector<msg_base::QRotorGoalStruct> &goal_swarms, int i, int AGENT_ACCOUNT, double orca_max_error)
{

	double temp = 0;
	temp += pow((swarms[i].pos_D - goal_swarms[i].goal_pos_D), 2);
	temp += pow((swarms[i].pos_E - goal_swarms[i].goal_pos_E), 2);
	temp += pow((swarms[i].pos_N - goal_swarms[i].goal_pos_N), 2);
	if (temp < orca_max_error)
		return true;
	return false;

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "act_orca_test");

  ros::NodeHandle nh;

  QRotorControlORCAImpl qrotor_control_impl_ptr_;
 		
	std::ofstream fout;
	fout.open("data_orca.txt");
	msg_base::QRotorStateStruct testFX;
	testFX.pos_N = 1.2; testFX.pos_E = 1.2; testFX.pos_D = 1.2;
	testFX.Va_N = 1.2; testFX.Va_E = 1.2; testFX.Va_D = 1.2; testFX.chi = 2;
	int UAV_NUM = 8;																		//改变无人机数量
	int AGENT_ACCOUNT = UAV_NUM;
	double orca_time_step = 0.2f;
	double height=0.6;
	std::vector<msg_base::QRotorStateStruct> swarms(AGENT_ACCOUNT);
	std::vector<msg_base::QRotorGoalStruct> goal_swarms(AGENT_ACCOUNT);
	std::vector<msg_base::QRotorCmdStruct> cmd_swarms(AGENT_ACCOUNT);
	
	//qrotor_control_impl_ptr_->hello();

	//initialize the scene
	swarms[0].pos_E = 5; swarms[0].pos_N = -5; swarms[0].pos_D = 0;										//initial point 
	swarms[0].Va_E = 0; swarms[0].Va_N = 0; swarms[0].Va_D = 0;												//Aircraft radius
	goal_swarms[0].goal_pos_E = -5; goal_swarms[0].goal_pos_N = 5; goal_swarms[0].goal_pos_D = 0;		//goal point 
	//goal_swarms[0].goal_pos_E = 0; goal_swarms[0].goal_pos_N = 0; goal_swarms[0].goal_pos_D = 0;			//another goal point

	swarms[1].pos_E = -5; swarms[1].pos_N = 5; swarms[1].pos_D = 0;
	swarms[1].Va_E = 0; swarms[1].Va_N = 0; swarms[1].Va_D = 0;
	goal_swarms[1].goal_pos_E = 5; goal_swarms[1].goal_pos_N = -5; goal_swarms[1].goal_pos_D = 0;
	//goal_swarms[1].goal_pos_E =20; goal_swarms[1].goal_pos_N = 0; goal_swarms[1].goal_pos_D = 0;

	swarms[2].pos_E = 5; swarms[2].pos_N = 5; swarms[2].pos_D = 0;
	swarms[2].Va_E = 0; swarms[2].Va_N = 0; swarms[2].Va_D = 0;
	goal_swarms[2].goal_pos_E = -5; goal_swarms[2].goal_pos_N = -5; goal_swarms[2].goal_pos_D = 0;
	//goal_swarms[2].goal_pos_E = -20; goal_swarms[2].goal_pos_N = 0; goal_swarms[2].goal_pos_D = 0;

	swarms[3].pos_E = -5; swarms[3].pos_N = -5; swarms[3].pos_D = 0;
	swarms[3].Va_E = 0; swarms[3].Va_N = 0; swarms[3].Va_D = 0;
	goal_swarms[3].goal_pos_E = 5; goal_swarms[3].goal_pos_N = 5; goal_swarms[3].goal_pos_D = 0;
	//goal_swarms[3].goal_pos_E =40; goal_swarms[3].goal_pos_N = 0; goal_swarms[3].goal_pos_D = 0;

	//以下为自己添加
	swarms[4].pos_E = -5; swarms[4].pos_N = 0; swarms[4].pos_D = 0;
	swarms[4].Va_E = 0; swarms[4].Va_N = 0; swarms[4].Va_D = 0;
	goal_swarms[4].goal_pos_E = 5; goal_swarms[4].goal_pos_N = 0; goal_swarms[4].goal_pos_D = 0;

	swarms[5].pos_E = 5; swarms[5].pos_N = 0; swarms[5].pos_D = 0;
	swarms[5].Va_E = 0; swarms[5].Va_N = 0; swarms[5].Va_D = 0;
	goal_swarms[5].goal_pos_E = -5; goal_swarms[5].goal_pos_N = 0; goal_swarms[5].goal_pos_D = 0;

	swarms[6].pos_E = 0; swarms[6].pos_N = 5; swarms[6].pos_D = 0;
	swarms[6].Va_E = 0; swarms[6].Va_N = 0; swarms[6].Va_D = 0;
	goal_swarms[6].goal_pos_E = 0; goal_swarms[6].goal_pos_N = -5; goal_swarms[6].goal_pos_D = 0;

	swarms[7].pos_E = 0; swarms[7].pos_N =-5; swarms[7].pos_D = 0;
	swarms[7].Va_E = 0; swarms[7].Va_N = 0; swarms[7].Va_D = 0;
	goal_swarms[7].goal_pos_E = 0; goal_swarms[7].goal_pos_N = 5; goal_swarms[7].goal_pos_D = 0;


	for (int i = 0; i<UAV_NUM; i++)
		fout << swarms[i].pos_E << "\t" << swarms[i].pos_N <<"\t"<<height<<"\t"<<1.5708<<"\t";//<< "," << swarms[i].pos_D << ",";
	fout << std::endl;
	  

	  
	while (ros::ok())
	{
		//qrotor_control_orca_impl::QRotorControlORCAImpl q;
			
		std::map<char, double>para;
		para['A'] = 8;								//改变无人机数量
		para['R'] = 0.3;
		para['V'] = 2.5;
		para['E'] = 0.1;
		para['P'] = 50;
		para['S'] = 1;
		para['T'] = 0.2;
		para['D'] = 5;

		//setParameters
		qrotor_control_impl_ptr_.setParmeters(para);
		
		
		int sum_reach = 0;
		for (int i = 0; i < UAV_NUM; i++)
		{

			if (whether_reach_goal_position(swarms, goal_swarms, i, AGENT_ACCOUNT, 0.1) == true)			//if arrive
				sum_reach++;
		}
		
		
		if (sum_reach <= 7)
		{
      		msg_base::QRotorCmdStruct cmd_uav;
		  	for(int i=0;i<AGENT_ACCOUNT;i++)
        		qrotor_control_impl_ptr_.calculateCommand(swarms, goal_swarms,i, cmd_swarms,cmd_uav);		//launch orca ,get velocity command
		}
		else
			break;

		for (int i = 0; i < UAV_NUM; i++)				//use velocity command to contral (update velocity)
		{
			swarms[i].Va_E = cmd_swarms[i].Va_E_c;
			swarms[i].Va_N = cmd_swarms[i].Va_N_c;
			swarms[i].Va_D = cmd_swarms[i].Va_D_c;
			swarms[i].pos_E += swarms[i].Va_E*orca_time_step;
			swarms[i].pos_N += swarms[i].Va_N*orca_time_step;
			swarms[i].pos_D += swarms[i].Va_D*orca_time_step;
			//std::cout << "V  " << i << " " << swarms[i].Va_E << " " << swarms[i].Va_N << " " << swarms[i].Va_D << std::endl;
			std::cout << "position " << i << "\t" << swarms[i].pos_E << "\t"<< swarms[i].pos_N << "\t"<< swarms[i].pos_D <<1.5708<<"\t"<< std::endl;
		}


		for (int i = 0; i <UAV_NUM; i++)
			fout << swarms[i].pos_E << "\t" << swarms[i].pos_N <<"\t"<<height<<"\t"<<1.5708<<"\t";//<< "," << swarms[i].pos_D << ",";
		fout << std::endl;
		
		
		//Sleep(1000);
	}
	fout.close();


	

  return 0; 
}


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
* Author: Cheng Hui, Mao Jiduo, Xu Tianye, NCS Lab.
*/
#include "orca_pkg/qrotor_control_orca_impl.h"


	QRotorControlORCAImpl::QRotorControlORCAImpl()
	{
		ROS_INFO("[Class] QRotorControlORCAImpl Loaded!");
		std::cout << "begin" << std::endl;
	}
	void QRotorControlORCAImpl::setParmeters( std::map<char, double> &para) {
		
		 //AGENT_ACCOUNT = para['A'];
		 orca_radius = para['R'];
		 orca_max_spead = para['V'];
		 orca_max_error = para['E'];
		 orca_rand_para = para['P'];
		 safe_radius = para['S'];
		 orca_time_step = para['T'];
		 orca_neighbor_dist = para['D'];
	}
	
	void QRotorControlORCAImpl::calculateCommand(const std::vector<msg_base::QRotorStateStruct> &swarms,
		const std::vector<msg_base::QRotorGoalStruct> &goal_swarms,const int agent_id,
		std::vector<msg_base::QRotorCmdStruct> &cmd_swarms,msg_base::QRotorCmdStruct &cmd_uav){
		//std::cout<<"orca calculateCommand"<<std::endl;
	  AGENT_ACCOUNT=swarms.size();
		if (AGENT_ACCOUNT < 1)
		{
			std::cout << "account < 1  error!" << std::endl;
			return;
		}
		UAV *uav = new UAV[AGENT_ACCOUNT];
		double *goal_velocity1 = new double[3];
		
		for (int i = 0; i < AGENT_ACCOUNT; i++)
		{
			uav[i].setPosition(swarms[i].pos_E, swarms[i].pos_N, swarms[i].pos_D);
			uav[i].setVelocity(swarms[i].Va_E, swarms[i].Va_N, swarms[i].Va_D);
			uav[i].setgoalPosition(goal_swarms[i].goal_pos_E, goal_swarms[i].goal_pos_N, goal_swarms[i].goal_pos_D);
		}
		//for (int i = 0; i < AGENT_ACCOUNT; i++)
		//{
			
			ORCA_COMPUTE_VELOCITY(uav, agent_id, AGENT_ACCOUNT, goal_velocity1, orca_time_step, orca_neighbor_dist, orca_radius + safe_radius, orca_max_spead, orca_rand_para, orca_max_error);
			cmd_swarms[agent_id].Va_E_c = goal_velocity1[0]; cmd_swarms[agent_id].Va_N_c = goal_velocity1[1]; cmd_swarms[agent_id].Va_D_c = goal_velocity1[2];
      cmd_uav=cmd_swarms[agent_id];
		//}
			
	}
	



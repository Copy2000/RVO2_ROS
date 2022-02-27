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
#ifndef __QROTOR_CONTROL_ORCA_IMPL_H__
#define __QROTOR_CONTROL_ORCA_IMPL_H__

#include "../orca/ORCA_API.h"
#include "msg_base.h" 


#include <map>
#include <stdio.h>
#include "ros/ros.h"
#include <iostream>
#include "../orca/UAV_STATE.h"

class QRotorControlORCAImpl{
	public:
	   /**
       * @brief Construct a new QRotorControlORCAImpl object
       */
		QRotorControlORCAImpl();
		/*setParameters 
	    * @brief setparameters about algorithm
        * 
        * @param[in] a map para contain key and value we need to set .
		* etc:
	   	* AGENT_ACCOUNT---the number of swarms
		* orca_radius----Aircraft radius
		* orca_max_error---max error distance from goal point
		* orca_rand_para---noise ,avoid deadlock
		* safe_radius--avoid radius =orca_radius(Aircraft radius)+safe_radius
		* orca_time_step----step size 
		* orca_neighbor_dist----min dis between the two neighbors ,suggested:more than double of  Aircraft radius
		*/
		virtual void setParmeters(std::map<char, double> &para) ;
		
       /**
       * @brief Calculate quadrotor platform's command
       * 
       * @param[in] swarms every quadrotor's  state stored in a vector
       * @param[in] goal_swarms every quadrotor's goal_position stored in a vector
       * @param[out] cmd_swarms every quadrotor's command_velocity stored in a vector
       */
		virtual void calculateCommand(const std::vector<msg_base::QRotorStateStruct> &swarms,
                                  const std::vector<msg_base::QRotorGoalStruct> &goal_swarms,const int agent_id,
                                  std::vector<msg_base::QRotorCmdStruct> &cmd_swarms,msg_base::QRotorCmdStruct &cmd_uav);
	private:
		int AGENT_ACCOUNT = 6;
		double orca_radius = 0.5f;
		double orca_max_spead = 3.0f;
		double orca_max_error = 0.1f;
		double orca_rand_para = 150;
		double safe_radius = 0.2f;
		double orca_neighbor_dist = 2.0f;
	public:
		double orca_time_step = 0.2f;
	};
#endif

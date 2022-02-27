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
* Author: Ren Xiaoguang, Wu Yunlong, Li Jinghua, micROS-DA Team, NIIDT, TAIIC.
*/

#ifndef __MSG_BASE_H__
#define __MSG_BASE_H__

#include <string>
#include <vector>
/**
* @brief Message Base Namespace
*/
namespace msg_base
{
	/**
	* @brief Fixed-wing state struct
	*/
	struct FWingStateStruct
	{
		float pos_N;  ///< Position in north direction (m)
		float pos_E;  ///< Position in east direction (m)
		float pos_D;	///< Position in negative height direction (m)
		float Va;	    ///< Airspeed (m/s)
		float chi;	  ///< Course (rad)
	};

	/**
	* @brief Fixed-wing command struct
	*/
	struct FWingCmdStruct
	{
		float Va_c;        ///< Commanded airspeed (m/s)
		float h_c;         ///< Commanded altitude (m)
		float chi_c;       ///< Commanded course (rad)
		float phi_ff;      ///< Feed forward command for orbits (rad)
		float aux[4];      ///< Optional auxiliary commands
		bool aux_valid;    ///< Auxiliary commands valid
	};

	/**
	* @brief Virtual agent (VA) struct
	*/
	struct FWingVAStruct
	{
		float x;	///< Virtual agent (VA) position in east direction
		float y;	///< Virtual agnet (VA) position in north direction
		float z;	///< Virtual agnet (VA) position in altitude direction
	};

	/**
	* @brief Waypoint struct
	*/
	struct FWingWaypointStruct
	{
		float w[3];           ///< Waypoint in local NED (m)
		float chi_d;          ///< Desired course at this waypoint (rad)
		float Va_d;           ///< Desired airspeed (m/s)
		bool chi_valid;       ///< Desired course valid (dubin or fillet paths)
		bool landing;         ///< Used by Landing actor plugin
		bool loiter_point;    ///< Used by Landing actor plugin
		bool set_current;     ///< Sets this waypoint to be executed now! Starts a new list
		bool clear_wp_list;   ///< Removes all waypoints and returns to origin.  The rest of this message will be ignored
	};

	/**
	* @brief Communication configuration struct
	*/
	struct CommConfigStruct
	{
		int mode;   ///< Communication mode from Liu's paper
	};


	/**
	* @brief Quadrotor state struct
	*/
	struct QRotorStateStruct
	{
		float pos_N;  ///< Position in north direction (m)
		float pos_E;  ///< Position in east direction (m)
		float pos_D;	///< Position in negative height direction (m)
		float Va_N;	    ///< Airspeed (m/s)  in north direction
		float Va_E;	    ///< Airspeed (m/s) in east direction
		float Va_D;	    ///< Airspeed (m/s) in negative height direction 
		float chi;	  ///< Course (rad)
	};

	/**
	* @brief Quadrotor command struct
	*/
	struct QRotorCmdStruct
	{
		float Va_N_c;	    ///< Airspeed (m/s)  in north direction
		float Va_E_c;	    ///< Airspeed (m/s) in east direction
		float Va_D_c;	    ///< Airspeed (m/s) in negative height direction 
		
	};

	/**
	* @brief Virtual agent (VA) struct
	*/
	struct QRotorVAStruct
	{
		float x;  ///< Virtual agent (VA) position in east direction
		float y;  ///< Virtual agnet (VA) position in north direction
		float z;  ///< Virtual agnet (VA) position in north direction
		float v_N;  ///< Virtual agent (VA) velocity in north direction
		float v_E;  ///< Virtual agnet (VA) velocity in east direction
		float v_D;  ///< Virtual agent (VA) velocity in height direction
		
	};
	struct QRotorGoalStruct
	{
		float goal_pos_N;  ///< Goal Position in north direction (m)
		float goal_pos_E;  ///< Goal Position in east direction (m)
		float goal_pos_D;	///< Goal Position in negative height direction (m)
		
	};
}

#endif
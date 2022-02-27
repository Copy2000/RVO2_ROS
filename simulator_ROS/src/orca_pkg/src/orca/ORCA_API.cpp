#include "ORCA_API.h"


void ORCA_COMPUTE_VELOCITY(UAV *uav, int agent_id,  int AGENT_ACCOUNT,double *Goal_velocity,double orca_time_step,double orca_neighbor_dist,double orca_radius,double orca_max_spead, double orca_rand_para, double orca_max_error)
{
	
	double ** current_position = new double *[AGENT_ACCOUNT];
	double ** current_velocity = new double *[AGENT_ACCOUNT];
	double ** goal_position = new double *[AGENT_ACCOUNT];
	double ** goal_velocity = new double *[AGENT_ACCOUNT];
	for (int i = 0; i < AGENT_ACCOUNT; i++)
	{
		current_position[i] = new double[3];
		current_velocity[i] = new double[3];
		goal_position[i] = new double[3];
		goal_velocity[i] = new double[3];
	}
		
	
	for (int i = 0; i < AGENT_ACCOUNT; i++)
	{
		current_position[i][0] = uav[i].read_Position()[0]; current_position[i][1] = uav[i].read_Position()[1]; current_position[i][2] = uav[i].read_Position()[2];
		current_velocity[i][0]= uav[i].read_Velocity()[0]; current_velocity[i][1] = uav[i].read_Velocity()[1]; current_velocity[i][2] = uav[i].read_Velocity()[2];
		goal_position[i][0] = uav[i].read_goalPosition()[0]; goal_position[i][1] = uav[i].read_goalPosition()[1]; goal_position[i][2] = uav[i].read_goalPosition()[2];
	}
   
	RVO::RVOSimulator *sim = new RVO::RVOSimulator();
	setupScenario(sim, current_position, goal_position, current_velocity, AGENT_ACCOUNT, orca_time_step,orca_neighbor_dist, orca_radius, orca_max_spead);
		if (reachedGoal(current_position[agent_id], goal_position[agent_id],orca_max_error) == false)//未达到目标点
		{
			renewScenario(sim, current_position, goal_position, current_velocity, AGENT_ACCOUNT);//更新场景
			setPreferredVelocities(sim, static_cast<size_t>(agent_id));//设置偏好速度
			
			sim->doStep(static_cast<size_t>(agent_id)); //运行一次orca算法


			goal_velocity[agent_id][0] = sim->getAgentVelocity(agent_id).x();
			goal_velocity[agent_id][1] = sim->getAgentVelocity(agent_id).y();
			goal_velocity[agent_id][2] = sim->getAgentVelocity(agent_id).z();
		}
		else//达到目标点悬停
		{
			goal_velocity[agent_id][0] = 0;
			goal_velocity[agent_id][1] = 0;
			goal_velocity[agent_id][2] = 0;

		}
		
		if (orca_rand_para <= 30)
			orca_rand_para = 30;
		Goal_velocity[0] = goal_velocity[agent_id][0] +(rand() % 10 - 10) / orca_rand_para;
		Goal_velocity[1] = goal_velocity[agent_id][1] +(rand() % 10 - 10) / orca_rand_para;
		Goal_velocity[2] = 0;
		//Goal_velocity[2] = goal_velocity[agent_id][2] + (rand() % 10 - 10) / orca_rand_para;   //三维避碰

	//return Goal_velocity;
}

/*double* active_formation_control(UAV *uav, int agent_id, int Leader_num, double* Leader_goal_position, double desire_velocity,double time_step)
{
	double* relative_dis = new double[2];

	//double** a = new double *[2];
	relative_dis[0] = Leader_goal_position[0] - uav[Leader_num].read_Position()[0];
	relative_dis[1] = Leader_goal_position[1] - uav[Leader_num].read_Position()[1];
	relative_dis[2] = Leader_goal_position[2] - uav[Leader_num].read_Position()[2];
	//relative_dis[2] = Leader_goal_position[2] - uav[Leader_num].read_Position()[2];
	double temp;
	temp = sqrt(pow(relative_dis[0], 2) + pow(relative_dis[1], 2));
	relative_dis[0] = desire_velocity*(relative_dis[0] / temp);
	relative_dis[1] = desire_velocity*(relative_dis[1] / temp);
	relative_dis[2] = desire_velocity*(relative_dis[2] / temp);

	
	double *temp_leader_position = new double[3];
	temp_leader_position[0]=relative_dis[0]*time_step+ uav[Leader_num].read_Position()[0];
	temp_leader_position[1]=relative_dis[1] * time_step +uav[Leader_num].read_Position()[1];
	temp_leader_position[2]=relative_dis[2] * time_step +uav[Leader_num].read_Position()[2];

	relative_dis[0] = Leader_goal_position[0] - temp_leader_position[0];
	relative_dis[1] = Leader_goal_position[1] - temp_leader_position[1];
	relative_dis[2] = 0;

	temp = sqrt(pow(relative_dis[0], 2) + pow(relative_dis[1], 2));
	relative_dis[0] =(relative_dis[0] / temp);
	relative_dis[1] =(relative_dis[1] / temp);
	relative_dis[2] =(relative_dis[2] / temp);

	double line_a, line_b, line_c;
	//line_a = relative_dis[1]; line_b = -relative_dis[0]; c = -(line_a*temp_leader_position[0]+line_b*temp_leader_position[1]);



	double *vector = new double[2];
	vector[0] = Leader_goal_position[0] - uav[Leader_num].read_Position()[0];
	vector[1] = Leader_goal_position[1] - uav[Leader_num].read_Position()[1];
	//vector[2] = Leader_goal_position[2] - uav[Leader_num].read_Position()[2];

}*/
/*
cout << current_position[MY_SOURCE_ID][0] << current_position[MY_SOURCE_ID][1] << endl;
cout << current_velocity[MY_SOURCE_ID][0] << current_velocity[MY_SOURCE_ID][1] << endl;
cout << goal_position[MY_SOURCE_ID][0] << goal_position[MY_SOURCE_ID][1] << endl;
cout << sim->getAgentVelocity(MY_SOURCE_ID - 1).x() << sim->getAgentVelocity(MY_SOURCE_ID - 1).y() << endl;
cout << sim->getAgentPosition(MY_SOURCE_ID - 1).x() << sim->getAgentPosition(MY_SOURCE_ID - 1).y() << endl;
cout << endl;
cout << endl;
current_position[MY_SOURCE_ID][0] += (sim->getAgentVelocity(MY_SOURCE_ID - 1).x() +(rand() % 10-10) /10 )*ORCA_TIME_STEP;
current_position[MY_SOURCE_ID][1] += (sim->getAgentVelocity(MY_SOURCE_ID - 1).y() + (rand() % 10 - 10)/ 10)*ORCA_TIME_STEP;
current_position[MY_SOURCE_ID][2] += (sim->getAgentVelocity(MY_SOURCE_ID - 1).z() + (rand() % 10 - 10)/ 10)*ORCA_TIME_STEP;




renewScenario(sim, current_position, goal_position, current_velocity, AGENT_ACCOUNT - 1);
setPreferredVelocities(sim, static_cast<size_t>(MY_SOURCE_ID));//设置偏好速度
sim->doStep(static_cast<size_t>(MY_SOURCE_ID)); //运行一次orca算法
goal_velocity[MY_SOURCE_ID + 1][0] = sim->getAgentVelocity(MY_SOURCE_ID).x();
goal_velocity[MY_SOURCE_ID + 1][1] = sim->getAgentVelocity(MY_SOURCE_ID).y();
goal_velocity[MY_SOURCE_ID + 1][2] = sim->getAgentVelocity(MY_SOURCE_ID).z();
current_position[MY_SOURCE_ID + 1][0] += (sim->getAgentVelocity(MY_SOURCE_ID).x() + (rand() % 10 - 10) / 10)*ORCA_TIME_STEP;
current_position[MY_SOURCE_ID + 1][1] += (sim->getAgentVelocity(MY_SOURCE_ID).y() +( rand() % 10 - 10) / 10)*ORCA_TIME_STEP;
current_position[MY_SOURCE_ID + 1][2] += (sim->getAgentVelocity(MY_SOURCE_ID).z() +( rand() % 10 - 10) / 10)*ORCA_TIME_STEP;


cout << current_position[MY_SOURCE_ID + 1][0] << current_position[MY_SOURCE_ID + 1][1] << endl;
cout << current_velocity[MY_SOURCE_ID + 1][0] << current_velocity[MY_SOURCE_ID + 1][1]  << endl;
cout << goal_position[MY_SOURCE_ID + 1][0] << goal_position[MY_SOURCE_ID + 1][1] << endl;
cout << sim->getAgentVelocity(MY_SOURCE_ID ).x() << sim->getAgentVelocity(MY_SOURCE_ID ).y()  << endl;
cout << sim->getAgentPosition(MY_SOURCE_ID ).x() << sim->getAgentPosition(MY_SOURCE_ID ).y()  << endl;
cout << endl;
cout << endl;*/

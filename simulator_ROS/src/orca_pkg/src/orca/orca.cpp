#include "orca.h"

std::vector<RVO::Vector3> goals;

void setupScenario(RVO::RVOSimulator *sim, double (**current_position), double (**goal_position), double (**current_velocity), int agent_account,double orca_time_step,double orca_neighbor_dist,double orca_radius,double orca_max_spead)//Different from the original edition, here add velocity message
{
	//double aaa = 0.2;
        /* Specify the global time step of the simulation. */
        sim->setTimeStep(orca_time_step);

        /* Specify the default parameters for agents that are subsequently added. */
        sim->setAgentDefaults(orca_neighbor_dist, agent_account, orca_time_horizon, orca_radius, orca_max_spead);

        /* Add agents, specifying their start position, and store their goals on the opposite side of the environment. */
        for(int i=0;i<agent_account;i++)
        {
          sim->addAgent(RVO::Vector3(current_position[i][0], current_position[i][1], current_position[i][2]), RVO::Vector3(current_velocity[i][0], current_velocity[i][1], current_velocity[i][2]));
          goals.push_back(RVO::Vector3(goal_position[i][0], goal_position[i][1], goal_position[i][2]));
        }
}

void renewScenario(RVO::RVOSimulator *sim, double(**current_position), double(**goal_position), double(**current_velocity), int agent_account)
{
	for(int i=0; i<agent_account; i++)
	{
		sim->setAgentPosition(static_cast<size_t>(i), RVO::Vector3(current_position[i][0], current_position[i][1], current_position[i][2]));
	    sim->setAgentVelocity(static_cast<size_t>(i), RVO::Vector3(current_velocity[i][0], current_velocity[i][1], current_velocity[i][2]));
        goals[static_cast<size_t>(i)] = RVO::Vector3(goal_position[i][0], goal_position[i][1], goal_position[i][2]);
	}
}



void setPreferredVelocities(RVO::RVOSimulator *sim, size_t agent_id)
{
        /* Set the preferred velocity to be a vector of unit magnitude (speed) in the direction of the goal. */
        
//    	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
		
		RVO::Vector3 goalVector = goals[agent_id] - sim->getAgentPosition(agent_id);
		float x = goalVector.x(); float y = goalVector.y(); float z = goalVector.y();
		RVO::Vector3 goalVector2 = RVO::Vector3(x, y, z);
              /*  if (RVO::absSq(goalVector) > 1.0f) {
                        goalVector = RVO::normalize(goalVector);
                }*/

                sim->setAgentPrefVelocity(agent_id, goalVector2);
//        }
}

bool reachedGoal(double *current_position, double *goal_position, double orca_max_error)
{
        /* Check if the agent has reached its goal. */
    double sum1 = 0, sum2 = 0;
	double tmp1 = (goal_position[0] - current_position[0]); //GPS坐标转换为以米为单位
	double tmp2 = (goal_position[1] - current_position[1]);
	double tmp3 = (goal_position[2] - current_position[2]);
	sum1 = tmp1 * tmp1 + tmp2 * tmp2 + tmp3*tmp3;
	//sum2 = 1.0f * orca_radius * orca_radius;
	if(sum1 > orca_max_error)
	{
		return false;
	}
	 return true;
}



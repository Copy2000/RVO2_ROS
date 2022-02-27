#ifndef _ORCAA_H_
#define _ORCAA_H_

#include <cmath>
#include <vector>
#include <cstddef>
#include <fstream>
#include "RVOSimulator.h"

extern double aaa;
/*
double ORCA_TIME_STEP; orca_time_step;
double ORCA_NEIGHBOR_DIST; orca_neighbor_dist;
double ORCA_TIME_HORIZON1; orca_time_horizon;
double ORCA_RADIUS; orca_radius;
double ORCA_MAX_SPEED; orca_max_spead;
double ORCA_MAX_ERROR; orca_max_error;
double ORCA_RAND_PARA; orca_rand_para;
*/
#define orca_time_horizon 1.0f
extern double aa;
#ifndef RVO_OUTPUT_TIME_AND_POSITIONS
#define RVO_OUTPUT_TIME_AND_POSITIONS 1
#endif


extern std::vector<RVO::Vector3> goals;


void setupScenario(RVO::RVOSimulator *sim, double(**current_position), double(**goal_position), double(**current_velocity), int agent_account, double orca_time_step, double orca_neighbor_dist, double orca_radius, double orca_max_spead);//Different from the original edition, here add velocity message

void renewScenario(RVO::RVOSimulator *sim, double(**current_position), double(**goal_position), double(**current_velocity), int agent_account);
//#if RVO_OUTPUT_TIME_AND_POSITIONS

//#endif

void setPreferredVelocities(RVO::RVOSimulator *sim, size_t agent_id);

bool reachedGoal(double*, double*,  double orca_max_error);

#endif

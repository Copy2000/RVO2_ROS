#ifndef UAV_STATE_H
#define UAV_STATE_H
class UAV
{
public:
	int UAV_STATE_FLAG = 0;//0--´ýÃü£¬1--ÔË¶¯ÖÐ¡£
	UAV()
	{
		currentPosition[0] = 0;currentPosition[1] = 0;currentPosition[2] = 0;
		currentVelocity[0] = 0;currentVelocity[1] = 0;currentVelocity[2] = 0;
		goalPosition[0] = 0; goalPosition[1] = 0; goalPosition[2] = 0;
	}
	UAV(double x, double y, double z, double vx, double vy, double vz, double goal_x, double goal_y, double goal_z)
	{
		currentPosition[0] = x;currentPosition[1] = y;currentPosition[2] = z;
		currentVelocity[0] = vx;currentVelocity[1] = vy;currentVelocity[2] = vz;
		goalPosition[0] = goal_x;goalPosition[1] = goal_y;goalPosition[2] = goal_z;
	}
	void setPosition(double x, double y, double z)
	{
		currentPosition[0] = x;currentPosition[1] = y;currentPosition[2] = z;
	}
	void setVelocity(double vx, double vy, double vz)
	{
		currentVelocity[0] = vx;currentVelocity[1] = vy;currentVelocity[2] = vz;
	}
	void setgoalPosition(double goal_x, double goal_y, double goal_z)
	{
		goalPosition[0] = goal_x;goalPosition[1] = goal_y;goalPosition[2] = goal_z;
	}
	double* read_Position()
	{
		return currentPosition;
	}
	double* read_Velocity()
	{
		return currentVelocity;
	}
	double* read_goalPosition()
	{
		return goalPosition;
	}
	double currentPosition[3];
	double currentVelocity[3];
	double goalPosition[3];

};



#endif
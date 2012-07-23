#ifndef __BICLOPS__ACCESS__H__
#define __BICLOPS__ACCESS__H__

#include "Biclops.h"

class BiclopsAccess
{
public:
	static  BiclopsAccess *instance();

	enum Mode 
	{
		OFFLINE = 0,
		INITIALIZED,
		HOMING,
		POSITION,
		PARKED,
	};
	enum Joints 
	{
	 PAN = 0,
	 TILT
	};

	static const int BICLOPS_NUM_JOINTS = 2;

	int getJointCount();

	double getJointLimitMin(int jnt);
	double getJointLimitMax(int jnt);

	double getJointPosition(int jnt);
	double getJointDesiredPosition(int jnt);

	void setJointPosition(int jnt, double val);
	void setJointPosition(double * vals);

	BiclopsAccess::Mode getMode();
	void setMode(Mode mode);

	void park();
	void home();

	void biclopsFSM();
	void biclopsUpdateLoop();

	static void recordLimits();

protected:
	BiclopsAccess();
	virtual ~BiclopsAccess();

	double unitsToRadians(double unit, int jnt);
	double radiansToUnit(double rads, int jnt);

private:
	static BiclopsAccess* _instance;
	static Biclops *_biclops;

	static Mode _mode;
	static Mode _commandedMode;

	static boolean _threadKill;

	static double limits[BICLOPS_NUM_JOINTS][2]; 

	static pthread_t _thread;
	static pthread_t _updateThread;
	static pthread_mutex_t mutex;
	static pthread_mutex_t position_mutex;

	static char configuration_file[256];
	static void setConfigurationFileName();

	static double joint_commands[BICLOPS_NUM_JOINTS];
	static double joint_last_set[BICLOPS_NUM_JOINTS];
};



#endif
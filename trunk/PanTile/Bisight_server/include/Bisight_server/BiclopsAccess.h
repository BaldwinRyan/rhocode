#ifndef __BICLOPS__ACCESS__H__
#define __BICLOPS__ACCESS__H__

#include "Biclops.h"
/*
	#define OFFLINE   0
	#define INITIALIZED 1
	#define HOMING 2
	#define POSITION 3
	#define PARKED 4
	
	#define  PAN 0
	#define  TILT 1
	
*/
/*	enum Mode 
	{OFFLINE_=0,
		INITIALIZED_,
		HOMING_,
		POSITION_,
		PARKED_
	}_mode, _commandedMode;
*/
class BiclopsAccess
{
public:
	static  BiclopsAccess *instance();
	
	enum Mode 
	{OFFLINE=0,
		INITIALIZED,
		HOMING,
		POSITION,
		PARKED
		};
	/*
	en um Joints 
	{
	 PAN = 0,
	 TILT
	};
	*/
	static const int BICLOPS_NUM_JOINTS = 2;

	int getJointCount();

	double getJointLimitMin(int jnt);
	double getJointLimitMax(int jnt);

	double getJointPosition(int jnt);
	double getJointDesiredPosition(int jnt);

	void setJointPosition(int jnt, double val);
	void setJointPosition(double * vals);

	//BiclopsAccess::Mode getMode();
	Mode getMode();
	void setMode(Mode mode);

	void park();
	void home();

	void biclopsFSM();
	void biclopsUpdateLoop();

	static void recordLimits();

public:
	BiclopsAccess();
	virtual ~BiclopsAccess();

	double unitsToRadians(double unit, int jnt);
	double radiansToUnit(double rads, int jnt);

public:
	static BiclopsAccess* _instance;
	static Biclops *_biclops;

	Mode _mode;
	Mode _commandedMode;

	static bool _threadKill;

	static double limits[BICLOPS_NUM_JOINTS][2]; 

	static pthread_t _thread;
	static pthread_t _updateThread;
	static pthread_mutex_t mutex;
	static pthread_mutex_t position_mutex;

	//char *configuration_file;
	static void setConfigurationFileName();

	static double joint_commands[BICLOPS_NUM_JOINTS];
	static double joint_last_set[BICLOPS_NUM_JOINTS];
};



#endif

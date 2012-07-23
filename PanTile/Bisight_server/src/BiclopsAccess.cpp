#include <Bisight_server/BiclopsAccess.h>
#include <PMDUtils.h>
// for debugging:
#include <iostream>
#include <string.h>
#include <ros/ros.h>

#include <DPPanTilt_msgs/SetAbsolutePosition.h>
#include <DPPanTilt_msgs/ReportDynamicInfo.h>
#include <DPPanTilt_msgs/ReportStaticInfo.h>
#include <DPPanTilt_msgs/Reset.h>
#include <DPPanTilt_msgs/HaltMotion.h>
/*
 * GDK, May 2009.
 * Important notes:
 *
 * MUCHO IMPORTANTO: this should be compiled using RELEASE settings. The library file that other projects
 * link to is the in RELEASE directory. This may cause confusion, since BiclopsAccessManaged needs to be
 * compiled using Debug settings to avoid signing name issues. So, be careful to change the compilation
 * mode when switching between these two projects. ALL CODE at or beneath the level of BiclopsAccess
 * should be compiled using RELEASE mode to avoid performance issues. 
 *
 * Other notes:
 *
 * 1) Although the Biclops is supposed to accurately find position 0 after homing, in the code we got
 * it sets its starting position to zero, then finds bounds relative to that. I have included code that
 * works around this by storing the bounds and rescaling inputs and outputs relative to them. I have
 * also used measured min and max degrees for each axis because the data sheets for this Biclops
 * (RevE) are not available.
 *
 * However, if we do suddenly get software that does proper homing, that's OK: the bound computations
 * should still all work. If it needs to be removed, all of that functionality is in functions unitsToRadians,
 * radiansToUnits, and recordLimits. The first two can just be replaced by the corresponding functions in PMDControlAxis,
 * and the last (and associated variable limits) can be removed.
 *
 * 2) Mode control is by a threaded FSM, which does the right thing when it gets a command. This is to make
 * sure that some function calls (e.g., Home) do not block (they would otherwise). 
 *
 * 3) If you change the FSM code, BE CAREFUL: herein lie mutexes. 
 *
 * 4) Similarly, handling incoming joint positions is handed by a separate thread performing an update loop.
 * This has the effect of a zero size buffer: if several joint commands are incoming we ignore all but the last.
 * Not doing this has caused a great deal of trouble before.
 *
 * 5) For some weird reason, if we set the debug level in the configuration file to 0 or 1 things crash, but
 * if we leave it at 2 things do not crash. No idea why. It looks like there's a problem with the COM port
 * stuff at the low debug levels, perhaps the higher ones try some extra parameters. As a workaround,
 * this code programmatically sets the debug level to zero once initialization is complete. This seems
 * to work fine. 
 *
 * 6) The configuration file name is built in setConfigurationFileName. It uses MRDS_DIR environmental
 * variable; if that isn't present, it uses C:\\Microsoft Robotics Dev Studio 2008.
 * If this driver crashes or misbehaves, please check that this file is in the right place. 
 *
 * 7) This code requires libBiclops, libPMD, libUtils, and the pthread library. 
 *
 * 8) If you wish to test this library, use BiclopsTest. It's a little command line program where you can
 * interactively send commands and query status. 
 */

#define PAN_MIN_DEGREES -175.0
#define PAN_MAX_DEGREES +175.0

#define TILT_MIN_DEGREES -60.0
#define TILT_MAX_DEGREES +60.0

// This is the enforced time between updates to the motors. 
// If you set it to zero, it will update as fast as it can 
// (using only the last message). If movement gets choppy,
// increase it. 
#define BICLOPS_UPDATE_SLEEP_TIME_MS 0

// Static class variables; these must be redeclared here
// to avoid linker symbol errors.
BiclopsAccess * BiclopsAccess::_instance = NULL;
Biclops * BiclopsAccess::_biclops = NULL;
bool BiclopsAccess::_threadKill = false;

double BiclopsAccess::limits[BICLOPS_NUM_JOINTS][2]; 
//Mode _mode;
//Mode _commandedMode;
//BiclopsAccess::Mode _mode;
//BiclopsAccess::Mode _commandedMode;

pthread_t BiclopsAccess::_thread;
pthread_t BiclopsAccess::_updateThread;
pthread_mutex_t BiclopsAccess::mutex;
pthread_mutex_t BiclopsAccess::position_mutex;

double BiclopsAccess::joint_commands[BICLOPS_NUM_JOINTS];
double BiclopsAccess::joint_last_set[BICLOPS_NUM_JOINTS];

char *configuration_file="/home/noam/ros_workspace/PanTilt/Bisight_server/BiclopsDefault.cfg";
// Static variable declaration done. 


void BiclopsAccess::setConfigurationFileName()
{/*
	if(GetEnvironmentVariableA("MRDS_DIR", configuration_file, 256) == 0)
	{
	  //strcpy_s(configuration_file, 256, "C:\\Microsoft Robotics Dev Studio 2008");
	}

	//strcat_s(configuration_file, 256, "\\store\\config\\BiclopsDefault.cfg");
	*/
  //configuration_file="/home/noam/ros_workspace/PanTilt/Bisight_server/BiclopsDefault.cfg";
}

void *biclopsFSMWrapper(void *args)
{
	BiclopsAccess::instance()->biclopsFSM();
	
	pthread_exit(NULL);
	return NULL;
}

void *biclopsUpdateLoopWrapper(void *args)
{
	BiclopsAccess::instance()->biclopsUpdateLoop();

	pthread_exit(NULL);
	return NULL;
}

void BiclopsAccess::biclopsFSM()
{
	while(!_threadKill)
	{
		// If we're offline, try to do the initialization.
		if(_mode == OFFLINE)
		{
			_biclops->SetDebugLevel(2);
			ROS_INFO("File name : %s",configuration_file);
			if(_biclops->Initialize(configuration_file))
			{
			  ROS_WARN("Initializing\n");
				pthread_mutex_lock(&mutex);
				_mode = INITIALIZED;
				_commandedMode = INITIALIZED;
				pthread_mutex_unlock(&mutex);
				_biclops->SetDebugLevel(0);
			}
			ROS_WARN(" Biclops Initialized");
			// Clear commands sent during initialization.
			_commandedMode = _mode;
			ROS_WARN("Mode : %d",_mode);
		}

		// If we're initialized, we can be asked to home or park.
		else if(_mode == INITIALIZED)
		{
		  home();
		  ROS_WARN("Commanded Mode : %d",_commandedMode);
			if(_commandedMode == HOMING)
			{
				// Set mode to homing and do the business.
				pthread_mutex_lock(&mutex);
				_mode = HOMING;
				_commandedMode = HOMING;
				pthread_mutex_unlock(&mutex);
				
				if(_biclops->HomeAxes(_biclops->AllAxesMask, true))
				{
					// Record limits
					_instance->recordLimits();

					// Set mode to "position mode", and erase previous commands. 
					pthread_mutex_lock(&mutex);
					_mode = POSITION;
					_commandedMode = POSITION;
					pthread_mutex_unlock(&mutex);
				}
				else
				{
					// Homing failed, bad biclops!
					pthread_mutex_lock(&mutex);
					_mode = INITIALIZED;
					_commandedMode = INITIALIZED;
					pthread_mutex_unlock(&mutex);
				}
			}
			else if(_commandedMode == PARKED)
			{
				// Park each motor and set mode.
				for(int j = 0; j < BICLOPS_NUM_JOINTS; j++)
				{
					_biclops->GetAxis(j)->Park();
				}

				pthread_mutex_lock(&mutex);
				_mode = PARKED;
				_commandedMode = PARKED;
				pthread_mutex_unlock(&mutex);
			}
			ROS_WARN("Mode after Homing : %d",_mode);
		}
	
		else if(_mode == POSITION)
		{
			// If we're in position mode we can be asked to home or park.
			if(_commandedMode == HOMING)
			{
				// Set mode to homing and do the business.
				pthread_mutex_lock(&mutex);
				_mode = HOMING;
				_commandedMode = HOMING;
				pthread_mutex_unlock(&mutex);
				
				if(_biclops->HomeAxes(_biclops->AllAxesMask, true))
				{
					// Record homing limits
					_instance->recordLimits();


					// Set mode to "position mode", and erase previous commands. 
					pthread_mutex_lock(&mutex);
					_mode =POSITION;
					_commandedMode = POSITION;
					pthread_mutex_unlock(&mutex);
				}
				else
				{
					// Homing failed, bad biclops!
					pthread_mutex_lock(&mutex);
					_mode = INITIALIZED;
					_commandedMode = INITIALIZED;
					pthread_mutex_unlock(&mutex);
				}
			}
			else if(_commandedMode == PARKED)
			{
				// Park each motor and set mode.
				for(int j = 0; j < BICLOPS_NUM_JOINTS; j++)
				{
					_biclops->GetAxis(j)->Park();
				}

				pthread_mutex_lock(&mutex);
				_mode = PARKED;
				_commandedMode = PARKED;
				pthread_mutex_unlock(&mutex);
			}
		}

		else if(_mode == PARKED)
		{
			// If we are parked, we can be asked to home.
			if(_commandedMode == HOMING)
			{
				// Set mode to homing and do the business.
				pthread_mutex_lock(&mutex);
				_mode = HOMING;
				_commandedMode = HOMING;
				pthread_mutex_unlock(&mutex);
				
				if(_biclops->HomeAxes(_biclops->AllAxesMask, true))
				{
					// Set mode to "position mode", and erase previous commands. 
					pthread_mutex_lock(&mutex);
					_mode = POSITION;
					_commandedMode = POSITION;
					pthread_mutex_unlock(&mutex);
				}
				else
				{
					// Homing failed, bad biclops!
					pthread_mutex_lock(&mutex);
					_mode = INITIALIZED;
					_commandedMode = INITIALIZED;
					pthread_mutex_unlock(&mutex);
				}
			}
		}

		usleep(20*1000);
	}
}

void BiclopsAccess::biclopsUpdateLoop()
{
	bool updated = false;

	//cout << "Firing up Biclops update thread." << endl;
// Publish Position & Speed
     
      
	while(!_threadKill)
	{

		for(int jnt = 0; jnt < BICLOPS_NUM_JOINTS; jnt++)
		{
			if(joint_commands[jnt] != joint_last_set[jnt])
			{
				pthread_mutex_lock(&position_mutex);
				double val = joint_commands[jnt];
				joint_last_set[jnt] = val;
				updated = true;
				pthread_mutex_unlock(&position_mutex);

				double unit = radiansToUnit(val, jnt);
	
				PMDAxisControl::Profile p;
				PMDAxisControl *axis = _biclops->GetAxis(jnt);

				axis->SetPosition(axis->UnitsToCounts(unit));
				axis->GetProfile(p);
				p.pos = unit;
				axis->SetProfile(p);

				axis->Move(false, true);
			}

	       
		}

	
		if(updated)
		{
			usleep(BICLOPS_UPDATE_SLEEP_TIME_MS*1000);
			updated = false;
		}
		else
		{
			usleep(0);
		}
	};
}

BiclopsAccess::BiclopsAccess()
{
	_threadKill = false;
	_biclops = new Biclops();

	_mode = OFFLINE;

	setConfigurationFileName();

	for(int j = 0; j < BICLOPS_NUM_JOINTS; j++)
	{
		joint_commands[j] = -1000;
		joint_last_set[j] = -1000;
	}

	pthread_mutex_init(&mutex, NULL);
	pthread_mutex_init(&position_mutex, NULL);
	pthread_create(&_thread, NULL, biclopsFSMWrapper, (void*)NULL);	
	pthread_create(&_updateThread, NULL, biclopsUpdateLoopWrapper, (void*)NULL);

	
  }

BiclopsAccess::~BiclopsAccess()
{
	_threadKill = true;
	pthread_join(_thread, NULL);
	pthread_join(_updateThread, NULL);
	pthread_mutex_destroy(&mutex);
	pthread_mutex_destroy(&position_mutex);

	delete _biclops;
}

BiclopsAccess* BiclopsAccess::instance()
{
	if(!_instance)	
	{
		_instance = new BiclopsAccess();
	}
	return _instance;
}


int BiclopsAccess::getJointCount()
{
	return BICLOPS_NUM_JOINTS;
}

double BiclopsAccess::getJointLimitMin(int jnt)
{
	if(jnt == _biclops->Pan)
	{
		return PMDUtils::DegsToRads(PAN_MIN_DEGREES);
	}
	else if(jnt == _biclops->Tilt)
	{
		return PMDUtils::DegsToRads(TILT_MIN_DEGREES);
	}
	else
	{
		return 0;
	}
}

double BiclopsAccess::getJointLimitMax(int jnt)
{
	if(jnt == _biclops->Pan)
	{
		return PMDUtils::DegsToRads(PAN_MAX_DEGREES);
	}
	else if(jnt == _biclops->Tilt)
	{
		return PMDUtils::DegsToRads(TILT_MAX_DEGREES);
	}
	else
	{
		return 0;
	}
}

double BiclopsAccess::getJointPosition(int jnt)
{
	// Can only ask during HOMING, POSITION and PARKED. 
	if(_mode <= HOMING) return 0.0;

	PMDint32 d; 
	PMDAxisControl *axis = _biclops->GetAxis(jnt);

	axis->GetActualPosition(d);
	
	double p = axis->CountsToUnits(d);

/*
	if (jnt == 0) {
	  std::cout<<"Inside joint position1 :"<<(int) d<< " --> " << p << " cpUnit: " << axis->GetCountsPerEncoderCycle() << std::endl;
	}
*/
	p = unitsToRadians(p, jnt);	

	return p;
}

double BiclopsAccess::getJointDesiredPosition(int jnt)
{
	// Can only ask during HOMING, POSITION and PARKED. 
	if(_mode <= HOMING) return 0.0;

	PMDint32 d; 
	PMDAxisControl *axis = _biclops->GetAxis(jnt);

	axis->GetCommandedPosition(d);
	double p = axis->CountsToUnits(d);
	p = unitsToRadians(p, jnt);

	return p;
}

void BiclopsAccess::setJointPosition(int jnt, double val)
{

	// Can only ask during POSITION
	if(_mode != POSITION) return;

	pthread_mutex_lock(&position_mutex);
	joint_commands[jnt] = val;
	pthread_mutex_unlock(&position_mutex);
}

void BiclopsAccess::setJointPosition(double * vals)
{
	// Can only ask during POSITION
	if(_mode != POSITION) return;

	pthread_mutex_lock(&position_mutex);

	for(int j = 0; j < BICLOPS_NUM_JOINTS; j++)
	{
		joint_commands[j] = vals[j];
	}

	pthread_mutex_unlock(&position_mutex);
}

BiclopsAccess::Mode BiclopsAccess::getMode()
{
	return _mode;
}

void BiclopsAccess::setMode(Mode cmode)
{
	// Lock so that updates are atomic. 
	pthread_mutex_lock(&mutex);

	// Sanity checking
	bool valid = true;

	if(_mode == OFFLINE) valid = false;
	if((_mode == INITIALIZED) && (cmode != HOMING)) valid = false;
	if(_mode == HOMING) valid = false;
	if((_mode == PARKED)&& (cmode != HOMING)) valid = false;

	if(valid)
		_commandedMode = cmode;

	pthread_mutex_unlock(&mutex);
}

void BiclopsAccess::park()
{
	setMode(PARKED);
}

void BiclopsAccess::home()
{
	setMode(HOMING);
}


void BiclopsAccess::recordLimits()
{
	for(int j = 0; j < BICLOPS_NUM_JOINTS; j++)
	{
		long hlf = 0, hlr = 0, slf = 0, slr = 0;  
		PMDAxisControl *axis = _biclops->GetAxis(j);
	    axis->GetHomingLimits(hlf, hlr, slf, slr);

		limits[j][0] = axis->CountsToUnits(hlr);
		limits[j][1] = axis->CountsToUnits(hlf);
	}
}

double BiclopsAccess::unitsToRadians(double unit, int jnt)
{
	double homedelta  = limits[jnt][1] - limits[jnt][0];
	double pointdelta = unit - limits[jnt][0];
	double propr = pointdelta / homedelta;

	double maxdeg = getJointLimitMax(jnt);
	double mindeg = getJointLimitMin(jnt);

	//	ROS_INFO("Units : %f, Joint Limit %d : %f, %f --> %f", unit, jnt, mindeg, maxdeg, mindeg + propr*(maxdeg - mindeg));
	return mindeg + propr*(maxdeg - mindeg);
}

double BiclopsAccess::radiansToUnit(double rads, int jnt)
{
	double maxdeg = getJointLimitMax(jnt);
	double mindeg = getJointLimitMin(jnt); 

	double homedelta = maxdeg - mindeg;
	double rdelta = rads - mindeg;
	double propr = rdelta / homedelta;

	return limits[jnt][0] + propr*(limits[jnt][1] - limits[jnt][0]);
}

 


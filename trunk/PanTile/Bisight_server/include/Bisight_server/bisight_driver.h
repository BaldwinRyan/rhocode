#ifndef _BISIGHT_DRIVER_H_
#define _BISIGHT_DRIVER_H_

#include <termios.h>


extern "C" {
#include <Bisight_server/comport.h>
}



// serial defines
//#define BISIGHT_DEFAULT_BAUD 38400
#define BISIGHT_BUFFER_LEN 255
#define BISIGHT_DEFAULT_PORT "/dev/ttyS0" //"/dev/ttyUSB0"
#define BISIGHT_DEFAULT_HZ 25
#define BISIGHT_DEFAULT_STATIC_PUBLISH_RATE 1.0
#define BISIGHT_DEFAULT_PARK_ON_EXIT 0

// command defines
/*
#define PTU46_PAN 'p'
#define PTU46_TILT 't'
#define PTU46_MIN 'n'
#define PTU46_MAX 'x'
#define PTU46_MIN_SPEED 'l'
#define PTU46_MAX_SPEED 'u'
#define PTU46_VELOCITY 'v'
#define PTU46_POSITION 'i'
*/

/*
  We need a set of joint limit constants - These are in Radians, 
  They should be multiplied by 1000 before being applied to data sent by 
  bisightdriver to the Kameleon.
 */
#define PAN_MIN          -2.0
#define PAN_MAX           2.0
#define TILT_MIN         -0.28 
#define TILT_MAX          1.1
#define VLEFT_MIN        -0.6 //-0.6
#define VLEFT_MAX         0.1 
#define VRIGHT_MIN       -0.1
#define VRIGHT_MAX        0.6 //0.6

// encoder to radient conversion constant
#define RAD_TO_ENC 1000

#define PAN 0
#define TILT 1
#define VERGE_LEFT 2
#define VERGE_RIGHT 3

#define HOMED 1
#define PARKED 0

/**
 * \brief PTU46 Pan Tilt Unit Driver
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 * $Id: ptu46.cc 7798 2009-06-06 09:00:59Z thjc $
 *
 * Author: Toby Collett (University of Auckland)
 * Date: 2003-02-10
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
class Bisight {
    public:

        /** Constructor - opens port
         * \param port Filename where PTU is connected
         * \param rate rate Baud rate to use */
        Bisight(const char* port);
        ~Bisight();

        /** \return true if the ptu is open/ready */
        bool isOpen () {
            return (((int)(Port.Fd)) > 0);
        };

	//float GetPosition (char type);
	int getJointPosition(int jnt, double &retValue);
	int getJointPositions(double *value);
	int setJointPosition(int jnt, double val);
	int setJointPositions(double *val);
	
	int park();
	
	int home();
	int getHomeValue();
	double getJointLimitMin(int jnt);
	double getJointLimitMax(int jnt);
	void emptyInputBuffer();
	int OKtransmitions;
	int FAILEDtransmitions;
	int home_value;
	

    protected:

	
        //int sendReceive(char *data, char *reply);
	int sendReceiveLoop(char *Buffer, char *Reply);
	int sendReceiveRepeatLoop(char *Buffer, char *Reply, int i);        
	int sendCommand(char *Buffer);
	int receiveReply(char *Buffer, char *Reply);
       
        void Disconnect();
	void parseReply(char *reply, int *data);
	void createCommand(char *Buffer, int *a, int b);
	void addChecksum(char *cmd, char *finalCmd);

	

	double limit_min[4];
	double limit_max[4];


        char buffer[BISIGHT_BUFFER_LEN+1]; 	///< read buffer
	int fd;					///< serial port descriptor
	struct termios oldtio; ///< old terminal settings
       
	double pan_position;
	double tilt_position;
	double pan_velocity;
	double tilt_velocity;

	int mode;

	//comport stuff
	ComPort_t   Port;
	
	

};



#endif

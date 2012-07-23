/*
 * Bisight ROS Package
 * Copyright (C) 2010 Dirk ruiken
 *
 */

// class declaration
#include <bisight/bisight_driver.h>

// serial includes
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <ros/ros.h>


//
// Pan-Tilt Control Class
//

// Constructor opens the serial port, and read the config info from it
Bisight::Bisight(const char * port) {
   fd = -1;
    // open the serial port

    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if ( fd<0 ) {
        fprintf(stderr, "Could not open serial device %s\n",port);
        return;
    }
    fcntl(fd,F_SETFL, 0);

    // save the current io settings
    tcgetattr(fd, &oldtio);
    
    // rtv - CBAUD is pre-POSIX and doesn't exist on OS X
    // should replace this with ispeed and ospeed instead.

    // set up new settings
    struct termios newtio;
    memset(&newtio, 0,sizeof(newtio));
    newtio.c_cflag = /*(rate & CBAUD) |*/ CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    //newtio.c_lflag = ICANON;
    //set to non- canonical input
    newtio.c_lflag = 0;
    //receive at least 1 character
    newtio.c_cc[VMIN] = 0;
    //timeout after VALUE x 0.1s after character
    newtio.c_cc[VTIME] = 5;

   
    int rate = 38400;
    speed_t sp = B38400;
    
    if (cfsetispeed(&newtio, sp) < 0 ||   cfsetospeed(&newtio, sp) < 0) {
        fprintf(stderr,"Failed to set serial baud rate: %d\n", rate);
        Disconnect();
        return;
    }


    // activate new settings
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

   
    // now set up the pan tilt camera
    //Write(" "); // terse feedback
    usleep(100000);
    tcflush(fd, TCIFLUSH);

    //fill limit_min and limit_max
    limit_min[PAN] 		= PAN_MIN;
    limit_max[PAN] 		= PAN_MAX;
    limit_min[TILT] 		= TILT_MIN;
    limit_max[TILT] 		= TILT_MAX;
    limit_min[VERGE_LEFT] 	= VLEFT_MIN;
    limit_max[VERGE_LEFT] 	= VLEFT_MAX;
    limit_min[VERGE_RIGHT] 	= VRIGHT_MIN;
    limit_max[VERGE_RIGHT] 	= VRIGHT_MAX;

    //homing needed?
    if (getHomeValue() == 0) {
	printf("Homing Bisight... \n");
	home(); 
	while (getHomeValue() == 0) {
		printf(".");
		usleep(100000);
	}	
	printf("\n");

    } else {
	printf("Bisight already homed.");	
    }
    mode = HOMED;

}


Bisight::~Bisight() {
    Disconnect();
}

void Bisight::Disconnect() {
    if (fd > 0) {
        // restore old port settings
        tcsetattr(fd, TCSANOW, &oldtio);
        // close the connection
        close(fd);
        fd = -1;
    }
}

void Bisight::parseReply(char *reply, int *data){
	
	int i=1;
	int j=0;
	
	//parse the string into some integers
	for (j=0;j<4;j++){
		
		data[j] = 0;
		if (reply[i] == '\r')
			data[j] = 0;
		else
			data[j] = atoi(&reply[i]);
		
		while (reply[i] != ' ' && reply[i] != '\r') i++;

		if (reply[i] == ' ') i++;
	}

	return;
}

int Bisight::sendReceive(char *data, char *reply) {

	//check serial port
    	if (fd < 0)
        	return 0;

	//add checksum 
	char data2[32];
	addChecksum(data, data2);

    	//autocalculate length
    	int length = strlen(data2);
	
    	// ugly error handling, if write fails then shut down unit
	//send command
   	if (write(fd, data2, length) < length) {
    	    	fprintf(stderr,"Error writing to Pan Tilt Unit, disabling\n");
      	 	Disconnect();
       	 	return 0;
    	}

	//wait for reply
	int len = read (fd, buffer, BISIGHT_BUFFER_LEN );
	//ROS_INFO("Message received: %s, length: %d", buffer, len);
   	if (len < 1) {
        	fprintf(stderr,"Error getting pan-tilt pos\n");
        	return 0;
	}

	//check if error was returned
	if (buffer[0] == 'E')
		fprintf(stderr,"Error response received: %c%c \n", buffer[0], buffer[1]);

    	buffer[len] = '\0';

	//return reply
	sprintf(reply, "%s", buffer);

	//ROS_INFO("write 3: done");
	
    	return 1;
}

double Bisight::getJointPosition(int jnt) {

	char cmd[4] = " ";

	//ROS_INFO("Requesting pos for joint: %d", jnt);    

	switch(jnt) {
		case PAN:
			cmd[0] = 'p'; 
			break;
		case TILT:
			cmd[0] = 't';
			break;
		case VERGE_LEFT:
			cmd[0] = 'l';
			break;
		case VERGE_RIGHT:
			cmd[0] = 'r';
			break;
		default:
			return 0;
			break;
	}

	//ROS_INFO("Command: %s", cmd);  
	
	sendReceive(cmd, buffer);
	
	int data[4];
	parseReply(buffer, data);

	return ((double)data[0] / (double)RAD_TO_ENC);
}

void Bisight::getJointPositions(double *value) {
	
	char cmd[32];
	
	sprintf(cmd, "a");

	sendReceive(cmd, buffer);
	int data[4];
	parseReply(buffer, data);
	
	int i;
	for (i=0; i<4; i++) {
		value[i] = (double)data[i] / (double)RAD_TO_ENC;
	}

}



/**
 * Sets the desired joint position
 * @param jnt the joint number
 * @param value the position in radians
 */
void Bisight::setJointPosition(int jnt, double val) {
	char cmd[32];
	int value[1];
    
	//clamping of position commands to joint limits
	
	if (val > limit_max[jnt])
		val = limit_max[jnt];
	if (val < limit_min[jnt])
		val = limit_min[jnt];

	value[0] = (int)(val * (double)RAD_TO_ENC);			   

	switch(jnt) {
		case PAN:
			sprintf(cmd, "P");
			createCommand(cmd, value, 1);
			
			break;
		case TILT:
			sprintf(cmd, "T");
			createCommand(cmd, value, 1);
			
			break;
		case VERGE_LEFT:			
			sprintf(cmd, "L");
			createCommand(cmd, value, 1);
			
			break;
		case VERGE_RIGHT:
			sprintf(cmd, "R");
			createCommand(cmd, value, 1);

			break;
		default:
			break;
	}
	
	sendReceive(cmd, buffer);
	
	//int data[4];	
   	//parseReply(buffer, data);

}

/**
 * Sets the desired joint position
 * @param jnt the joint number
 * @param value the position in radians
 */
void Bisight::setJointPositions(double *val) {
	int i;
	for (i=0; i<4; i++) {	
		if (val[i] > limit_max[i])
			val[i] = limit_max[i];
		if (val[i] < limit_min[i])
			val[i] = limit_min[i];
	}

	char cmd[32];
	int value[4];
    
	value[0] = (int)(val[0] * (double)RAD_TO_ENC);			   
	value[1] = (int)(val[1] * (double)RAD_TO_ENC);
	value[2] = 0;
	value[3] = 0;

	sprintf(cmd, "A");
	createCommand(cmd, value, 4);

	sendReceive(cmd, buffer);
	//int data[4];
	//parseReply(buffer, data);
}






/**
 * @param jnt the joint of interest
 * @return the minimum joint limit in radians
 */
double Bisight::getJointLimitMin(int jnt) {
	if (jnt < 0 || jnt > 3)
		return 0;

	return limit_min[jnt];
}

/**
 * @param jnt the joint of interest
 * @return the max joint limit
 */
double Bisight::getJointLimitMax(int jnt) {
	if (jnt < 0 || jnt > 3)
		return 0;

	return limit_max[jnt];
}


void Bisight::createCommand(char *Buffer, int *a, int b){
	char str[32];
	if(b == 0)
		return;
	if(b == 1){
		sprintf(str, "%d", a[0]);
		strcat(Buffer, str);
		return;
	}
	if(b==2){
		sprintf(str, "%d %d", a[0], a[1]);
		strcat(Buffer, str);
		return;
	}
	if(b==3){
		sprintf(str, "%d %d %d", a[0], a[1], a[2]);
		strcat(Buffer, str);
		return;
	}
	if(b==4){
		sprintf(str, "%d %d %d %d", a[0], a[1], a[2], a[3]);
		strcat(Buffer, str);
		return;
	}
}

void Bisight::addChecksum(char *cmd, char *finalCmd){
		
	int sum, i;
	char cks;
	char tmp[32], tmp2[32];
	sprintf(tmp, "%s",  cmd);
	
	//checksum area
	strcat(tmp, " ");
	i = 0;
	sum = 0;
	
   	//calc checksum
	while(tmp[i] != 0x00) {
		sum+= tmp[i++];
	}
	cks = (char)sum;

	sprintf(tmp2, "%c", cks);

	//concatenate command, checksum
	strcat(tmp, tmp2);
    	
	//add return to command + checksum
	sprintf(finalCmd, "%s\r", tmp);
    
}

void Bisight::park() {
	char cmd[4] = "Q"; 
	
	if(getHomeValue() == 1)	{

		mode = PARKED;
		printf("Parking Bisight...\n");
		sendReceive(cmd, buffer);
		
	        printf("Reply from Park = %s\n", buffer);
		//int data[4];
    		//parseReply(buffer, data);

	}
	else{
		printf("Bisight needs to be homed before it can be parked!");
	}
}

void Bisight::home() {
	
	printf("Homing Bisight");
	
	char cmd[4] = "H";
	sendReceive(cmd, buffer);
	
	printf("Reply from Home = %s\n", buffer);
	//int data[4];
	//parseReply(Reply, data);
}

int Bisight::getHomeValue() {
  	char cmd[4] = "h";

	if (sendReceive(cmd, buffer)) {
		int data[4];
   		parseReply(buffer, data);		
		return data[0];
	}
	return -1;  
}




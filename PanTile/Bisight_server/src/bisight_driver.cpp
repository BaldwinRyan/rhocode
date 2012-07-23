/*
 * Bisight ROS Package
 * Copyright (C) 2010 Dirk ruiken
 *
 */

// class declaration
#include <Bisight_server/bisight_driver.h>

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
Bisight::Bisight(const char * port_name) {   

    ComConstruct(&Port);
    
    ROS_INFO("Opening serial port %s\n", port_name);
    if (ComOpen(&Port, port_name) != E_OKAY) {
        fprintf(stderr, "Error opening serial communication port \n" );
    }

    if (ComFlush(&Port, TCIOFLUSH) != E_OKAY) {
        fprintf(stderr, "Error flushing serial port \n");
    }

    OKtransmitions = 0;
    FAILEDtransmitions = 0;


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
	ROS_INFO("Homing Bisight... ");
	home(); 
	//wait some time for home() to finish
	usleep(25000000);
	home_value = true;
    } else {
	ROS_INFO("Bisight already homed.");	
    }
    mode = HOMED;

}


Bisight::~Bisight() {
    //Disconnect();
}

void Bisight::Disconnect() {
    /*if (fd > 0) {
        // restore old port settings
        tcsetattr(fd, TCSANOW, &oldtio);
        // close the connection
        close(fd);
        fd = -1;
    }*/
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
}

int Bisight::sendReceiveLoop(char *Buffer, char *Reply) {
  if(sendCommand(Buffer) == E_OKAY) {
    //    printf("SendReceiveLoop OKAY\n");
    if(receiveReply(Buffer, Reply) == E_OKAY) {
      return 1;
    }
  }
  // printf("sendRecieveLoop Failed.  Trying again. \n");
  return sendReceiveRepeatLoop(Buffer, Reply, 0);
}


int Bisight::sendReceiveRepeatLoop(char *Buffer, char *Reply, int i) {
  if(sendCommand(Buffer) == E_OKAY) {
    if(receiveReply(Buffer, Reply) == E_OKAY) {
      return 1;
    }
  }
  if(i<2){
    i++;
    return sendReceiveRepeatLoop(Buffer, Reply, i);
  } else {
    //printf("sendReceiveLoop: Failed too many times. %c\n", Buffer[0]);
    //usleep(2000000);
    return 0;
  }
}



/*
int Bisight::sendReceive(char *data, char *reply) {

	//check serial port
    	if (fd < 0)
        	return 0;

	//add checksum 
	char data2[32];
	addChecksum(data, data2);

    	//autocalculate length
    	int length = strlen(data2);
	
	//send command - at most 
	bool resend;
	int i;
	for (i=0; i<2; i++) {
		resend = false;

	    	// ugly error handling, if write fails then shut down unit
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
			resend = true;			
			//return 0;
		}

		//check if error was returned
		if (buffer[0] == 'E') {
			fprintf(stderr,"Error response received: %c%c \n", buffer[0], buffer[1]);		
			resend = true;
		}

	    	buffer[len] = '\0';
	
		if (resend) {
			//empty receive buffer
			len = read (fd, buffer, BISIGHT_BUFFER_LEN );
  		}
		if (resend && i==1) {
			fprintf(stderr, "Repeated receive error.\n");
			return 0;
		}
		if (!resend)
			break;

		
	}

	//return reply
	sprintf(reply, "%s", buffer);

	//ROS_INFO("write 3: done");
	
    	return 1;
}
*/


int Bisight::getJointPosition(int jnt, double &retValue) {

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
	
	if (sendReceiveLoop(cmd, buffer)) {
		int data[4];
		parseReply(buffer, data);
		retValue = (double)data[0] / (double)RAD_TO_ENC;
		return true;
	}

	return false;
}

int Bisight::getJointPositions(double *value) {
	
	char cmd[32];
	
	sprintf(cmd, "a");

	if (sendReceiveLoop(cmd, buffer))
	{
		int data[4];
		parseReply(buffer, data);
		
		int i;
		for (i=0; i<4; i++) {
			value[i] = (double)data[i] / (double)RAD_TO_ENC;
		}
		return 1;
	}
	return 0;
}


int Bisight::setJointPosition(int jnt, double val) {
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
	
	if(sendReceiveLoop(cmd, buffer)) {
		int data[4];
      		parseReply(buffer, data);
      		return data[0];
   	}	

	return false;		
}

/**
 * Sets the desired joint position
 * @param jnt the joint number
 * @param value the position in radians
 */
int Bisight::setJointPositions(double *val) {
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

	if(sendReceiveLoop(cmd, buffer)) {
	      int data[4];
	      parseReply(buffer, data);
	      
	      return data[0];
	}

	return false;
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

int Bisight::sendCommand(char *Buffer) {
  int Stat, sum, i;
  char cks;
  char tmp[32], tmp2[32];
  sprintf(tmp, "%s", Buffer);
  //First we need to add a checksum:
  strcat(tmp," ");
  i = 0;
  sum = 0;
  //calc checksum
  while(tmp[i] != 0x00)
    sum += tmp[i++];
  cks = (char)sum;
  //printf("Checksum is %c  \n", cks);
  //printf("Sum is %d\n", sum);
  //append checksum and \r
  sprintf(tmp2, "%c", cks);
  strcat(tmp, tmp2);
  //if you want to see what we're sending, print it here:
  //printf("Sending %s.\n", tmp);
  sprintf(tmp2, "%s\r", tmp);
 

  if (ComFlush(&Port, TCIOFLUSH) != E_OKAY) {
    fprintf(stderr, "Error flushing serial port \n");
  }

  //So now the string we want to send to the kameleon/bisight is in "tmp2"
  Stat = ComWrite(&Port, tmp2, -1);
  
  return(Stat);
}

int Bisight::receiveReply(char *Buffer, char *Reply) {
  int Stat;

  //printf("reading com port...\n");
  Stat = ComRead(&Port, Reply, -64, NULL, 100);
  //printf("reply is %s\n", Reply);
  if(Stat == E_OKAY) {
    //check to make sure reply matches command buffer
    if(strncmp(Buffer, Reply, 1)!=0) {
      Stat = E_INVALID;
      //printf("invalid reply: Buff[0] = %c, Reply[0] = %c.\n", Buffer[0], Reply[0]);
      FAILEDtransmitions++;
      if(strncmp("E", Reply, 1) == 0) {
	//printf("Got an Error message back: %s \n", Reply);
	if(strncmp("E2", Reply, 2) == 0) {
	  printf("E2: Repeating receiveReply \n");
	  receiveReply(Buffer, Reply);
	}
      }
      if (ComFlush(&Port, TCIOFLUSH) != E_OKAY) {
     	fprintf(stderr, "Error flushing serial port \n");
      }
    }
    else{ OKtransmitions ++;}
  } else {
    //    printf("comport Read Stat is %d. \n", Stat);
    FAILEDtransmitions ++;
    //if (Stat == E_TIMEOUT) {
      //printf("Sleeping\n");
      //sleep(1);
    //}
    if (ComFlush(&Port, TCIOFLUSH) != E_OKAY) {
      fprintf(stderr, "Error flushing serial port \n");
    }
  } 
  return Stat;  
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

int Bisight::park() {
  char Buf[32], Reply[32];
  int data[4];
  int Stat;

  printf("parking...");
  if(getHomeValue()==1) {
    
    sprintf(Buf, "Q");
    createCommand(Buf, NULL, 0);
    
    if(sendCommand(Buf) == E_OKAY) {
      home_value = 0;  
      Stat = ComRead(&Port, Reply, -64, NULL, 200000);
      //printf("reply is %s\n", Reply);
      if(Stat == E_OKAY) {
	//check to make sure reply matches command buffer
	if(strncmp(Buf, Reply, 1)!=0) {
	  Stat = E_INVALID;
	  printf("invalid reply: Buff[0] = %c, Reply[0] = %c.\n", Buf[0], Reply[0]);
	  if (ComFlush(&Port, TCIOFLUSH) != E_OKAY) {
	    fprintf(stderr, "Error flushing serial port \n");
	  }
	}
      }
      // sendReceiveLoop(Buf, Reply);
      parseReply(Reply, data);
      return data[0];
    }
    else { return -1;}
  }
  else {return 1;}
  return -1;
}

int Bisight::home() {
	  char cmd[4] = "H";
	  int Stat;
	  
	  printf("Homing... \n");
	  
	  if(sendCommand(cmd) == E_OKAY) {
	    		Stat = ComRead(&Port, buffer, -64, NULL, 200000);
			//printf("reply is %s\n", buffer);
			if(Stat == E_OKAY) {
				//check to make sure reply matches command buffer
				if(strncmp(cmd, buffer, 1)!=0) {
					Stat = E_INVALID;
					printf("invalid reply: Buff[0] = %c, Reply[0] = %c.\n", cmd[0], buffer[0]);
					if (ComFlush(&Port, TCIOFLUSH) != E_OKAY) {
						fprintf(stderr, "Error flushing serial port \n");
					}
				}
			}
		  
		    	//printf("return from homing\n");
			//sendReceiveLoop(cmd, buffer);
			int data[4];  
			parseReply(buffer, data);
			return data[0];
	  }
	  return -1;  
}

int Bisight::getHomeValue() {
	  char cmd[4] = "h";

	  if(sendReceiveLoop(cmd, buffer)){
		int data[4];
		parseReply(buffer, data);
		home_value = data[0];
		return data[0];
	  }
	  else return -1;
}









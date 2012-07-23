/****************************************************************************
 * comport.c                                                                *
 ****************************************************************************
 * 000707 - DWheeler - Converted from C++ to C.                             *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include "Bisight_server/comport.h"


#include <sys/signal.h>
#include <term.h>

#define TCOONHW 0x0005



//static void AlrmTrap(int Signal) {
//}

/* Start the timeout timer (except stops the timer if mSec == 0).  If the   *
 * timer times out, a SIGALRM is generated, and an interrupted system call  *
 * will (typically) return EINTR.                                           *
 * Returns E_OKAY   if the timer was started (or stopped).                  *
 *         E_SYSTEM if the timeout timer could not be set.                  */
static int StartTimer(ComPort_t *Port, unsigned int mSec) {
    int Stat;
    struct itimerspec TmOut;
    TmOut.it_value.tv_sec = mSec / 1000;
    TmOut.it_value.tv_nsec = (mSec % 1000) * 1000000;
    TmOut.it_interval.tv_sec = TmOut.it_interval.tv_nsec = 0;
    //    if (timer_settime(Port->Tmr, TIMER_ADDREL, &TmOut, NULL) == 0)
         Stat = E_OKAY;
	 // else Stat = E_SYSTEM;
    return(Stat);
}


/* Stop the timeout timer.                                                  *
 * Returns E_OKAY   if the timer was stopped.                               *
 *         E_SYSTEM if the timeout timer could not be set.                  */
static int StopTimer(ComPort_t *Port) {
    return(StartTimer(Port, 0));
}

/*void sig_handler(int signal_number) {

  printf("caught signal %d\n", signal_number);
  fflush(stdout);
  sleep(1);
}
*/

/* Initialize the port to default Kameleon bicmd.c communication params.    *
 * Returns: E_OKAY    if successful.                                        *
 *          E_SYSTEM  if any of several system calls failed.                *
 *          E_NOTOPEN if the port is not open.                              */
int ComInit(ComPort_t *Port) {
  /*
    struct sigaction sa;
    siginterrupt(SIGINT, 1);
    memset(&sa, 0, sizeof(sa));
    sa.sa_flags != SA_INTERRUPT;
    sa.sa_handler = &sig_handler;
    sigaction(SIGINT, &sa, NULL);
  */

    int            Stat;
    struct termios Tios;
    if (Port->Fd < 0) Stat = E_NOTOPEN;
    else {
        memcpy(&Tios, &(Port->OldTios), sizeof(struct termios));
        Tios.c_iflag =   IGNBRK     /* IGNBRK = Ignore rcvd hardware breaks.*/
                       | 0          /* BRKINT = Generate SIGINT on break.   */
                       | IGNPAR     /* IGNPAR = Ignore parity errors.       */
                       | 0          /* PARMRK = Mark parity errors.         */
                       | 0          /* INPCK  = Enable software parity chk. */
                       | 0          /* ISTRIP = Strip 8th bit of rcvd chars.*/
	               | 0          /* INLCR  = Map '\n' to '\r' on input.  */
                       | 0          /* IGNCR  = Ignore received '\r'.       */
                       | 0          /* ICRNL  = Map '\r' to '\n' on input.  */
                       | 0          /* IXON   = Enable s/w output flow ctrl.*/
                       | 0;         /* IXOFF  = Enable s/w input flow ctrl. */
        Tios.c_oflag =   0;         /* OPOST  = Post-process output data.   */
        Tios.c_cflag =   0          /* IHFLOW = Enable h/w input flow ctrl. */
                       | 0          /* OHFLOW = Enable h/w output flow ctrl.*/
                       | 0          /* PARSTK = Stick parity.               */
                       | 0          /* IIDLE  = ?                           */
                       | CS8        /* CSIZE, CS5,CS6,CS7,CS8 = Data bits.  */
                       | 0          /* CSTOPB = 2 stop bits (else 1).       */
                       | CREAD      /* CREAD  = Enable receiver.            */
                       | 0          /* PARENB = Enable parity.              */
                       | 0          /* PARODD = Odd parity (else even).     */
                       | 0          /* HUPCL  = Hang up on last close.      */
                       | CLOCAL;    /* CLOCAL = Assume no modem control.    */
        Tios.c_lflag =   0          /* ISIG   = Generate SIGs on spec chars.*/
                       | 0          /* ICANON = Enable input line editing.  */
                       | 0          /* ECHO   = Echo received characters.   */
                       | 0          /* ECHOE  = Erase erases displayed char.*/
                       | 0          /* ECHOK  = Kill erases displayed line. */
                       | 0          /* ECHONL = Echo '\n' even if ECHO off. */
                       | 0          /* NOFLSH = No flush on INTR, QUIT,SUSP.*/
                       | 0          /* TOSTOP = ?                           */
                       | 0;         /* IEXTEN = Enable "extra" special chrs.*/
#ifdef QNX
        Tios.c_qflag =   0          /* TC_PROTECT_HFLOW                     */
                       | 0          /* TC_PROTECT_SFLOW                     */
                       | 0          /* TC_WAIT_SFLOW                        */
                       | 0          /* TC_NOPGRP                            */
                       | 0          /* TC_ECHOI                             */
                       | 0          /* TC_IFWD                              */
                       | 0;         /* TC_PROTECT_IEXTEN                    */
#endif 

	cfsetispeed(&Tios, B38400);
        cfsetospeed(&Tios, B38400);

	Tios.c_cflag |= B38400 | CS8 | CLOCAL | CREAD;
	//Tios.c_iflag = IGNPAR;

	Tios.c_cc[VMIN] = 0;
	Tios.c_cc[VTIME] = 5;

	tcflush(Port->Fd, TCIFLUSH);
	tcsetattr(Port->Fd, TCSANOW, &Tios);


        
	/*
	if (   (tcsetattr(Port->Fd, TCSANOW, &Tios) < 0)
            || (tcflow(Port->Fd, TCOON)             < 0)  // stty -ospaged. 
	|| (tcflow(Port->Fd, TCOONHW)           < 0) )// stty -ohpaged. 
	Stat = E_SYSTEM;
         else Stat = E_OKAY;
	*/
    }
 

    Stat = E_OKAY; // linux hack
    return(Stat);
}


/* Set the baud rate to the specified speed as soon as the transmit buffer  *
 * is empty.                                                                *
 * Returns: E_OKAY if successful.                                           *
 *          E_INVALID if an unsupported baud rate is specified.             *
 *          E_SYSTEM  if the baud rate could not be changed.                *
 *          E_NOTOPEN if the port is not open.                              *
 *                                                                          *
 * BUG NOTE:  It appears the the last byte or two in the transmit buffer    *
 * are actually sent at the new baud rate.  Presumably, this is because     *
 * tcsetattr(..., TCSADRAIN, ...) changes the baud rate after the software  *
 * buffer empties, but does not necessarily wait for the hardware buffer to *
 * drain.                                                                   */
int ComSetBaud(ComPort_t *Port, int Rate) {
    struct termios Tios;
    speed_t        Speed;
    int            Stat;
    Stat = E_OKAY;
    if (Port->Fd < 0) Stat = E_NOTOPEN;
    else {
        switch (Rate) {
            case   600: Speed =   B600;   break;
            case  1200: Speed =  B1200;   break;
            case  2400: Speed =  B2400;   break;
            case  4800: Speed =  B4800;   break;
            case  9600: Speed =  B9600;   break;
            case 19200: Speed = B19200;   break;
            case 38400: Speed = B38400;   break;
            default   : Stat = E_INVALID; break;
        }
    }
    if ((Stat == E_OKAY) && (tcgetattr(Port->Fd, &Tios) < 0)) Stat = E_SYSTEM;
    if (Stat == E_OKAY) {
        cfsetispeed(&Tios, Speed);
        cfsetospeed(&Tios, Speed);
        if (tcsetattr(Port->Fd, TCSADRAIN, &Tios) < 0) Stat = E_SYSTEM;
        else                                           Port->Baud = Rate;
    }
    return(Stat);
}


int ComGetBaud(ComPort_t *Port) {
    return(Port->Baud);
}


/* Timed wait for the output buffer to drain (i.e.: For data in the output  *
 * buffer to be sent).  If mSec == 0, waits forever (no timeout).           *
 * Returns: E_OKAY    if output buffer drained.                             *
 *          E_TIMEOUT if mSec milliseconds elapsed before drained.          *
 *          E_NOTOPEN if the port is not open.                              *
 *          E_SYSTEM  if system call failed.                                */
int ComDrainWait(ComPort_t *Port, unsigned int mSec) {
    int Stat;
    if (Port->Fd < 0) Stat = E_NOTOPEN;
    else              Stat = StartTimer(Port, mSec);
    if (Stat == E_OKAY) {
        if (tcdrain(Port->Fd) < 0) {
            if (errno == EINTR) Stat = E_TIMEOUT;
            else                Stat = E_SYSTEM;
        }
        StopTimer(Port);
    }
    return(Stat);
}


/* Flushes (di[Dscards) data in the input and/or output queues.               *
 * Queue = TCIFLUSH  to flush the input queue.                              *
 *         TCOFLUSH  to flush the output queue.                             *
 *         TCIOFLUSH to flush both queues.                                  *
 * Returns: E_OKAY    if successful.                                        *
 *          E_INVALID if Queue was invalid.                                 *
 *          E_SYSTEM  if the tcflush() call failed.                         *
 *          E_NOTOPEN if the port is not open.                              */
int ComFlush(ComPort_t *Port, int Queue) {
    int Stat;
    int fl;

    if(Port->Fd < 0) {
      Stat = E_NOTOPEN;
    } else {
      fl = tcflush(Port->Fd, Queue);
      if (fl == 0) {
	Stat = E_OKAY;
      } else if(errno == EINVAL) {
	Stat = E_INVALID;
      } else {
        Stat = E_SYSTEM;
      }
    }
    return(Stat);
}


/* Write Length bytes from Buffer to the port.  If Length < 0, Buffer is    *
 * assumed to contain a '\0'-terminated string.                             *
 * Returns: E_OKAY    if successful.                                        *
 *          E_WRITE   if the write() system call failed.                    *
 *          E_NOTOPEN if the port is not open.                              */
int ComWrite(ComPort_t *Port, char *Buffer, int Length) {
    int Stat, Wrote;
    if (Length < 0) Length = strlen(Buffer);
    if (Port->Fd < 0) Stat = E_NOTOPEN;
    else              Stat = E_OKAY;
    while ((Stat == E_OKAY) && (Length > 0)) {
        Wrote = write(Port->Fd, Buffer, Length);
        if (Wrote < 0) Stat = E_WRITE;
        else {
            Buffer += Wrote;
            Length -= Wrote;
        }
    }
    return(Stat);
}


/* Read fixed or variable length data from the port, with optional timeout. *
 * If mSec == 0:   Read waits forever for the specified data (see BufLen).  *
 * If mSec >  0:   Read times out after mSec milliseconds if the specified  *
 *                 data is not received.  NumRead can be checked to         *
 *                 determine how much data is in Buffer.                    *
 * If BufLen >= 0: A fixed length data block of BufLen bytes is expected.   *
 *                 Read returns when BufLen bytes have been read.           *
 * If BufLen <  0: A variable length string terminated by "=> " of length   *
 *                 < -BufLen is expected.  Read terminates when the string  *
 *                 is received or the buffer overflows.  Note that in this  *
 *                 mode, a terminating '\0' is appended, so -BufLen should  *
 *                 allow at least one extra byte.                           *
 * Data received is returned in Buffer.  If NumRead != NULL, the number of  *
 * bytes read (excluding the appended '\0' if BufLen < 0) is returned in    *
 * NumRead.                                                                 *
 * Returns: E_OKAY    if successful.                                        *
 *          E_TIMEOUT if read timed out.                                    *
 *          E_OVFL    if Buffer filled before "=> " received (BufLen < 0).  *
 *          E_READ    if the read() system call failed.                     *
 *          E_NOTOPEN if the port is not open.                              */
int ComRead(ComPort_t *Port, char *Buffer, int BufLen, int *NumRead,
            unsigned int mSec                                       ){
    int     Stat, ReadSoFar;
    ssize_t Rd;
    ReadSoFar = 0;
    if      (Port->Fd < 0) {
      Stat = E_NOTOPEN;
      //printf("ComRead() -- not open error!!\n");
    }
    else if (mSec     > 0) {
      // printf("ComRead() -- starting timer\n");
       Stat = StartTimer(Port, mSec);
       //printf("ComRead() -- timer started\n");
    }
    else {
       Stat = E_OKAY;
       //printf("ComRead() -- OKAY\n");
    }
    if (Stat == E_OKAY) {
        if (BufLen < 0) {
	  //printf("ComRead() -- reading variable length block\n");
            /* Reading variable length string of at most -BufLen - 1 bytes. */
            BufLen = - BufLen;
            Buffer[0] = '\0';
            while (true) {
	      //printf("ComRead() -- variable iterate\n");
                if (ReadSoFar + 1 == BufLen) {
                    Stat = E_OVFL;
		    //printf("ComRead() -- OVERFLOW ERROR!!\n");
		    /**/    break;
                }
		//printf("ComRead() -- reading!!\n");
                //usleep(10000);
		//printf("Buff:%s, ReadSoFar=%d\n", Buffer, ReadSoFar);
		Rd = read(Port->Fd, Buffer + ReadSoFar, 1);
		//printf("RD = %d, errno=%d\n", Rd, errno);

		if (Rd <= 0) {
		   // printf("ComRead() -- READ NEGATIVE ERROR!!\n");
                    Stat = ( (errno == EINTR) ? E_TIMEOUT : E_READ);
            /**/    break;
                }
                ReadSoFar += Rd;
                Buffer[ReadSoFar] = '\0';
                if (   (ReadSoFar >= 1                            )
                    && (strcmp(Buffer + ReadSoFar - 1, "\r") == 0) ) {
                    Stat = E_OKAY;
            /**/    break;
                }
            }
        } else {
            /* Reading fixed length block of BufLen bytes. */
  	    //printf("ComRead() -- reading fixed length block\n");
            while (true) {
	      //printf("ComRead() -- fixed iterate\n");
                Rd = read(Port->Fd, Buffer + ReadSoFar, BufLen - ReadSoFar);
                if (Rd < 0) {
                    Stat = ( (errno == EINTR) ? E_TIMEOUT : E_READ);
            /**/    break;
                }
                ReadSoFar += Rd;
                if (ReadSoFar == BufLen) {
                    Stat = E_OKAY;
            /**/    break;
                }
            }
        }
        StopTimer(Port);
    }
    if (NumRead != NULL) *NumRead = ReadSoFar;
    return(Stat);
}


/* Restore the original communication parameters and close the port.  Does  *
 * nothing if the port is not open.                                         */
void ComClose(ComPort_t *Port) {
    if (Port->Fd >= 0) {
        tcsetattr(Port->Fd, TCSADRAIN, &(Port->OldTios));
        close(Port->Fd);
        Port->Fd = -1;
        //timer_delete(Port->Tmr);
        Port->Tmr = (timer_t) -1;
        signal(SIGALRM, SIG_DFL);
    }
}


/* Open the specified serial port, save old communication parameters, and   *
 * set initial communication parameters (default parameters are appropriate *
 * for the Barrett Hand).                                                   *
 * Returns: E_OKAY   if successful.                                         *
 *          E_OPEN   if the port could not be opened.                       *
 *          E_INUSE  if the port is already open by someone else.           *
 *          E_SYSTEM if any of several system calls failed.                 */
int ComOpen(ComPort_t *Port , const char *DevName) {
      int                    Stat;

    /* If already open, close the old one first. */
    ComClose(Port);

    Stat = E_OKAY;

    // Allocate timer and set SIGALRM processing for timeout detection. 
  /*  if (   ( (Port->Tmr = timer_create(CLOCK_REALTIME, NULL, 0)) >= 0)
	   && (signal(SIGALRM, AlrmTrap) != SIG_ERR                  ) ) {
      Stat = E_OKAY;
    } else {
      Stat = E_SYSTEM;
    }*/
    //

    /* Open the specified serial port. */
    if ((Stat == E_OKAY) && ( (Port->Fd = open(DevName, O_RDWR | O_NOCTTY )) < 0)) {
       Stat = E_OPEN;
    } 
 
    /* Get device info and check open_count to see if device already in     *
     * use.                                                                 */
    /*    if ((Stat == E_OKAY) && (dev_info(Port->Fd, &DevInfo) < 0))
        Stat = E_SYSTEM;
   
    if ((Stat == E_OKAY) && (DevInfo.open_count != 1))    Stat = E_INUSE;
    */
    /* Save old communication parameters and set new ones. */
    if ((Stat == E_OKAY) && (tcgetattr(Port->Fd, &(Port->OldTios)) < 0)) {
      Stat = E_SYSTEM;
    }

    if (Stat != E_OKAY) {
        /* An error occurred above. */
        if (Port->Fd >= 0) {  /* If device was opened, just close it. */
            close(Port->Fd);
            Port->Fd = -1;
        }
	if (Port->Tmr >= 0) { // If timer was allocated, deallocate it. 
	  //timer_delete(Port->Tmr);
            Port->Tmr = (timer_t) -1;
        }
        signal(SIGALRM, SIG_DFL);
    } else {
        /* No errors so far, so set communication parameters.  If error,    *
         * (try to) restore original parameters and close the device.       */
        Stat = ComInit(Port);
        if (Stat != E_OKAY) ComClose(Port);
    }
    //printf("ComOpen ret: %d, Port->Fd=%d\n", Stat, Port->Fd);
    
    return(Stat);
}


/* This function is the C equivalent of the C++ default constructor.  It    *
 * should be called before any other function for a given ComPort_t to      *
 * initialize the ComPort_t structure to a known state, and should not be   *
 * called thereafter.                                                       */
void ComConstruct(ComPort_t *Port) {
    Port->Fd   = -1;
    Port->Tmr  = (timer_t)-1;
    Port->Baud = 38400;
}


/* This function is the C equivalent of the C++ destructor.  It should be   *
 * called when the program is completely finished with the ComPort_t        *
 * structure.                                                               */
void ComDestroy(ComPort_t *Port) {
    ComClose(Port);
}

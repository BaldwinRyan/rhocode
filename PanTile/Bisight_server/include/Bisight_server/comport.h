 /****************************************************************************
 * comport.h                                                                *
 ****************************************************************************
 * 000707 - DWheeler - Converted from C++ to C.                             *
 ****************************************************************************/

#ifndef _comport_h_
#define _comport_h_

#include <termios.h>
#include <time.h>

#define E_OKAY       0  // No error.
#define E_OPEN      -1  // Error opening device.
#define E_INUSE     -2  // Port already in use.
#define E_SYSTEM    -3  // Error in system call.
#define E_NOTOPEN   -4  // Device is not open.
#define E_WRITE     -5  // Write error.
#define E_READ      -6  // Read error.
#define E_TIMEOUT   -7  // Operation timed out.
#define E_OVFL      -8  // Input buffer overflow.
#define E_INVALID   -9  // Invalid parameter specified.

#define false 0                 
#define true  1                

typedef struct {
    int            Fd;
    int            Baud;
    struct termios OldTios;
    timer_t        Tmr;
} ComPort_t;


extern void ComConstruct(ComPort_t *Port                           );
extern void ComDestroy  (ComPort_t *Port                           );
extern int  ComOpen     (ComPort_t *Port, const char *DevName      );
extern int  ComInit     (ComPort_t *Port                           );
extern int  ComSetBaud  (ComPort_t *Port, int Rate                 );
extern int  ComGetBaud  (ComPort_t *Port                           );
extern int  ComRead     (ComPort_t *Port, char *Buffer, int BufLen,
                         int *NumRead, unsigned int mSec           );
extern int  ComWrite    (ComPort_t *Port, char *Buffer, int Length );
extern int  ComDrainWait(ComPort_t *Port, unsigned int mSec        );
extern int  ComFlush    (ComPort_t *Port, int Queue                );
extern void ComClose    (ComPort_t *Port                           );

#endif

TARGET = ../lib/libBiclops.so

SOURCES = Biclops.cpp PMD.cpp PMDParallel.cpp \
          PMDSerial.cpp PMDTransport.cpp PortableSerial.cpp \
          PMDAxisControl.cpp PMDController.cpp PMDUtils.cpp \
          Compass.cpp Sequencing.cpp PMDNetwork.cpp

OBJECTS = $(SOURCES:.cpp=.o)

INCLUDES = -I../include

COMPFLAGS = -g -DUNIX

all: $(TARGET)

$(TARGET): $(OBJECTS)
	ar rcs $(TARGET) $(OBJECTS)

.cpp.o:
	g++ $(COMPFLAGS) $(INCLUDES) -c $<

depend: $(SOURCES)
	makedepend -f libBiclops.mak $(COMPFLAGS) $(INCLUDES) $(SOURCES)

clean:
	/bin/rm -rf $(OBJECTS) $(TARGET)

#------------------
# Dependencies
# -----------------

# DO NOT DELETE THIS LINE -- make depend depends on it.

Biclops.o: /usr/include/math.h /usr/include/features.h
Biclops.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
Biclops.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
Biclops.o: /usr/include/bits/mathcalls.h ../include/PMDUtils.h
Biclops.o: /usr/include/time.h
Biclops.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
Biclops.o: /usr/include/bits/time.h /usr/include/bits/types.h
Biclops.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
Biclops.o: ../include/PMDAxisControl.h /usr/include/stdio.h
Biclops.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
Biclops.o: /usr/include/bits/wchar.h /usr/include/gconv.h
Biclops.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
Biclops.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
Biclops.o: ../include/PMD.h ../include/PMDTypes.h ../include/PMDErrorCodes.h
Biclops.o: ../include/PMDTransport.h ../include/PMDIOTransport.h
Biclops.o: ../include/PortableSerial.h /usr/include/sys/types.h
Biclops.o: /usr/include/sys/ioctl.h /usr/include/bits/ioctls.h
Biclops.o: /usr/include/asm/ioctls.h /usr/include/asm/ioctl.h
Biclops.o: /usr/include/bits/ioctl-types.h /usr/include/sys/ttydefaults.h
Biclops.o: /usr/include/sys/stat.h /usr/include/bits/stat.h
Biclops.o: /usr/include/fcntl.h /usr/include/bits/fcntl.h
Biclops.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
Biclops.o: /usr/include/bits/confname.h /usr/include/signal.h
Biclops.o: /usr/include/bits/sigset.h /usr/include/bits/signum.h
Biclops.o: /usr/include/termios.h /usr/include/bits/termios.h
Biclops.o: /usr/include/pthread.h /usr/include/sched.h
Biclops.o: /usr/include/bits/sched.h /usr/include/bits/pthreadtypes.h
Biclops.o: /usr/include/bits/initspin.h /usr/include/bits/sigthread.h
Biclops.o: ../include/PMDCollections.h ../include/PMDController.h
Biclops.o: ../include/PMDSerial.h ../include/Sequencing.h
Biclops.o: ../include/Compass.h ../include/singleton.h ../include/Biclops.h
PMD.o: /usr/include/string.h /usr/include/features.h /usr/include/sys/cdefs.h
PMD.o: /usr/include/gnu/stubs.h
PMD.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
PMD.o: /usr/include/stdio.h /usr/include/bits/types.h
PMD.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
PMD.o: /usr/include/libio.h /usr/include/_G_config.h /usr/include/wchar.h
PMD.o: /usr/include/bits/wchar.h /usr/include/gconv.h
PMD.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
PMD.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
PMD.o: ../include/PMDOPCodes.h ../include/PMDSerial.h
PMD.o: ../include/PortableSerial.h /usr/include/sys/types.h
PMD.o: /usr/include/time.h /usr/include/bits/time.h /usr/include/sys/ioctl.h
PMD.o: /usr/include/bits/ioctls.h /usr/include/asm/ioctls.h
PMD.o: /usr/include/asm/ioctl.h /usr/include/bits/ioctl-types.h
PMD.o: /usr/include/sys/ttydefaults.h /usr/include/sys/stat.h
PMD.o: /usr/include/bits/stat.h /usr/include/fcntl.h
PMD.o: /usr/include/bits/fcntl.h /usr/include/unistd.h
PMD.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
PMD.o: /usr/include/signal.h /usr/include/bits/sigset.h
PMD.o: /usr/include/bits/signum.h /usr/include/termios.h
PMD.o: /usr/include/bits/termios.h /usr/include/pthread.h
PMD.o: /usr/include/sched.h /usr/include/bits/sched.h
PMD.o: /usr/include/bits/pthreadtypes.h /usr/include/bits/initspin.h
PMD.o: /usr/include/bits/sigthread.h ../include/PMDIOTransport.h
PMD.o: ../include/PMDTypes.h /usr/include/math.h /usr/include/bits/huge_val.h
PMD.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
PMD.o: ../include/PMD.h ../include/PMDErrorCodes.h ../include/PMDTransport.h
PMDParallel.o: /usr/include/stdio.h /usr/include/features.h
PMDParallel.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
PMDParallel.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
PMDParallel.o: /usr/include/bits/types.h /usr/include/bits/wordsize.h
PMDParallel.o: /usr/include/bits/typesizes.h /usr/include/libio.h
PMDParallel.o: /usr/include/_G_config.h /usr/include/wchar.h
PMDParallel.o: /usr/include/bits/wchar.h /usr/include/gconv.h
PMDParallel.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
PMDParallel.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
PMDParallel.o: /usr/include/sys/timeb.h /usr/include/time.h
PMDParallel.o: /usr/include/bits/time.h ../include/PMDTypes.h
PMDParallel.o: /usr/include/math.h /usr/include/bits/huge_val.h
PMDParallel.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
PMDParallel.o: ../include/PMDOPCodes.h ../include/PMDErrorCodes.h
PMDParallel.o: ../include/PMDParallel.h ../include/PMDIOTransport.h
PMDSerial.o: /usr/include/stdio.h /usr/include/features.h
PMDSerial.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
PMDSerial.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
PMDSerial.o: /usr/include/bits/types.h /usr/include/bits/wordsize.h
PMDSerial.o: /usr/include/bits/typesizes.h /usr/include/libio.h
PMDSerial.o: /usr/include/_G_config.h /usr/include/wchar.h
PMDSerial.o: /usr/include/bits/wchar.h /usr/include/gconv.h
PMDSerial.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
PMDSerial.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
PMDSerial.o: /usr/include/string.h ../include/PMDTypes.h /usr/include/math.h
PMDSerial.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
PMDSerial.o: /usr/include/bits/mathcalls.h ../include/PMDOPCodes.h
PMDSerial.o: ../include/PMDSerial.h ../include/PortableSerial.h
PMDSerial.o: /usr/include/sys/types.h /usr/include/time.h
PMDSerial.o: /usr/include/bits/time.h /usr/include/sys/ioctl.h
PMDSerial.o: /usr/include/bits/ioctls.h /usr/include/asm/ioctls.h
PMDSerial.o: /usr/include/asm/ioctl.h /usr/include/bits/ioctl-types.h
PMDSerial.o: /usr/include/sys/ttydefaults.h /usr/include/sys/stat.h
PMDSerial.o: /usr/include/bits/stat.h /usr/include/fcntl.h
PMDSerial.o: /usr/include/bits/fcntl.h /usr/include/unistd.h
PMDSerial.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
PMDSerial.o: /usr/include/signal.h /usr/include/bits/sigset.h
PMDSerial.o: /usr/include/bits/signum.h /usr/include/termios.h
PMDSerial.o: /usr/include/bits/termios.h /usr/include/pthread.h
PMDSerial.o: /usr/include/sched.h /usr/include/bits/sched.h
PMDSerial.o: /usr/include/bits/pthreadtypes.h /usr/include/bits/initspin.h
PMDSerial.o: /usr/include/bits/sigthread.h ../include/PMDIOTransport.h
PMDSerial.o: ../include/PMDUtils.h ../include/PMDAxisControl.h
PMDSerial.o: ../include/PMD.h ../include/PMDErrorCodes.h
PMDSerial.o: ../include/PMDTransport.h ../include/PMDDiagnostics.h
PMDTransport.o: ../include/PMDTypes.h /usr/include/math.h
PMDTransport.o: /usr/include/features.h /usr/include/sys/cdefs.h
PMDTransport.o: /usr/include/gnu/stubs.h /usr/include/bits/huge_val.h
PMDTransport.o: /usr/include/bits/mathdef.h /usr/include/bits/mathcalls.h
PMDTransport.o: ../include/PMDOPCodes.h ../include/PMDErrorCodes.h
PMDTransport.o: ../include/PMDTransport.h ../include/PMDIOTransport.h
PortableSerial.o: ../include/PMDUtils.h /usr/include/time.h
PortableSerial.o: /usr/include/features.h /usr/include/sys/cdefs.h
PortableSerial.o: /usr/include/gnu/stubs.h
PortableSerial.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
PortableSerial.o: /usr/include/bits/time.h /usr/include/bits/types.h
PortableSerial.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
PortableSerial.o: ../include/PMDAxisControl.h /usr/include/stdio.h
PortableSerial.o: /usr/include/libio.h /usr/include/_G_config.h
PortableSerial.o: /usr/include/wchar.h /usr/include/bits/wchar.h
PortableSerial.o: /usr/include/gconv.h
PortableSerial.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
PortableSerial.o: /usr/include/bits/stdio_lim.h
PortableSerial.o: /usr/include/bits/sys_errlist.h ../include/PMD.h
PortableSerial.o: ../include/PMDTypes.h /usr/include/math.h
PortableSerial.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
PortableSerial.o: /usr/include/bits/mathcalls.h ../include/PMDErrorCodes.h
PortableSerial.o: ../include/PMDTransport.h ../include/PMDIOTransport.h
PortableSerial.o: ../include/PortableSerial.h /usr/include/sys/types.h
PortableSerial.o: /usr/include/sys/ioctl.h /usr/include/bits/ioctls.h
PortableSerial.o: /usr/include/asm/ioctls.h /usr/include/asm/ioctl.h
PortableSerial.o: /usr/include/bits/ioctl-types.h
PortableSerial.o: /usr/include/sys/ttydefaults.h /usr/include/sys/stat.h
PortableSerial.o: /usr/include/bits/stat.h /usr/include/fcntl.h
PortableSerial.o: /usr/include/bits/fcntl.h /usr/include/unistd.h
PortableSerial.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
PortableSerial.o: /usr/include/signal.h /usr/include/bits/sigset.h
PortableSerial.o: /usr/include/bits/signum.h /usr/include/termios.h
PortableSerial.o: /usr/include/bits/termios.h /usr/include/pthread.h
PortableSerial.o: /usr/include/sched.h /usr/include/bits/sched.h
PortableSerial.o: /usr/include/bits/pthreadtypes.h
PortableSerial.o: /usr/include/bits/initspin.h /usr/include/bits/sigthread.h
PMDAxisControl.o: /usr/include/math.h /usr/include/features.h
PMDAxisControl.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
PMDAxisControl.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
PMDAxisControl.o: /usr/include/bits/mathcalls.h /usr/include/limits.h
PMDAxisControl.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/limits.h
PMDAxisControl.o: ../include/PMDUtils.h /usr/include/time.h
PMDAxisControl.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
PMDAxisControl.o: /usr/include/bits/time.h /usr/include/bits/types.h
PMDAxisControl.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
PMDAxisControl.o: ../include/PMDAxisControl.h /usr/include/stdio.h
PMDAxisControl.o: /usr/include/libio.h /usr/include/_G_config.h
PMDAxisControl.o: /usr/include/wchar.h /usr/include/bits/wchar.h
PMDAxisControl.o: /usr/include/gconv.h
PMDAxisControl.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
PMDAxisControl.o: /usr/include/bits/stdio_lim.h
PMDAxisControl.o: /usr/include/bits/sys_errlist.h ../include/PMD.h
PMDAxisControl.o: ../include/PMDTypes.h ../include/PMDErrorCodes.h
PMDAxisControl.o: ../include/PMDTransport.h ../include/PMDIOTransport.h
PMDAxisControl.o: ../include/PortableSerial.h /usr/include/sys/types.h
PMDAxisControl.o: /usr/include/sys/ioctl.h /usr/include/bits/ioctls.h
PMDAxisControl.o: /usr/include/asm/ioctls.h /usr/include/asm/ioctl.h
PMDAxisControl.o: /usr/include/bits/ioctl-types.h
PMDAxisControl.o: /usr/include/sys/ttydefaults.h /usr/include/sys/stat.h
PMDAxisControl.o: /usr/include/bits/stat.h /usr/include/fcntl.h
PMDAxisControl.o: /usr/include/bits/fcntl.h /usr/include/unistd.h
PMDAxisControl.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
PMDAxisControl.o: /usr/include/signal.h /usr/include/bits/sigset.h
PMDAxisControl.o: /usr/include/bits/signum.h /usr/include/termios.h
PMDAxisControl.o: /usr/include/bits/termios.h /usr/include/pthread.h
PMDAxisControl.o: /usr/include/sched.h /usr/include/bits/sched.h
PMDAxisControl.o: /usr/include/bits/pthreadtypes.h
PMDAxisControl.o: /usr/include/bits/initspin.h /usr/include/bits/sigthread.h
PMDAxisControl.o: ../include/PMDOPCodes.h ../include/PMDCollections.h
PMDAxisControl.o: ../include/PMDController.h ../include/PMDSerial.h
PMDAxisControl.o: ../include/Sequencing.h ../include/Compass.h
PMDAxisControl.o: ../include/singleton.h
PMDController.o: /usr/include/string.h /usr/include/features.h
PMDController.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
PMDController.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
PMDController.o: /usr/include/stdio.h /usr/include/bits/types.h
PMDController.o: /usr/include/bits/wordsize.h /usr/include/bits/typesizes.h
PMDController.o: /usr/include/libio.h /usr/include/_G_config.h
PMDController.o: /usr/include/wchar.h /usr/include/bits/wchar.h
PMDController.o: /usr/include/gconv.h
PMDController.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
PMDController.o: /usr/include/bits/stdio_lim.h
PMDController.o: /usr/include/bits/sys_errlist.h ../include/PMDUtils.h
PMDController.o: /usr/include/time.h /usr/include/bits/time.h
PMDController.o: ../include/PMDAxisControl.h ../include/PMD.h
PMDController.o: ../include/PMDTypes.h /usr/include/math.h
PMDController.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
PMDController.o: /usr/include/bits/mathcalls.h ../include/PMDErrorCodes.h
PMDController.o: ../include/PMDTransport.h ../include/PMDIOTransport.h
PMDController.o: ../include/PortableSerial.h /usr/include/sys/types.h
PMDController.o: /usr/include/sys/ioctl.h /usr/include/bits/ioctls.h
PMDController.o: /usr/include/asm/ioctls.h /usr/include/asm/ioctl.h
PMDController.o: /usr/include/bits/ioctl-types.h
PMDController.o: /usr/include/sys/ttydefaults.h /usr/include/sys/stat.h
PMDController.o: /usr/include/bits/stat.h /usr/include/fcntl.h
PMDController.o: /usr/include/bits/fcntl.h /usr/include/unistd.h
PMDController.o: /usr/include/bits/posix_opt.h /usr/include/bits/confname.h
PMDController.o: /usr/include/signal.h /usr/include/bits/sigset.h
PMDController.o: /usr/include/bits/signum.h /usr/include/termios.h
PMDController.o: /usr/include/bits/termios.h /usr/include/pthread.h
PMDController.o: /usr/include/sched.h /usr/include/bits/sched.h
PMDController.o: /usr/include/bits/pthreadtypes.h
PMDController.o: /usr/include/bits/initspin.h /usr/include/bits/sigthread.h
PMDController.o: ../include/PMDCollections.h ../include/PMDController.h
PMDController.o: ../include/PMDSerial.h ../include/Sequencing.h
PMDController.o: ../include/Compass.h ../include/singleton.h
PMDUtils.o: /usr/include/math.h /usr/include/features.h
PMDUtils.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
PMDUtils.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
PMDUtils.o: /usr/include/bits/mathcalls.h /usr/include/stdio.h
PMDUtils.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
PMDUtils.o: /usr/include/bits/types.h /usr/include/bits/wordsize.h
PMDUtils.o: /usr/include/bits/typesizes.h /usr/include/libio.h
PMDUtils.o: /usr/include/_G_config.h /usr/include/wchar.h
PMDUtils.o: /usr/include/bits/wchar.h /usr/include/gconv.h
PMDUtils.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
PMDUtils.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
PMDUtils.o: /usr/include/ctype.h /usr/include/endian.h
PMDUtils.o: /usr/include/bits/endian.h ../include/PMDCollections.h
PMDUtils.o: ../include/PortableSerial.h /usr/include/sys/types.h
PMDUtils.o: /usr/include/time.h /usr/include/bits/time.h
PMDUtils.o: /usr/include/sys/ioctl.h /usr/include/bits/ioctls.h
PMDUtils.o: /usr/include/asm/ioctls.h /usr/include/asm/ioctl.h
PMDUtils.o: /usr/include/bits/ioctl-types.h /usr/include/sys/ttydefaults.h
PMDUtils.o: /usr/include/sys/stat.h /usr/include/bits/stat.h
PMDUtils.o: /usr/include/fcntl.h /usr/include/bits/fcntl.h
PMDUtils.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
PMDUtils.o: /usr/include/bits/confname.h /usr/include/signal.h
PMDUtils.o: /usr/include/bits/sigset.h /usr/include/bits/signum.h
PMDUtils.o: /usr/include/termios.h /usr/include/bits/termios.h
PMDUtils.o: /usr/include/pthread.h /usr/include/sched.h
PMDUtils.o: /usr/include/bits/sched.h /usr/include/bits/pthreadtypes.h
PMDUtils.o: /usr/include/bits/initspin.h /usr/include/bits/sigthread.h
PMDUtils.o: ../include/PMDController.h ../include/PMDSerial.h
PMDUtils.o: ../include/PMDIOTransport.h ../include/PMDTypes.h
PMDUtils.o: ../include/PMDAxisControl.h ../include/PMD.h
PMDUtils.o: ../include/PMDErrorCodes.h ../include/PMDTransport.h
PMDUtils.o: ../include/Sequencing.h ../include/PMDUtils.h
PMDUtils.o: ../include/Compass.h ../include/singleton.h
Compass.o: /usr/include/string.h /usr/include/features.h
Compass.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
Compass.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
Compass.o: ../include/PMDUtils.h /usr/include/time.h /usr/include/bits/time.h
Compass.o: /usr/include/bits/types.h /usr/include/bits/wordsize.h
Compass.o: /usr/include/bits/typesizes.h ../include/PMDAxisControl.h
Compass.o: /usr/include/stdio.h /usr/include/libio.h /usr/include/_G_config.h
Compass.o: /usr/include/wchar.h /usr/include/bits/wchar.h
Compass.o: /usr/include/gconv.h
Compass.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
Compass.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
Compass.o: ../include/PMD.h ../include/PMDTypes.h /usr/include/math.h
Compass.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
Compass.o: /usr/include/bits/mathcalls.h ../include/PMDErrorCodes.h
Compass.o: ../include/PMDTransport.h ../include/PMDIOTransport.h
Compass.o: ../include/PortableSerial.h /usr/include/sys/types.h
Compass.o: /usr/include/sys/ioctl.h /usr/include/bits/ioctls.h
Compass.o: /usr/include/asm/ioctls.h /usr/include/asm/ioctl.h
Compass.o: /usr/include/bits/ioctl-types.h /usr/include/sys/ttydefaults.h
Compass.o: /usr/include/sys/stat.h /usr/include/bits/stat.h
Compass.o: /usr/include/fcntl.h /usr/include/bits/fcntl.h
Compass.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
Compass.o: /usr/include/bits/confname.h /usr/include/signal.h
Compass.o: /usr/include/bits/sigset.h /usr/include/bits/signum.h
Compass.o: /usr/include/termios.h /usr/include/bits/termios.h
Compass.o: /usr/include/pthread.h /usr/include/sched.h
Compass.o: /usr/include/bits/sched.h /usr/include/bits/pthreadtypes.h
Compass.o: /usr/include/bits/initspin.h /usr/include/bits/sigthread.h
Compass.o: ../include/PMDCollections.h ../include/PMDController.h
Compass.o: ../include/PMDSerial.h ../include/Sequencing.h
Compass.o: ../include/Compass.h ../include/singleton.h
Sequencing.o: ../include/Sequencing.h ../include/PMDAxisControl.h
Sequencing.o: /usr/include/stdio.h /usr/include/features.h
Sequencing.o: /usr/include/sys/cdefs.h /usr/include/gnu/stubs.h
Sequencing.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stddef.h
Sequencing.o: /usr/include/bits/types.h /usr/include/bits/wordsize.h
Sequencing.o: /usr/include/bits/typesizes.h /usr/include/libio.h
Sequencing.o: /usr/include/_G_config.h /usr/include/wchar.h
Sequencing.o: /usr/include/bits/wchar.h /usr/include/gconv.h
Sequencing.o: /usr/lib/gcc-lib/i586-mandrake-linux-gnu/3.2.2/include/stdarg.h
Sequencing.o: /usr/include/bits/stdio_lim.h /usr/include/bits/sys_errlist.h
Sequencing.o: ../include/PMD.h ../include/PMDTypes.h /usr/include/math.h
Sequencing.o: /usr/include/bits/huge_val.h /usr/include/bits/mathdef.h
Sequencing.o: /usr/include/bits/mathcalls.h ../include/PMDErrorCodes.h
Sequencing.o: ../include/PMDTransport.h ../include/PMDIOTransport.h
Sequencing.o: ../include/PortableSerial.h /usr/include/sys/types.h
Sequencing.o: /usr/include/time.h /usr/include/bits/time.h
Sequencing.o: /usr/include/sys/ioctl.h /usr/include/bits/ioctls.h
Sequencing.o: /usr/include/asm/ioctls.h /usr/include/asm/ioctl.h
Sequencing.o: /usr/include/bits/ioctl-types.h /usr/include/sys/ttydefaults.h
Sequencing.o: /usr/include/sys/stat.h /usr/include/bits/stat.h
Sequencing.o: /usr/include/fcntl.h /usr/include/bits/fcntl.h
Sequencing.o: /usr/include/unistd.h /usr/include/bits/posix_opt.h
Sequencing.o: /usr/include/bits/confname.h /usr/include/signal.h
Sequencing.o: /usr/include/bits/sigset.h /usr/include/bits/signum.h
Sequencing.o: /usr/include/termios.h /usr/include/bits/termios.h
Sequencing.o: /usr/include/pthread.h /usr/include/sched.h
Sequencing.o: /usr/include/bits/sched.h /usr/include/bits/pthreadtypes.h
Sequencing.o: /usr/include/bits/initspin.h /usr/include/bits/sigthread.h
Sequencing.o: ../include/PMDUtils.h ../include/PMDCollections.h
Sequencing.o: ../include/PMDController.h ../include/PMDSerial.h
Sequencing.o: ../include/Compass.h ../include/singleton.h

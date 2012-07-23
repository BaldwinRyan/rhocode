TARGET = BiclopsBareBones
TARGETPATH = ../bin/

LIBS = ../lib/libBiclops.so

SOURCES = $(TARGET).cpp

OBJECTS = $(SOURCES:.cpp=.o)

INCLUDES = -I../include

COMPFLAGS = -g -DUNIX

all: depend $(TARGETPATH)$(TARGET)

$(TARGETPATH)$(TARGET): BiclopsBareBones.o $(LIBS)
	g++ $(COMPFLAGS) -o $(TARGETPATH)$(TARGET) $(OBJECTS) $(LIBS)

.cpp.o:
	g++ $(COMPFLAGS) $(INCLUDES) -c $<

depend: $(SOURCES)
	makedepend -f BiclopsBareBones.mak $(COMPFLAGS) $(INCLUDES) $(SOURCES)

clean:
	/bin/rm -rf $(TARGETPATH)$(TARGET) $(OBJECTS)

#------------------
# Dependencies
# -----------------

# DO NOT DELETE THIS LINE -- make depend depends on it.


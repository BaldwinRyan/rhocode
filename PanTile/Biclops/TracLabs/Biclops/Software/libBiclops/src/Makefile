# set this to whatever
INSTALL_ROOT=..

UTILSDIR=../../../../Motion/libUtils
PMDDIR=../../../../Motion/libPMD

STATIC_LIB=libBiclops.a
SHARED_LIB=libBiclops.so

TARGETS = $(STATIC_LIB) $(SHARED_LIB)

SOURCES = Biclops.cpp

CXX=g++
CPP=g++
CC=g++

OBJECTS = $(SOURCES:.cpp=.o)

INCLUDES = -I../include -I$(UTILSDIR)/include -I$(PMDDIR)/include

CXXFLAGS = -DUNIX -fPIC -ansi -Wall -pedantic -O3
CPPFLAGS = $(CXXFLAGS)
CFLAGS=$(CXXFLAGS)

LDFLAGS= -L$(UTILSDIR)/lib -L$(PMDDIR)/lib -L.
LDLIBS= -lBiclops -lPMD -lUtils

all: deps.d $(TARGETS)

$(STATIC_LIB): $(OBJECTS)
	ar rcs $@ $^
	cp $@ ../lib

$(SHARED_LIB): $(OBJECTS)
	$(CXX) -shared -Wl,-soname,$@ -o $@  $^ 
	cp $@ ../lib
	
.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $<

deps.d: $(SOURCES)
	$(CXX) -MM $(INCLUDES) $(SOURCES) -MF $@

clean:
	- @ rm -rf $(OBJECTS)

distclean: clean
	- @ rm -ff $(TARGETS) deps.d

BiclopsBareBones: ../samples/BiclopsBareBones.cpp $(TARGETS)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $(LDFLAGS) $(LDLIBS) -o $@ $< 

TrajectoryPMD: ../samples/TrajectoryPMD.cpp $(TARGETS)
	$(CXX) $(CXXFLAGS) $(INCLUDES)  $(LDFLAGS) $(LDLIBS) -o $@ $<

INSTALL_FLAGS= --preserve-timestamps
INSTALL_DIR_INC=$(INSTALL_ROOT)/include
INSTALL_DIR_LIB=$(INSTALL_ROOT)/lib
INSTALL_DIR_SHARE=$(INSTALL_ROOT)/share

INSTALL_TARGET_LIBS=$(STATIC_LIB) $(SHARED_LIB)
INSTALL_TARGET_INCS=$(wildcard ../include/*.h)
INSTALL_TARGET_SHARES+=../data/BiclopsDefault.cfg

dump:
	- @ echo LIBS=  $(INSTALL_TARGET_LIBS)
	- @ echo INCS=  $(INSTALL_TARGET_INCS)
	- @ echo SHARE= $(INSTALL_TARGET_SHARES)

install: $(INSTALL_TARGET_LIBS) $(INSTALL_TARGET_INCS) $(INSTALL_TARGET_SHARES)
	- @ install $(INSTALL_FLAGS) --mode=775 -d $(INSTALL_ROOT) $(INSTALL_DIR_INC) $(INSTALL_DIR_LIB) $(INSTALL_DIR_SHARE)
	- @ install $(INSTALL_FLAGS) --mode=664 $(INSTALL_TARGET_LIBS)   $(INSTALL_DIR_LIB)
	- @ install $(INSTALL_FLAGS) --mode=664 $(INSTALL_TARGET_INCS)   $(INSTALL_DIR_INC)
	- @ install $(INSTALL_FLAGS) --mode=664 $(INSTALL_TARGET_SHARES) $(INSTALL_DIR_SHARE)

-include deps.d


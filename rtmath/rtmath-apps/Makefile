BINOUTCOMBO = combo
BINOUTDDPARSE = ddparse
FC = gfortran
CC = gcc
CPP = g++
LD = g++
FFLAGS = -fdefault-integer-8 -fdefault-real-8 -ggdb
CCFLAGS = -ggdb -std=c++0x -fopenmp
LIBS = -lm -lgfortran -lnetcdf -ggdb -lboost_filesystem -lboost_system -lrtmath -fopenmp
WDIR=`pwd`
IDIR=$(HOME)/bin

FFILES =
CFILES = 
CPPFILES = 
COMBOFILES = combo.cpp
DDPARSEFILES = ddparse.cpp

all: combo ddparse

clean:
	rm -rf *.o $(BINOUTCOMBO) $(BINOUTDDPARSE)
	
distclean:
	rm -rf $(IDIR)/$(BINOUTCOMBO) $(IDIR)/$(BINOUTDDPARSE)
	
install: all distclean
	ln -s $(WDIR)/$(BINOUTCOMBO) $(IDIR)/$(BINOUTCOMBO)
	ln -s $(WDIR)/$(BINOUTDDPARSE) $(IDIR)/$(BINOUTDDPARSE)

.f.o:
	$(FC) $(FFLAGS) -c $?

.c.o:
	$(CC) $(CCFLAGS) -c $?

.cpp.o:
	$(CPP) $(CCFLAGS) -c $?

combo: $(FFILES:.f=.o) $(CFILES:.c=.o) $(CPPFILES:.cpp=.o) $(COMBOFILES:.cpp=.o) 
	$(LD) -o $(BINOUTCOMBO) $(FFILES:.f=.o) $(CFILES:.c=.o) $(COMBOFILES:.cpp=.o) $(CPPFILES:.cpp=.o) $(LIBS)
	
ddparse: $(FFILES:.f=.o) $(CFILES:.c=.o) $(CPPFILES:.cpp=.o) $(DDPARSEFILES:.cpp=.o) 
	$(LD) -o $(BINOUTDDPARSE) $(FFILES:.f=.o) $(CFILES:.c=.o) $(DDPARSEFILES:.cpp=.o) $(CPPFILES:.cpp=.o) $(LIBS)

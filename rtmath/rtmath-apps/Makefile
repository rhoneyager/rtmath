BINOUTCOMBO = combo
FC = gfortran
CC = gcc
CPP = g++
LD = g++
FFLAGS = -fdefault-integer-8 -fdefault-real-8 -ggdb
CCFLAGS = -ggdb
LIBS = -lm -lgfortran -lnetcdf -ggdb -lboost_filesystem -l boost_system

FFILES =
CFILES = 
CPPFILES = combo.cpp

all: combo

clean:
	rm -rf *.o $(BINOUTC)

.f.o:
	$(FC) $(FFLAGS) -c $?

.c.o:
	$(CC) $(CCFLAGS) -c $?

.cpp.o:
	$(CPP) $(CCFLAGS) -c $?

combo: $(FFILES:.f=.o) $(CFILES:.c=.o) $(CPPFILES:.cpp=.o) 
	$(LD) -o $(BINOUTCOMBO) $(FFILES:.f=.o) $(CFILES:.c=.o) $(CPPFILES:.cpp=.o) $(LIBS)
	

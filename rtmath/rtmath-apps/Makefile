#FC = gfortran
CC = gcc
CPP = g++
LD = g++
#FFLAGS = -fdefault-integer-8 -fdefault-real-8 -ggdb
CCFLAGS = -ggdb -std=c++0x -fopenmp
LIBS = -lm -lgfortran -lnetcdf -ggdb -lboost_filesystem -lboost_unit_test_framework -lboost_system -lrtmath -fopenmp
WDIR=`pwd`
IDIR=$(HOME)/bin

COMBOFILES = $(wildcard combo/*.cpp)
DDPARSEFILES = $(wildcard ddparse/*.cpp)

all: combo ddparse

clean:
	rm -rf *.o */*.o bin/*
	
distclean:
	rm -rf $(IDIR)/combo $(IDIR)/ddparse
	
install: all distclean
	ln -s $(WDIR)/bin/combo $(IDIR)/combo
	ln -s $(WDIR)/bin/ddparse $(IDIR)/ddparse

.f.o:
	$(FC) $(FFLAGS) -c $? -o $(?:.f=.o)

.c.o:
	$(CC) $(CCFLAGS) -c $? -o $(?:.c=.o)

.cpp.o:
	$(CPP) $(CCFLAGS) -c $? -o $(?:.cpp=.o)

combo: $(COMBOFILES:.cpp=.o) 
	$(LD) -o bin/combo $(COMBOFILES:.cpp=.o) $(LIBS)
	
ddparse: $(DDPARSEFILES:.cpp=.o) 
	$(LD) -o bin/ddparse $(DDPARSEFILES:.cpp=.o) $(LIBS)

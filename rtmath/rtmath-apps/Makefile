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
RTMATHVERFILES = $(wildcard rtmath-version/*.cpp)
LBLCALCFILES = $(wildcard lblcalc/*.cpp)
BACK_F_FILES = $(wildcard backscatter_f/*.cpp)
GM_FILES = $(wildcard getm/*.cpp)
TM_FILES = $(wildcard transmittance/*.cpp)


all: combo ddparse rtmath-version lblcalc backscatter_f getm transmittance

clean:
	rm -rf *.o */*.o bin/*
	
distclean:
	rm -rf $(IDIR)/combo $(IDIR)/rtmath-*

# Should wildcard all of these	
install: all distclean
	ln -s $(WDIR)/bin/combo $(IDIR)/combo
	ln -s $(WDIR)/bin/rtmath-ddparse $(IDIR)/rtmath-ddparse
	ln -s $(WDIR)/bin/rtmath-version $(IDIR)/rtmath-version
	ln -s $(WDIR)/bin/rtmath-lblcalc $(IDIR)/rtmath-lblcalc
	ln -s $(WDIR)/bin/rtmath-back_f $(IDIR)/rtmath-back_f
	ln -s $(WDIR)/bin/rtmath-getm $(IDIR)/rtmath-getm
	ln -s $(WDIR)/bin/rtmath-transmittance $(IDIR)/rtmath-transmittance

.f.o:
	$(FC) $(FFLAGS) -c $? -o $(?:.f=.o)

.c.o:
	$(CC) $(CCFLAGS) -c $? -o $(?:.c=.o)

.cpp.o:
	$(CPP) $(CCFLAGS) -c $? -o $(?:.cpp=.o)

combo: $(COMBOFILES:.cpp=.o) 
	$(LD) -o bin/combo $(COMBOFILES:.cpp=.o) $(LIBS)
	
ddparse: $(DDPARSEFILES:.cpp=.o) 
	$(LD) -o bin/rtmath-ddparse $(DDPARSEFILES:.cpp=.o) $(LIBS)

rtmath-version: $(RTMATHVERFILES:.cpp=.o) 
	$(LD) -o bin/rtmath-version $(RTMATHVERFILES:.cpp=.o) $(LIBS)

lblcalc: $(LBLCALCFILES:.cpp=.o) 
	$(LD) -o bin/rtmath-lblcalc $(LBLCALCFILES:.cpp=.o) $(LIBS)

backscatter_f: $(BACK_F_FILES:.cpp=.o) 
	$(LD) -o bin/rtmath-back_f $(BACK_F_FILES:.cpp=.o) $(LIBS)

getm: $(GM_FILES:.cpp=.o)
	$(LD) -o bin/rtmath-getm $(GM_FILES:.cpp=.o) $(LIBS)

transmittance: $(TM_FILES:.cpp=.o)
	$(LD) -o bin/rtmath-transmittance $(TM_FILES:.cpp=.o) $(LIBS)


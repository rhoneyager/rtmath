#FC = gfortran
CC = gcc
CPP = g++
LD = g++
#FFLAGS = -fdefault-integer-8 -fdefault-real-8 -ggdb
CCFLAGS = -ggdb -std=c++0x -fopenmp
LIBS = -lm -lgfortran -lnetcdf -ggdb -lboost_filesystem -lboost_unit_test_framework -lboost_system -fopenmp ../rtmath/lib/librtmath.a `root-config --libs`
INCDIR = -I `root-config --incdir`
WDIR=`pwd`
IDIR=$(HOME)/bin

COMBOFILES = $(wildcard combo/*.cpp)
CNVUNITSFILES = $(wildcard conv-units/*.cpp)
DDPARSEFILES = $(wildcard ddparse/*.cpp)
RTMATHVERFILES = $(wildcard rtmath-version/*.cpp)
GM_FILES = $(wildcard getm/*.cpp)
TM_FILES = $(wildcard transmittance/*.cpp)
AS_FILES = $(wildcard absorb-slab/*.cpp)
UU_FILES = $(wildcard uuid/*.cpp)
SEC_FILES = $(wildcard sector/*.cpp)

all: combo ddparse rtmath-version getm transmittance absorb-slab uuid sector convunits

clean:
	rm -rf *.o */*.o bin/*
	
distclean:
	rm -rf $(IDIR)/rtmath-*

# Should wildcard all of these	
install: all distclean
	ln -s $(WDIR)/bin/rtmath-combo $(IDIR)/rtmath-combo
	ln -s $(WDIR)/bin/rtmath-ddparse $(IDIR)/rtmath-ddparse
	ln -s $(WDIR)/bin/rtmath-version $(IDIR)/rtmath-version
	ln -s $(WDIR)/bin/rtmath-absorb-slab $(IDIR)/rtmath-absorb-slab
	ln -s $(WDIR)/bin/rtmath-back_f $(IDIR)/rtmath-back_f
	ln -s $(WDIR)/bin/rtmath-getm $(IDIR)/rtmath-getm
	ln -s $(WDIR)/bin/rtmath-getm $(IDIR)/rtmath-genmtab
	ln -s $(WDIR)/bin/rtmath-sector $(IDIR)/rtmath-sector
	ln -s $(WDIR)/bin/rtmath-transmittance $(IDIR)/rtmath-transmittance
	ln -s $(WDIR)/bin/rtmath-uuid-gen $(IDIR)/rtmath-uuid-gen

.f.o:
	$(FC) $(FFLAGS) -c $? -o $(?:.f=.o)

.c.o:
	$(CC) $(CCFLAGS) $(INCDIR) -c $? -o $(?:.c=.o)

.cpp.o:
	$(CPP) $(CCFLAGS) $(INCDIR) -c $? -o $(?:.cpp=.o)

combo: $(COMBOFILES:.cpp=.o) 
	$(LD) -o bin/rtmath-combo $(COMBOFILES:.cpp=.o) $(LIBS)
	
convunits: $(CNVUNITSFILES:.cpp=.o)
	$(LD) -o bin/rtmath-conv-units $(CNVUNITSFILES:.cpp=.o) $(LIBS)

ddparse: $(DDPARSEFILES:.cpp=.o) 
	$(LD) -o bin/rtmath-ddparse $(DDPARSEFILES:.cpp=.o) $(LIBS)

rtmath-version: $(RTMATHVERFILES:.cpp=.o) 
	$(LD) -o bin/rtmath-version $(RTMATHVERFILES:.cpp=.o) $(LIBS)

getm: $(GM_FILES:.cpp=.o)
	$(LD) -o bin/rtmath-getm $(GM_FILES:.cpp=.o) $(LIBS)

transmittance: $(TM_FILES:.cpp=.o)
	$(LD) -o bin/rtmath-transmittance $(TM_FILES:.cpp=.o) $(LIBS)

absorb-slab: $(AS_FILES:.cpp=.o)
	$(LD) -o bin/rtmath-absorb-slab $(AS_FILES:.cpp=.o) $(LIBS)

uuid: $(UU_FILES:.cpp=.o)
	$(LD) -o bin/rtmath-uuid-gen $(UU_FILES:.cpp=.o) $(LIBS)

sector: $(SEC_FILES:.cpp=.o)
	$(LD) -o bin/rtmath-sector $(SEC_FILES:.cpp=.o) $(LIBS)


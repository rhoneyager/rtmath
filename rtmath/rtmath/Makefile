# Makefile for the rtmath library
# Makes the dynamic library, static library and the testing suite

CC = gcc
CPP = g++
LD = g++
AR = ar
RTMATHCONF = $(HOME)/.rtmath
CCFLAGS = -ggdb -fPIC -rdynamic -std=c++0x -D_DEBUG
LIBS = -ggdb -lm -lnetcdf -lboost_filesystem -lboost_unit_test_framework -lboost_system
SRCSCPP = $(wildcard src/*.cpp)
SRCSC = $(wildcard *.c)

OBJS = $(SRCSCPP:.cpp=.o) $(SRCSC:.c=.o)
SRCSTEST = $(wildcard tests/*.cpp)
OBJSTEST = $(SRCSTEST:.cpp=.o)

TESTDATA=`pwd`/tests/data/

all: dynamic static

clean:
	rm -rf *.o bin/* lib/* tests/*.o src/*.o build/*

# Pre-build updates repositiory revition
pre-build: FORCE
	@echo "Runnning pre-build"
	@tcsh ./getRev.csh
	@touch src/debug.cpp
	@echo "#define RTC "\"$(RTMATHCONF)"\"" > build/rtc.h
	@touch src/config.cpp


# No pure c files used here for now
.c.o: 
	$(CC) $(CCFLAGS) -c $? -o $(?:.c=.o)

.cpp.o:
	$(CPP) $(CCFLAGS) -c $? -o $(?:.cpp=.o)

dynamic: pre-build $(OBJS)
	$(LD) -shared $(CCFLAGS) -o lib/librtmath.so $(OBJS)

static: pre-build $(OBJS)
	$(AR) rcs lib/librtmath.a $(OBJS)

FORCE:

# Compiles regular code along with test files
test: static $(OBJSTEST)
	$(LD) -ggdb -lboost_unit_test_framework $(CCFLAGS) -o bin/rtmath-test $(OBJSTEST) lib/librtmath.a $(LIBS)
	./bin/rtmath-test --log-level=test_suite --report_level=short


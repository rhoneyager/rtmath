# Makefile for the rtmath library
# Makes the dynamic library, static library and the testing suite

CC = gcc
CPP = g++
LD = g++
AR = ar
CCFLAGS = -ggdb -fPIC -rdynamic -std=c++0x
LIBS = -ggdb -lm -lnetcdf -lboost_filesystem -lboost_system
SRCSCPP = $(wildcard *.cpp) $(wildcard polynomials/*.cpp)
SRCSC = $(wildcard *.c)

OBJS = $(SRCSCPP:.cpp=.o) $(SRCSC:.c=.o)
SRCSTEST = $(wildcard tests/*.cpp)
OBJSTEST = $(SRCSTEST:.cpp=.o)

all: dynamic static

clean:
	rm -rf *.o bin/* tests/*.o polynomials/*.o

# Pre-build updates repositiory revition
pre-build:
	@tcsh ./getRev.csh
	@touch debug.cpp


# No pure c files used here for now
.c.o: pre-build
	$(CC) $(CCFLAGS) -c $? -o $(?:.c=.o)

.cpp.o: pre-build
	$(CPP) $(CCFLAGS) -c $? -o $(?:.cpp=.o)

dynamic: $(OBJS)
	$(LD) -shared $(CCFLAGS) -o bin/librtmath.so $(OBJS)

static: $(OBJS)
	$(AR) rcs bin/librtmath.a $(OBJS)

# Compiles regular code along with test files
test: $(OBJS) $(OBJSTEST)
	$(LD) -ggdb -lboost_unit_test_framework $(CCFLAGS) $(LIBS) -o bin/rtmath-test $(OBJS) $(OBJSTEST)
	./bin/rtmath-test --log-level=test_suite --report_level=short


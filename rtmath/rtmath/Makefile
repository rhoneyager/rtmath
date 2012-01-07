# Makefile for the rtmath library
# Makes the dynamic library, static library and the testing suite

CC = gcc
CPP = g++
LD = g++
AR = ar
CCFLAGS = -ggdb -fPIC -rdynamic -std=c++0x
LIBS = -ggdb -lm -lnetcdf -lboost_filesystem -lboost_system
SRCSCPP = $(wildcard src/*.cpp)
SRCSC = $(wildcard *.c)

OBJS = $(SRCSCPP:.cpp=.o) $(SRCSC:.c=.o)
SRCSTEST = $(wildcard tests/*.cpp)
OBJSTEST = $(SRCSTEST:.cpp=.o)

all: dynamic static

clean:
	rm -rf *.o bin/* lib/* tests/*.o src/*.o

# Pre-build updates repositiory revition
pre-build: FORCE
	@tcsh ./getRev.csh
	@touch src/debug.cpp


# No pure c files used here for now
.c.o: pre-build
	$(CC) $(CCFLAGS) -c $? -o $(?:.c=.o)

.cpp.o: pre-build
	$(CPP) $(CCFLAGS) -c $? -o $(?:.cpp=.o)

dynamic: $(OBJS) pre-build
	$(LD) -shared $(CCFLAGS) -o lib/librtmath.so $(OBJS)

static: $(OBJS) pre-build
	$(AR) rcs lib/librtmath.a $(OBJS)

FORCE:

# Compiles regular code along with test files
test: dynamic $(OBJSTEST)
	$(LD) -ggdb -lboost_unit_test_framework $(CCFLAGS) $(LIBS) -o bin/rtmath-test $(OBJSTEST) -lrtmath
	./bin/rtmath-test --log-level=test_suite --report_level=short


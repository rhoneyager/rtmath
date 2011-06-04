CC=g++
FILES=`ls *.cpp`
OFILES=matrixop.o phaseFunc.o polynomial.o quadrature.o rtmath-base.o zeros.o
#CFLAGS=-rdynamic -shared -fPIC -Wall 
CFLAGS=-Wall -shared -fPIC
#LFLAGS=-Wl,export-dynamic -lm
LFLAGS=-lm -lgcc_eh -lstdc++ --export-all-symbols


all : static dynamic

static : $(OFILES)
	ar rcs librtmathbase.a $(OFILES)

dynamic : $(OFILES)
	$(CC) $(CFLAGS) $(LFLAGS) -o librtmathbase.so
#	ld -G $(OFILES) $(LFLAGS) -o librtmathbase.so

clean :
	rm -f *.o *.a *.so *.so.1

%.o : %.cpp
	$(CC) $(CFLAGS) -c $< -o $@


CC=g++
FILES=`ls *.cpp`
OFILES=matrixop.o phaseFunc.o polynomial.o quadrature.o rtmath-base.o zeros.o
CFLAGS=-rdynamic -shared -fPIC -Wall 
LFLAGS=-Wl,export-dynamic -lm


all : static dynamic

static : $(OFILES)
	ar rcs librtmathbase.a $(OFILES)

dynamic : $(OFILES)
	ld -G $(OFILES) -o librtmathbase.so

clean :
	rm -f *.o *.a *.so *.so.1

%.o : %.cpp
	$(CC) $(CFLAGS) -c $< -o $@


CC=g++
FILES=`ls *.cpp`
OFILES=atmos.o damatrix.o damatrix_quad.o debug.o layer.o lbl.o rtmath.o
CFLAGS=-rdynamic -shared -fPIC -Wall 
LFLAGS=-Wl,export-dynamic -lm


all : static dynamic

static : $(OFILES)
	ar rcs librtmath.a $(OFILES)

dynamic : $(OFILES)
	ld -G $(OFILES) -o librtmath.so

clean :
	rm -f *.o *.a *.so *.so.1

%.o : %.cpp
	$(CC) $(CFLAGS) -c $< -o $@


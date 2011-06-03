CC=g++
FILES=`ls *.cpp`
OFILES=Qcalc.o rayleighPhaseFunc.o Scalc.o
CFLAGS=-rdynamic -shared -fPIC -Wall 
LFLAGS=-Wl,export-dynamic -lm


all : static dynamic

static : $(OFILES)
	ar rcs librayleigh.a $(OFILES)

dynamic : $(OFILES)
	ld -G $(OFILES) -o librayleigh.so

clean :
	rm -f *.o *.a *.so *.so.1

%.o : %.cpp
	$(CC) $(CFLAGS) -c $< -o $@


CC=g++
FILES=`ls *.cpp`
OFILES=abNCalc.o AnCalc.o mie.o miePhaseFunc.o piNCalc.o Qcalc.o Scalc.o tauNCalc.o wnCalc.o
CFLAGS=-rdynamic -shared -fPIC -Wall 
LFLAGS=-Wl,export-dynamic -lm


all : static dynamic

static : $(OFILES)
	ar rcs libmie.a $(OFILES)

dynamic : $(OFILES)
	ld -G $(OFILES) -o libmie.so

clean :
	rm -f *.o *.a *.so *.so.1

%.o : %.cpp
	$(CC) $(CFLAGS) -c $< -o $@


# Makefile for rtmath 
# Makes the library, tests and applications
PD = `pwd`

all: rtmath test apps

clean:

test: rtmath
	@cd rtmath
	@make -f rtmath/Makefile test
	@cd ../

apps: rtmath
	@make -f rtmath-apps/Makefile all

rtmath:
	@make -f rtmath/Makefile all



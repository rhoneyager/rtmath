# Makefile for rtmath 
# Makes the library, tests and applications
PD = `pwd`

all: rtmath test apps

clean:
	@cd $(PD)
	@cd rtmath-apps
	@make clean
	@cd ../rtmath/
	@make clean

test: rtmath
	@cd $(PD)
	@cd rtmath
	@make test
	@cd $(PD)

apps: rtmath
	@cd $(PD)
	@cd rtmath-apps
	@make all
	@cd $(PD)

rtmath:
	@cd $(PD)
	@cd rtmath
	@make all
	@cd $(PD)



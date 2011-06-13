#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc.exe
CCC=g++.exe
CXX=g++.exe
FC=gfortran.exe
AS=as.exe

# Macros
CND_PLATFORM=MinGW-Windows
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/zeros.o \
	${OBJECTDIR}/rtmath-base.o \
	${OBJECTDIR}/quadrature.o \
	${OBJECTDIR}/matrixop.o \
	${OBJECTDIR}/phaseFunc.o \
	${OBJECTDIR}/polynomial.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=-fvisibility=default
CXXFLAGS=-fvisibility=default

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath-base.dll

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath-base.dll: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -shared -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath-base.dll ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/zeros.o: zeros.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall  -MMD -MP -MF $@.d -o ${OBJECTDIR}/zeros.o zeros.cpp

${OBJECTDIR}/rtmath-base.o: rtmath-base.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall  -MMD -MP -MF $@.d -o ${OBJECTDIR}/rtmath-base.o rtmath-base.cpp

${OBJECTDIR}/quadrature.o: quadrature.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall  -MMD -MP -MF $@.d -o ${OBJECTDIR}/quadrature.o quadrature.cpp

${OBJECTDIR}/matrixop.o: matrixop.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall  -MMD -MP -MF $@.d -o ${OBJECTDIR}/matrixop.o matrixop.cpp

${OBJECTDIR}/phaseFunc.o: phaseFunc.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall  -MMD -MP -MF $@.d -o ${OBJECTDIR}/phaseFunc.o phaseFunc.cpp

${OBJECTDIR}/polynomial.o: polynomial.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall  -MMD -MP -MF $@.d -o ${OBJECTDIR}/polynomial.o polynomial.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath-base.dll

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc

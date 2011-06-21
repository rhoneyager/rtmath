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
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
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
	${OBJECTDIR}/damatrix_quad.o \
	${OBJECTDIR}/matrixop.o \
	${OBJECTDIR}/phaseFunc.o \
	${OBJECTDIR}/atmos.o \
	${OBJECTDIR}/rtmath.o \
	${OBJECTDIR}/error.o \
	${OBJECTDIR}/Stdafx.o \
	${OBJECTDIR}/polynomial.o \
	${OBJECTDIR}/debug_mem.o \
	${OBJECTDIR}/debug.o \
	${OBJECTDIR}/lbl.o \
	${OBJECTDIR}/damatrix.o \
	${OBJECTDIR}/layer.o


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
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.a: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.a
	${AR} -rv ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.a ${OBJECTFILES} 
	$(RANLIB) ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.a

${OBJECTDIR}/zeros.o: nbproject/Makefile-${CND_CONF}.mk zeros.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/zeros.o zeros.cpp

${OBJECTDIR}/rtmath-base.o: nbproject/Makefile-${CND_CONF}.mk rtmath-base.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/rtmath-base.o rtmath-base.cpp

${OBJECTDIR}/quadrature.o: nbproject/Makefile-${CND_CONF}.mk quadrature.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/quadrature.o quadrature.cpp

${OBJECTDIR}/damatrix_quad.o: nbproject/Makefile-${CND_CONF}.mk damatrix_quad.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/damatrix_quad.o damatrix_quad.cpp

${OBJECTDIR}/matrixop.o: nbproject/Makefile-${CND_CONF}.mk matrixop.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/matrixop.o matrixop.cpp

${OBJECTDIR}/phaseFunc.o: nbproject/Makefile-${CND_CONF}.mk phaseFunc.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/phaseFunc.o phaseFunc.cpp

${OBJECTDIR}/atmos.o: nbproject/Makefile-${CND_CONF}.mk atmos.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/atmos.o atmos.cpp

${OBJECTDIR}/rtmath.o: nbproject/Makefile-${CND_CONF}.mk rtmath.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/rtmath.o rtmath.cpp

${OBJECTDIR}/error.o: nbproject/Makefile-${CND_CONF}.mk error.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/error.o error.cpp

${OBJECTDIR}/Stdafx.o: nbproject/Makefile-${CND_CONF}.mk Stdafx.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/Stdafx.o Stdafx.cpp

${OBJECTDIR}/polynomial.o: nbproject/Makefile-${CND_CONF}.mk polynomial.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/polynomial.o polynomial.cpp

${OBJECTDIR}/debug_mem.o: nbproject/Makefile-${CND_CONF}.mk debug_mem.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/debug_mem.o debug_mem.cpp

${OBJECTDIR}/debug.o: nbproject/Makefile-${CND_CONF}.mk debug.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/debug.o debug.cpp

${OBJECTDIR}/lbl.o: nbproject/Makefile-${CND_CONF}.mk lbl.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/lbl.o lbl.cpp

${OBJECTDIR}/damatrix.o: nbproject/Makefile-${CND_CONF}.mk damatrix.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/damatrix.o damatrix.cpp

${OBJECTDIR}/layer.o: nbproject/Makefile-${CND_CONF}.mk layer.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -DHEAP_CHECK -D_DEBUG -MMD -MP -MF $@.d -o ${OBJECTDIR}/layer.o layer.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.a

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc

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
	${OBJECTDIR}/damatrix_quad.o \
	${OBJECTDIR}/atmos.o \
	${OBJECTDIR}/rtmath.o \
	${OBJECTDIR}/Stdafx.o \
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
LDLIBSOPTIONS=../rtmath-base/dist/Debug/GNU-Linux-x86/librtmath-base.dll

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.so

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.so: ../rtmath-base/dist/Debug/GNU-Linux-x86/librtmath-base.dll

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.so: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} --enable-auto-import -shared -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.so -fPIC ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/damatrix_quad.o: damatrix_quad.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/damatrix_quad.o damatrix_quad.cpp

${OBJECTDIR}/atmos.o: atmos.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/atmos.o atmos.cpp

${OBJECTDIR}/rtmath.o: rtmath.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/rtmath.o rtmath.cpp

${OBJECTDIR}/Stdafx.o: Stdafx.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/Stdafx.o Stdafx.cpp

${OBJECTDIR}/debug_mem.o: debug_mem.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/debug_mem.o debug_mem.cpp

${OBJECTDIR}/debug.o: debug.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/debug.o debug.cpp

${OBJECTDIR}/lbl.o: lbl.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/lbl.o lbl.cpp

${OBJECTDIR}/damatrix.o: damatrix.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/damatrix.o damatrix.cpp

${OBJECTDIR}/layer.o: layer.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -g -Wall -fPIC  -MMD -MP -MF $@.d -o ${OBJECTDIR}/layer.o layer.cpp

# Subprojects
.build-subprojects:
	cd ../rtmath-base && ${MAKE}  -f Makefile CONF=Debug
	cd ../rtmath-base && ${MAKE}  -f Makefile CONF=Debug

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/librtmath.so

# Subprojects
.clean-subprojects:
	cd ../rtmath-base && ${MAKE}  -f Makefile CONF=Debug clean
	cd ../rtmath-base && ${MAKE}  -f Makefile CONF=Debug clean

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc

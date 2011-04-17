#******************************************************************************
#  Makefile para a compilação
#		Oscar E. Ramos Ponce - UFRGS, Brasil 2011
#******************************************************************************

DRAWLIBS = -L${HOME}/ode-0.11.1/drawstuff/src/.libs -ldrawstuff
ODELIBS  = `pkg-config ode --libs` -lGLU  

CMP = g++
ODEFLAGS = `pkg-config ode --cflags` 
CPPINCLUDE = -I./include
CPPFLAGS =  -Wall

CMPFLAGS = ${CPPFLAGS} ${ODEFLAGS} ${CPPINCLUDE}
LDFLAGS  = ${ODELIBS} ${DRAWLIBS} 

FUNC = robPrimitives.o robModel.o robKinematics.o robMathAid.o robSteps.o

all: robSimpleSteps.o ${FUNC}
	${CMP} ${CMPFLAGS} -o robotsimu robSimpleSteps.o ${FUNC} ${LDFLAGS}  

#rob01ShowModel.o: ../tests/rob01ShowModel.cpp
#	${CMP} ${CMPFLAGS} -c ../tests/rob01ShowModel.cpp ${LDFLAGS}  

robSimpleSteps.o:
	${CMP} ${CMPFLAGS} -c ../tests/robSimpleSteps.cpp ${LDFLAGS}  

#----------------------------------------------------------------------------
#  		SOURCE FILES
#----------------------------------------------------------------------------

robPrimitives.o:
	${CMP} ${CMPFLAGS} -c ../src/robPrimitives.cpp

robModel.o:
	${CMP} ${CMPFLAGS} -c ../src/robModel.cpp

robKinematics.o: 
	${CMP} ${CMPFLAGS} -c ../src/robKinematics.cpp

robMathAid.o:
	${CMP} ${CMPFLAGS} -c ../src/robMathAid.cpp

robSteps.o:
	${CMP} ${CMPFLAGS} -c ../src/robSteps.cpp

#----------------------------------------------------------------------------

clean:
	rm -f  *~ *.bak *.o

distclean: clean
	rm -f robotsimu
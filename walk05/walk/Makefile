CURRENT_MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
WALK_BASE_PATH := $(abspath $(dir $(CURRENT_MAKEFILE_PATH))..)

CC=gcc
CFLAGS=-O3 -DWALK_BASE_PATH="\"$(WALK_BASE_PATH)\""
XLIBS = -lX11 -lGL -lGLU -lstdc++

BIPED2 = dynamics.o sdfast/b1g_dyn.o sdfast/b1g_sar.o sdfast/b1glib.o
BIPED2I = dynamics-impact.o sdfast/b1ss_dyn.o sdfast/b1ss_sar.o sdfast/b1sslib.o sdfast/b1ds_dyn.o sdfast/b1ds_sar.o sdfast/b1dslib.o sdfast/b1g_dyn.o sdfast/b1g_sar.o sdfast/b1glib.o lu.o

.PHONY: all clean

all: simulate animate simulatei learn

simulate: simulate.o parameters.o controller.o data.o $(BIPED2) ../useful/trajectory/trajectory.o ../useful/mrdplot/mrdplot.o
	$(CC) $(CFLAGS) -o $@ simulate.o parameters.o controller.o data.o $(BIPED2) ../useful/trajectory/trajectory.o ../useful/mrdplot/mrdplot.o -lm

simulatei: simulate.o parameters.o controller.o data.o $(BIPED2I) ../useful/trajectory/trajectory.o ../useful/mrdplot/mrdplot.o
	$(CC) $(CFLAGS) -o $@ simulate.o parameters.o controller.o data.o $(BIPED2I) ../useful/trajectory/trajectory.o ../useful/mrdplot/mrdplot.o -lm 
# ~/seegrid/projects/useful/numerical-recipes-c/librecipes_c.a -lm

animate.o: animate.c main.h
	$(CC) -c $(CFLAGS) -Wall -fno-exceptions -fomit-frame-pointer -ffast-math -I../useful/drawstuff-linux -DdNODEBUG -o animate.o animate.c

animate: animate.o parameters.o ../useful/mrdplot/mrdplot.o
	$(CC) $(CFLAGS) -o $@ animate.o parameters.o ../useful/mrdplot/mrdplot.o ../useful/drawstuff-linux/libdrawstuff.a $(XLIBS) -lm

learn: learn.o parameters.o controller.o data.o $(BIPED2) ../useful/trajectory/trajectory.o ../useful/mrdplot/mrdplot.o
	$(CC) $(CFLAGS) -o $@ learn.o parameters.o controller.o data.o $(BIPED2) ../useful/trajectory/trajectory.o ../useful/mrdplot/mrdplot.o -lm

animate.o:	main.h main2.h
controller.o:	main.h main2.h
data.o:	main.h main2.h
dynamics.o:	main.h main2.h
dynamics-impact.o:	main.h main2.h
parameters.o:	main.h main2.h
simulate.o:	main.h main2.h
learn.o:	main.h main2.h

clean: 
	$(RM) simulate simulatei animate learn *.o

# makefile
# VRP to SNT.
# Autor: xdocek09

CC=g++
CFLAGS=-pedantic -Wall -Wextra -std=c++11 -O2 -g
vrpWC: CFLAGS += -D NO_CANDIDATES 
vrpDis: CFLAGS += -D VIS_DISTANCE
vrpW2O: CFLAGS += -D NO_TWO_OPT

PROGS=vrp

all: $(PROGS)
.PHONY: all clean

%.o: src/%.cpp 
	$(CC) $(CFLAGS) -c $< -o $@ 

vrp: VRP.o ACO.o main.o
	$(CC) $(CFLAGS) $^ -o $@ -lm
	
vrpWC: vrp
vrpDis: vrp
vrpW2O: vrp

	
clean:
	rm -f *.o $(PROGS)

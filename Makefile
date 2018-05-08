# makefile
# VRP to SNT.
# Autor: xdocek09

CC=g++
CFLAGS=-pedantic -Wall -Wextra -std=c++11 -O2
PROGS=vrp

all: $(PROGS)
.PHONY: doc all clean

%.o: src/%.cpp 
	$(CC) $(CFLAGS) -c $< -o $@ 

vrp: VRP.o ACO.o main.o
	$(CC) $(CFLAGS) $^ -o $@ -lm

clean:
	rm -f *.o $(PROGS)

#!/bin/sh
make clean
make

for r in $(seq 1 14); do
  rm -f "exp/orig/res$r.txt"
done

for r in $(seq 1 10); do
  for i in $(seq 1 14); do
    ./vrp "./solve/vrpnc$i.txt" | tail -n 1 >> "exp/orig/res$i.txt"
  done
done

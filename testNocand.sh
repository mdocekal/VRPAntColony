#!/bin/sh
make clean
make vrpWC

for r in $(seq 1 10); do
  rm -f "exp/nocand/res$r.txt"
done

for r in $(seq 1 10); do
  for i in $(seq 1 14); do
    ./vrp "./solve/vrpnc$i.txt" | tail -n 1 >> "exp/nocand/res$i.txt"
  done
done

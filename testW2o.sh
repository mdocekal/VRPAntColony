#!/bin/sh
make clean
make vrpW2O

for r in $(seq 1 10); do
  rm -f "exp/w2o/res$r.txt"
done

for r in $(seq 1 10); do
  for i in $(seq 1 14); do
    ./vrp "./solve/vrpnc$i.txt" | tail -n 1 >> "exp/w2o/res$i.txt"
  done
done

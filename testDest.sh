#!/bin/sh
make clean
make vrpDis

for r in $(seq 1 10); do
  rm -f "exp/dest/res$r.txt"
done

for r in $(seq 1 10); do
  for i in $(seq 1 14); do
    ./vrp "./solve/vrpnc$i.txt" | tail -n 1 >> "exp/dest/res$i.txt"
  done
done

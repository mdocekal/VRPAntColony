#!/usr/bin/env python3

import sys

vals=[]
for line in sys.stdin:
    vals.append(float(line.strip()))
print(round(min(vals),2))
print(round(sum(vals)/len(vals),2))

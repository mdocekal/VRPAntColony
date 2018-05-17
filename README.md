# VRPAntColony
Vehicle routing problem with Ant colony optimization.

# Instalation
For full implementation simply run:

    make

Implementation without 2-opt:

    make vrpW2O

Implementation without candidates list:

    make vrpWC


Implementation with alternative visibility:

    make vrpDis

# Run
It is simple one argument program so:

    ./vrp PATH_TO_PROBLEM



# Input format and Tests

Folder solve contains fourteen benchmark downloaded from: http://neo.lcc.uma.es/vrp/vrp-instances/capacitated-vrp-instances/ .

Format of input file must be the same as format of these files.

# Experiments
Folder exp contains results of experiments runed with scripts test*.sh

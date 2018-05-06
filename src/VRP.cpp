/**
 * Project: VRPAntColony
 * @file VRP.cpp
 * @date 6. 5. 2018
 * @author xdocek09
 * @brief  Source file for representation of vehicle routing problem.
 */

#include "VRP.h"
#include <string>
#include <sstream>

VRP::VRP(std::istream& input) {
	std::string line;
	std::stringstream sLine;
	//first line consists of:
	//	number of customers, vehicle capacity, maximum route time, drop time

	unsigned numberOfCustomers;
	if(std::getline(input, line)){
		sLine<<line;
		sLine >> numberOfCustomers;
		sLine >> vehicleCapacity;
		sLine >> maxRouteTime;
		sLine >> dropTime;
		sLine.clear();
	}else{
		throw std::runtime_error("VRP: invalid input. No first line.");
	}
	//second line haves depot coordinates
	if(std::getline(input, line)){
		//depot x-coordinate, depot y-coordinate
		depot.id=0;
		depot.type=EnityType::DEPOT;
		depot.quantity=0;
		sLine<<line;
		sLine >> depot.p.x;
		sLine >> depot.p.y;
		sLine.clear();
	}else{
		throw std::runtime_error("VRP: invalid input. No depot line.");
	}


	//get the customers
	for(; std::getline(input, line) && numberOfCustomers>0; numberOfCustomers--){
		//for each customer in turn: x-coordinate, y-coordinate, quantity
		Entity c;
		c.id=customers.size()+1;	//+1 because 0 is depot
		c.type=EnityType::CUSTOMER;
		sLine<<line;
		sLine >> c.p.x;
		sLine >> c.p.y;
		sLine >> c.quantity;

		customers.push_back(c);
		sLine.clear();
	}

	if(numberOfCustomers>0)
		throw std::runtime_error("VRP: invalid input. Number of customers doesn't match.");

}

/*** End of file: VRP.cpp ***/

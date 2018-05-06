/**
 * Project: VRPAntColony
 * @file VRP.h
 * @date 6. 5. 2018
 * @author xdocek09
 * @brief Header file for representation of vehicle routing problem.
 */ 

#ifndef VRP_H_
#define VRP_H_
#include <iostream>
#include <vector>
/**
 * Structure for coordinates.
 */
struct Point{
    unsigned x, y;
};

enum class EnityType{
	DEPOT, CUSTOMER
};

/**
 * Structure for customer.
 */
struct Entity{
	unsigned id;
	EnityType type;
    Point p;	//! customers coordinates
    unsigned quantity;
};

/**
 * Represents vehicle routing problem
 */
class VRP {
public:

	/**
	 * Creates empty VRP.
	 */
	VRP(){};

	/**
	 * Loads VRP from the stream.
	 *
	 * @param[in] input
	 * 	The VRP.
	 * @throw std::runtime_error on invalid input
	 */
	VRP(std::istream& input);

	const std::vector<Entity>& getCustomers() const {
		return customers;
	}

	const Entity& getDepot() const {
		return depot;
	}

	unsigned getDropTime() const {
		return dropTime;
	}

	unsigned getMaxRouteTime() const {
		return maxRouteTime;
	}

	unsigned getVehicleCapacity() const {
		return vehicleCapacity;
	}

	/**
	 * Distance between two customers.
	 *
	 * @param[in] a
	 * 	First customer.
	 * @param[in] b
	 * 	Second customer.
	 * @return distance
	 */
	static double distance(const Entity& a, const Entity& b){
		int diffX=a.p.x-b.p.x;
		int diffY=a.p.y-b.p.y;

		return diffX*diffX + diffY*diffY;
	}

private:
	unsigned vehicleCapacity=0;
	unsigned maxRouteTime=0;
	unsigned dropTime=0;


	Entity depot;

	std::vector<Entity> customers;


};

#endif /* VRP_H_ */

/*** End of file: VRP.h ***/

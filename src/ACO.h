/**
 * Project: VRPAntColony
 * @file ACO.h
 * @date 6. 5. 2018
 * @author xdocek09
 * @brief Header file of ant colony optimization for vehicle routing problem.
 */

#ifndef ACO_H_
#define ACO_H_

#include <set>
#include <memory>
#include <vector>
#include <random>
#include "VRP.h"


struct Vertex;
/**
 * Representation of arc.
 */
struct Arc{
	double pheromone;	//! Actual pheromone
	double visibility;  //! Pre calculated visibility.
	double distance;	//! distance between vertices
	std::set<Vertex*> v;	//! Vertices

};

/**
 * Customer or depot.
 */
struct Vertex{
	const Entity* c;			//! customer assigned to vertex
	std::vector<Arc*> candidates;	//! candidates for visiting
};

class ACO;
/**
 * Representation of one ant.
 */
class Ant{
public:
	/**
	 * Ant initialization.
	 *
	 * @param[in] iV
	 * 	Init vertex.
	 */
	Ant(const Vertex* iV, ACO* aco):filledCapacity(0),time(0),parentACO(aco) {
		route.push_back(iV);
		randGen.seed(std::random_device()());
		dist(0,1);
	};

	/**
	 * Generate new solution.
	 */
	void genNewSolution();


private:
	std::vector<const Vertex*> route;	//! Already visted vertices. In vist order.
	unsigned filledCapacity;
	double time; //!time on route
	ACO* parentACO;
	std::mt19937 randGen;
	std::uniform_real_distribution<double> dist;
	/**
	 * Selects next vertex to visit.
	 *
	 * @return
	 * 	Vertex to next visit. Nullptr in case of failure(no feasible vertex).
	 */
	Vertex* nextVisist();

};


/**
 * Solver for vehicle routing problem. Uses Ant colony optimization discribed in paper:
 * An improved Ant System algorithm for the Vehicle Routing Problem
 * 	Paper, autors: Bernd Bullnheimer, Richard F. Hartl and Christine Strauss
 *
 */
class ACO {
public:

	/**
	 * ACO intitialization.
	 *
	 * @param[in] v
	 * 	VRP problem.
	 */
	ACO(VRP& v);

	/**
	 * Solves VRP for given problem.
	 * @param[in] iterations
	 * 	Number of iterations.
	 */
	void solve(const unsigned iterations);

	double getAlfa() const {
		return alfa;
	}

	void setAlfa(double alfa = 5) {
		this->alfa = alfa;
	}

	const std::vector<Ant>& getAnts() const {
		return ants;
	}

	void setAnts(const std::vector<Ant>& ants) {
		this->ants = ants;
	}

	const std::vector<Arc>& getArcs() const {
		return arcs;
	}

	void setArcs(const std::vector<Arc>& arcs) {
		this->arcs = arcs;
	}

	double getBeta() const {
		return beta;
	}

	void setBeta(double beta = 5) {
		this->beta = beta;
	}

	unsigned getElitAnts() const {
		return elitAnts;
	}

	void setElitAnts(unsigned elitAnts = 6) {
		this->elitAnts = elitAnts;
	}

	double getF() const {
		return f;
	}

	void setF(double f = 2) {
		this->f = f;
		arcCreate();
	}

	double getG() const {
		return g;
	}

	void setG(double g = 2) {
		this->g = g;
		arcCreate();
	}

	unsigned getNumOfCandidates() const {
		return numOfCandidates;
	}

	void setNumOfCandidates(unsigned numOfCandidates) {
		this->numOfCandidates = numOfCandidates;
		arcCreate();
	}

	double getRo() const {
		return ro;
	}

	void setRo(double ro = 0.75) {
		this->ro = ro;
	}

	const std::vector<Vertex>& getVertices() const {
		return vertices;
	}

	void setVertices(const std::vector<Vertex>& vertices) {
		this->vertices = vertices;
	}

	const VRP& getVrp() const {
		return vrp;
	}

private:
	VRP vrp;

	//TODO: boundaries check in setters
	//default values are set according to given paper.
	double alfa=5; //! Impact parameter of pheromone.
	double beta=5; //! Impact parameter of visibility.
	double ro=0.75;	//! Trail persistance. Trail evaporation is (1-ro). Number in interval [0,1]
	double f=2;	//! Number greater or equal than zero. Parameter used for visibility calculation.
	double g=2;	//! Number greater than one. Parameter used for visibility calculation.
	unsigned elitAnts=6; //!Number of selected elitist ants.
	unsigned numOfCandidates; //!Maximum number of candidates. (Without the depot)


	std::vector<Vertex> vertices;	//First vertex in vector is depot
	std::vector<Arc> arcs;
	std::vector<Ant> ants;

	/**
	 * Creates arcs.
	 */
	void arcCreate();
};

#endif /* ACO_H_ */

/*** End of file: ACO.h ***/

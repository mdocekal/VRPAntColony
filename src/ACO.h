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


class Vertex;
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
class Vertex{
public:
	const Entity* c;			//! customer assigned to vertex
	std::vector<Arc*> candidates;	//! candidates for visiting

	/**
	 * Calculates distance to depot.
	 *
	 * @return Distance to depot.
	 * @throw std::runtime_error	Vertex has not depot in candidates.
	 */
	double distToDepot() const{
		for(auto a : candidates){
			for(auto v: a->v){
				if(v->c->type==EnityType::DEPOT){
					//we can compare distance with time because we are assuming that
					//vehicle velocity is one distance unit per one time unit
					return a->distance;
				}
			}
		}
		//if we are here than something is rotten in the state of this program
		throw std::runtime_error("Unexpected error when finding distance to depot from vertex.");
	}

	/**
	 * Calculates distance from this vertex to given vertex.
	 *
	 * @param[in] v
	 * 	Distance to this vertex.
	 * @return Distance to vertex.
	 * @throw std::runtime_error	Unexpected error when finding distance.
	 */

	double distToVertex(const Vertex& v) const{
		return VRP::distance(*c, *(v.c));
	}

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
	 * 	Init vertex. According this vertex will be set initial filledCapacity and time.
	 */
	Ant(const Vertex* iV, ACO* aco);

	/**
	 * Generates solution for given problem.
	 * @throw std::runtime_error When maximum route time is too small.
	 */
	std::vector<const Vertex*> genSolution();

private:
	const Vertex* initVertex;
	std::vector<const Vertex*> route;	//! Already visited vertices. In visit order.
	unsigned filledCapacity=0;
	double time=0; //!time on route
	ACO* parentACO;
	std::mt19937 randGen;
	std::uniform_real_distribution<double> dist;

	std::set<const Vertex*> tabu;	//! Already visited customers. We can use route but set is faster for finding elements in it

	/**
	 * Finds next vertex to visit.
	 *
	 * @return
	 * 	Vertex to next visit. Nullptr in case of failure(no feasible vertex).
	 * @throw std::runtime_error	Unexpected error when selecting next vertex.
	 */
	const Vertex* nextVisit();

	/**
	 * Ant goes to depot.
	 */
	void returnToDepot();
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
	 * ACO initialization.
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
		arcCreate();
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

	/**
	 * Calculates cost of route.
	 *
	 * @param[in] solution
	 * 	Solution for measure.
	 * @return
	 *  solution cost
	 */
	double solutionCost(const std::vector<const Vertex*>& solution) const{
		return solutionCost(solution, 0, solution.size()-1);
	}

	/**
	 * Calculates cost of route. For given index bounds. [x,y]
	 *
	 * @param[in] solution
	 * 	Solution for measure.
	 * @param[in] x
	 * 	Start from this index.
	 * @param[in] y
	 *  End on this index.
	 * @return
	 *  solution cost
	 */
	double solutionCost(const std::vector<const Vertex*>& solution, const unsigned x, const unsigned y) const {
		double time =0; //for depot there is no drop time
		if (solution[x]->c->type == EnityType::CUSTOMER)
			time += vrp.getDropTime();

		for (unsigned i = x; i < y; ++i) {
			time += solution[i]->distToVertex(*solution[i+1]);

			if (solution[i+1]->c->type == EnityType::CUSTOMER)
				time += vrp.getDropTime();
		}

		return time;
	}


	/**
	 * Tries to optimize solution with two opt heuristic.
	 *
	 * @param[in|out] solution
	 * 	Solution for optimization.
	 */
	void twoOpt(std::vector<const Vertex*>& solution) const;

	/**
	 * Get solution.
	 *
	 * @return Solution
	 */
	const std::pair<double, std::vector<const Vertex*> >& getBestSoFar() const {
		return bestSoFar;
	}

private:
	VRP vrp;

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

	std::pair<double, std::vector<const Vertex*>> bestSoFar; //! so far the best solution searched


	/**
	 * Select arc with given vertex.
	 *
	 * @param[in] v
	 * 	Vertex for arc searching.
	 * @param[in] u
	 * 	Other vertex for arc searching.
	 * @return
	 * 	Corepsonding arc. Nullptr when arc with given vertex is not in candidates.
	 */
	Arc* selectArc(const Vertex& v, const Vertex& u){

		for(Arc& a: arcs){
			bool one=false;
			for(const Vertex* aV: a.v){
				if(aV->c->id==v.c->id || aV->c->id==u.c->id){
					if(one) return &a;
					one=true;
				}
			}
		}

		return nullptr;
	}

	/**
	 * Creates arcs.
	 */
	void arcCreate();
};

#endif /* ACO_H_ */

/*** End of file: ACO.h ***/

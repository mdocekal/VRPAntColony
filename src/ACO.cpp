/**
 * Project: VRPAntColony
 * @file ACO.cpp
 * @date 6. 5. 2018
 * @author xdocek09
 * @brief  Header file of ant colony optimization for vehicle routing problem.
 */

#include "ACO.h"
#include <algorithm>
#include <cstdlib>
#include <cmath>

ACO::ACO(VRP& v) :vrp(v) {

	//create vertices
	//depot is first
	Vertex d;
	d.c=&(vrp.getDepot());
	vertices.push_back(d);

	//than all customers
	for(const Entity& c : vrp.getCustomers()){
		Vertex v;
		v.c=&c;
		vertices.push_back(v);
	}

	//according to paper n/4
	numOfCandidates=(vertices.size()-1)/4;
	arcCreate();




}

void ACO::arcCreate(){
	//clear old arcs
	arcs.clear();
	for(auto v: vertices) v.candidates.clear();

	for (int i = 0; i < static_cast<int>(vertices.size()) - 1; i++) {
		for (int j = i + 1; i < static_cast<int>(vertices.size()) - 1; j++) {

			Arc a;
			a.pheromone=1;
			a.distance=VRP::distance(*vertices[i].c, *vertices[j].c);
			a.v.insert(&vertices[i]);
			a.v.insert(&vertices[j]);

			//lets calc visibility
			a.visibility=VRP::distance(*vertices[i].c, *vertices[0].c)+VRP::distance(*vertices[j].c, *vertices[0].c)
				-g*VRP::distance(*vertices[i].c, *vertices[j].c)
				+f*abs(VRP::distance(*vertices[i].c, *vertices[0].c)-VRP::distance(*vertices[j].c, *vertices[0].c));

			//create the arc
			arcs.push_back(a);

			//set arc to vertices
			if(i!=0){ //depot we will insert later
				vertices[i].candidates.push_back(&arcs[arcs.size() - 1]);
				vertices[j].candidates.push_back(&arcs[arcs.size() - 1]);
			}
		}
	}

	//ok now we have all arcs and their vertices
	//now its time to sort the arcs in vertices according to distances
	//and create list of vertices to visit

	for (Vertex& v : vertices) {
		std::sort(v.candidates.begin(), v.candidates.end(),
				[](const Arc*const a, const Arc*const b) -> bool
				{	return a->distance < b->distance;});

		//filter only the best ones as candidates
		unsigned selectN=numOfCandidates<=v.candidates.size()?numOfCandidates:v.candidates.size();
		v.candidates.resize(selectN);
	}




}

void ACO::solve(const unsigned iterations){
	//TODO: Check only one vetex solution. Further we asume more verticies.


	//iter
	for(unsigned i=0; i<iterations; i++ ){
		//init iter
		ants.clear();
		//acording to paper we are creating one ant at each customer
		for(unsigned i=1; i<vertices.size();i++){ //zero is depot
			ants.push_back(Ant(&vertices[vertices.size()-1], this));
		}
	}

}

void Ant::genNewSolution(){
	for(unsigned n=1; parentACO->getVrp().getCustomers().size(); n++){
		//we are starting from 1 because we have set initial position


	}
}

Vertex* Ant::nextVisist(){
	//filter feasible verticies
	std::vector<const Vertex*> vert;
	std::vector<const Arc*> arcs; //coresponding arcs to selected verticies
	for(auto a: route[route.size()-1]->candidates){//candidates from previous vertex
		for(const Vertex* v : a->v){
			if (v!=route[route.size()-1]){
				if(std::find(route.begin(), route.end(), v) == route.end() ){
					//Arc contains actual vertice and possible next vertice and we just filtered actual and
					//next is not a tabu.

					if(v->c->quantity+filledCapacity<=parentACO->getVrp().getVehicleCapacity()){
						//we have enought capacity

						if(VRP::distance(*(v->c), *(route[route.size()-1]->c))+time+parentACO->getVrp().getDropTime()
								<=parentACO->getVrp().getMaxRouteTime()){
							//we are in route time
							//we can compare distance with time because we are asuming that
							//vehicle velocity is one distance unit per one time unit
							vert.push_back(v);
							arcs.push_back(a);
						}

					}
				}
				break;
			}
		}
	}

	//we need to get probabilities for rulete
	if(vert.size()==0) return nullptr;

	//do sum
	double sum;
	for(auto a : arcs)
		sum+=std::pow(a->pheromone, parentACO->getAlfa())+std::pow(a->visibility, parentACO->getBeta());


	double shoot=dist(randGen); //load gun





	std::uniform_int_distribution<> dis { 0, 36 };


}


/*** End of file: ACO.cpp ***/

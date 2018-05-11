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

	//create ants
	//according to paper we are creating one ant at each customer
	for (unsigned i = 1; i < vertices.size(); i++) { //zero is depot
		ants.push_back(Ant(&vertices[i], this));
	}

}

void ACO::arcCreate(){
	//clear old arcs
	arcs.clear();
	arcs.reserve(vertices.size()*(1+vertices.size()-2)/2);	//sum of arithmetic series

	for(Vertex& v: vertices) v.candidates.clear();

	//select max

	for (int i = 0; i < static_cast<int>(vertices.size()) - 1; i++) {
		for (int j = i + 1; j < static_cast<int>(vertices.size()); j++) {

			Arc a;
			a.pheromone=1;

			a.distance=vertices[i].distToVertex(vertices[j]);

			a.v.insert(&vertices[i]);
			a.v.insert(&vertices[j]);

			//lets calc visibility

			/*
			 * This improvment from paper i commented out, because its values are in domain of greate number
			 * compared to pheromone. Therefore influence of feromone is wipped out.
			 * */
			a.visibility=vertices[i].distToVertex(vertices[0])+vertices[0].distToVertex(vertices[j])
				-g*vertices[i].distToVertex(vertices[j])
				+f*std::abs(vertices[i].distToVertex(vertices[0])-vertices[0].distToVertex(vertices[j]));



			//a.visibility=1/vertices[i].distToVertex(vertices[j]);

			a.visibility=std::pow(a.visibility, beta);

			std::cout << a.visibility <<std::endl;

			//create the arc
			arcs.push_back(a);

			//set arc to vertices
			vertices[i].candidates.push_back(&arcs[arcs.size() - 1]);
			vertices[j].candidates.push_back(&arcs[arcs.size() - 1]);

		}
	}


	//ok now we have all arcs and their vertices
	//now its time to sort the arcs in vertices according to distances
	//and create list of vertices to visit

	for (Vertex& v : vertices) {
		Arc* toDepot=nullptr;
		for (Arc* a : v.candidates) {

			for (const Vertex* actV : a->v) {
				if (actV->c->type == EnityType::DEPOT) {
					toDepot=a;
					break;
				}
			}
			if (toDepot!=nullptr)
				break;
		}

		std::sort(v.candidates.begin(), v.candidates.end(),
				[](const Arc*const a, const Arc*const b) -> bool
				{	return a->distance < b->distance;});

		//filter only the best ones as candidates
		unsigned selectN=numOfCandidates<=v.candidates.size()?numOfCandidates:v.candidates.size();
		if(selectN>v.candidates.size()){//if is greater or equal than depot must be in and also there is no need for resizing
			v.candidates.resize(selectN+1); //+1 because of reserve for depot
			bool containsDepot=false;
			//check if depot is in candidate list
			for(const Arc* a:v.candidates){
				for(const Vertex* actV: a->v){
					if(actV->c->type==EnityType::DEPOT){
						containsDepot=true;
						break;
					}
				}
				if(containsDepot)break;
			}

			if(!containsDepot){
				//depot is not in candidates
				v.candidates.pop_back();
				//add depot
				v.candidates.push_back(toDepot);
			}
		}
	}


}

void ACO::solve(const unsigned iterations){
	//TODO: Check only one vertex solution. Further we assume more vertices.

	bestSoFar.first=std::numeric_limits<double>::infinity();
	bestSoFar.second.clear();

	//iter
	for(unsigned i=0; i<iterations; i++ ){
		std::cout << "\t" <<i<<") INIT" <<std::endl;
		//init iter

		std::vector<std::vector<const Vertex*>> iterSolutions;

		std::vector<std::pair<double,unsigned>> sortedSolutions; //cost, solution index
		std::cout << "\t" <<i<<") create new solution" <<std::endl;
		//create new solution for each ant
		for(Ant& a: ants){
			iterSolutions.push_back(a.genSolution());//every ant creates its solution
			//lets try to improve solution with 2-opt heuristic
			twoOpt(iterSolutions.back());
			sortedSolutions.push_back(std::make_pair(solutionCost(iterSolutions.back()),sortedSolutions.size()));
		}
		std::cout << "\t" <<i<<") sort solutions" <<std::endl;
		//now we have from every ant one solution for given VRP
		//improved with 2-opt heuristic
		//now its time to select the elites ants and update pheromones on searched path

		std::sort(sortedSolutions.begin(), sortedSolutions.end(),
				[](const std::pair<double,unsigned>& a, const std::pair<double,unsigned>& b) -> bool{
					return a.first<b.first;
				}
		);

		if(sortedSolutions.size()>elitAnts){
			sortedSolutions.resize(elitAnts);
		}
		std::cout << "\t" <<i<<") select best so far" <<std::endl;
		if(bestSoFar.first>sortedSolutions[0].first){
			//we searched new best
			bestSoFar=std::make_pair(sortedSolutions[0].first, iterSolutions[sortedSolutions[0].second]);
		}
		std::cout << "\t" <<i<<") update pheromones" <<std::endl;
		//update pheromones
		/*std::cout << "\t\t";
						for(Arc& a: arcs){
							for(const Vertex* v : a.v){
								std::cout << v->c->id << " -> ";
							}
							std::cout << "(" << a.pheromone << "), ";
						}
						std::cout << std::endl;*/
		//evaporation
		for(Arc& a: arcs) a.pheromone=ro*a.pheromone;

		//increase pheromones of visited arcs
		for (unsigned mi = 0; mi < sortedSolutions.size() - 1; mi++) {//we don't want the last one
			double pDelta = (sortedSolutions.size() - (mi + 1))
					/ sortedSolutions[mi].first;
			std::cout << "\t\t UPDATE SOLUTION OF " << mi <<std::endl;
			for (unsigned vi = 0;
					vi < iterSolutions[sortedSolutions[mi].second].size() - 1;
					vi++) {
				//select arc for update
				Arc* a =iterSolutions[sortedSolutions[mi].second][vi]->selectCandidate(
						*(iterSolutions[sortedSolutions[mi].second][vi+1])
						);

				a->pheromone += pDelta;

			}

		}

		//increase pheromones for arcs that belongs to best solution so far
		double pDeltaBest=sortedSolutions.size()/bestSoFar.first;

		for (unsigned vi = 0;vi < bestSoFar.second.size() - 1;vi++) {
			//select arc for update
			Arc* a =bestSoFar.second[vi]->selectCandidate(*(bestSoFar.second[vi+1]));
			a->pheromone += pDeltaBest;

		}
		/*
		std::cout << "\t\t";
				for(Arc& a: arcs){
					for(const Vertex* v : a.v){
						std::cout << v->c->id << " -> ";
					}
					std::cout << "(" << a.pheromone << "), ";
				}
				std::cout << std::endl;

		for(auto& r: iterSolutions){
			for(auto v: r){
				std::cout << v->c->id << " -> ";
			}
			std::cout << "len: " << solutionCost(r) << std::endl;
		}*/
		std::cout << "BEST: ";
		for(auto v: bestSoFar.second){
						std::cout << v->c->id << " -> ";
					}
					std::cout << "len: " << bestSoFar.first << std::endl;

	}

}

Ant::Ant(const Vertex* iV, ACO* aco):initVertex(iV), parentACO(aco), dist(0,1) {}

std::vector<const Vertex*> Ant::genSolution(){
	route.clear();
	tabu.clear();

	//add init vertex and depot
	route.push_back(&(parentACO->getVertices()[0]));
	route.push_back(initVertex);
	randGen.seed(std::random_device()());
	filledCapacity=initVertex->c->quantity;

	//search distance from depot
	time=initVertex->distToDepot();
	//add drop time
	time+=parentACO->getVrp().getDropTime();
	//add it to tabu
	tabu.insert(initVertex);

	while(tabu.size()<parentACO->getVertices().size()-1){//if we need to visit some customer
		//find next visit
		const Vertex* nextVertex = nextVisit();

		if (nextVertex == nullptr) {
			//can not find feasible vertex
			//return to the depot
			//it finishes one vehicle route and start new for another vehicle
			double distToDepot;
			while (route.size()>0 && (distToDepot = route.back()->distToDepot()) + time
					> parentACO->getVrp().getMaxRouteTime()) {

				//we must return because distance to depot exceeds route length limit
				if(route.size()>1){
					time-=parentACO->getVrp().getDropTime();
					time-=route.back()->distToVertex(*(route[route.size()-2]));
					filledCapacity -= route.back()->c->quantity;
				}
				tabu.erase(route.back());
				route.pop_back();
			}
			//std::cout << "can not find feasible vertex" << std::endl;
			//std::cout << time << std::endl;
			//std::cout << parentACO->getVrp().getMaxRouteTime() << std::endl;
			if (route.size() > 0) {
				//add depot
				route.push_back(&(parentACO->getVertices()[0]));

				//we are in depot so we can reset these
				filledCapacity = 0;
				time = 0;
			}else{
				throw std::runtime_error("Maximum route time is too small. Vehicles can not visit some customers.");
			}

		} else {
			//next vertex was find
			route.push_back(nextVertex);
			//update capacity and time
			filledCapacity += nextVertex->c->quantity;
			//find distance
			time += nextVertex->distToVertex(*route[route.size() - 1]);
			//add drop time
			time += parentACO->getVrp().getDropTime();

			//add it to tabu
			tabu.insert(nextVertex);

		}
	}
	//return to depot
	route.push_back(&(parentACO->getVertices()[0]));

	return route;

}

const Vertex* Ant::nextVisit(){
	//filter feasible vertices
	std::vector<const Vertex*> vert;
	std::vector<const Arc*> arcs; //corresponding arcs to selected vertices
	for(const Arc* a: route.back()->candidates){//candidates from previous vertex
		for(const Vertex* v : a->v){
			if (v!=route.back()){
				if(tabu.find(v) == tabu.end() ){
					//Arc contains actual vertex and possible next vertex and we just filtered actual and
					//next is not a tabu.
					if(v->c->type!=EnityType::DEPOT){//we are skipping depot

						if(v->c->quantity+filledCapacity<=parentACO->getVrp().getVehicleCapacity()){
							//we have enough capacity

							if(VRP::distance(*(v->c), *(route.back()->c))+time+parentACO->getVrp().getDropTime()
									<=parentACO->getVrp().getMaxRouteTime()){
								//we are in route time
								//we can compare distance with time because we are assuming that
								//vehicle velocity is one distance unit per one time unit
								vert.push_back(v);
								arcs.push_back(a);
							}

						}
					}
				}
				break;
			}
		}
	}

	//we need to get probabilities for rulete
	if(vert.size()==0) return nullptr;

	//do sum of all probabilities
	double sum=0;

	std::vector<const Arc*> arcsUse; //last filter according to probability

	//std::cout << "ARCS "<< arcs.size() << std::endl;
	for(auto a : arcs){
		//std::cout << "P "<< a->pheromone << " V " << a->visibility << std::endl;
		//std::cout << "\tP "<< std::pow(a->pheromone, parentACO->getAlfa()) << " V " << std::pow(a->visibility, parentACO->getBeta()) << std::endl;
		double p=std::pow(a->pheromone, parentACO->getAlfa())*a->visibility;
		if(p>0){
			sum+=p; //power for visibility is precalculated
			arcsUse.push_back(a);
		}
	}

	if(arcsUse.size()==0) return nullptr; //we have arcs with zero or smaller probability only

	double shoot=dist(randGen); //load gun

	double probSum=0;

	unsigned i=0;



	for(auto a : arcsUse){

		probSum+=(std::pow(a->pheromone, parentACO->getAlfa())*a->visibility)/sum; //power for visibility is precalculated

		if(probSum>=shoot){
			//std::cout << "\tSELECTED "<< vert[i]->c->id << " FROM " << route.back()->c->id << " SHOOT " << shoot << " PROB SUM " << probSum << " SUM " << sum << std::endl;
			//we shoot in that range
			return vert[i];
		}
		i++;
	}
	std::cout << "PROB SUM " << probSum << std::endl;
	std::cout << "shoot " << shoot << std::endl;
	std::cout << "sum " << sum << std::endl;
	for(auto a : arcsUse){

			std::cout << "\t" << std::pow(a->pheromone, parentACO->getAlfa())*a->visibility << std::endl;
			std::cout << "\t\t";
			for(auto v: a->v){
				std::cout << v->c->id << ", ";
			}
			std::cout << std::endl;
			std::cout << "\t\t" << a->pheromone << "\t" << a->visibility << std::endl;
		}
	//if we are here than something is rotten in the state of this program
	throw std::runtime_error("Unexpected error when selecting next vertex.");

}


void ACO::twoOpt(std::vector<const Vertex*>& solution) const{
	//we are optimizing each vehicle route separately

	unsigned routeStart=1;//0 is depot that we can not switch
	for(unsigned i=1; i < solution.size();i++){
		if(solution[i]->c->type==EnityType::DEPOT){
			//end of one vehicle route

			//we can not swap depot so lets not calculate it in cost
			double bestCost=solutionCost(solution, routeStart, i-1);


		  IMPROVED_START_AGAIN://until no improvement is made
			for (unsigned x = routeStart; x < i; x++) {
				for (unsigned y = x + 1; y < i; y++) {
					std::vector<const Vertex*> newRoute;

					//swap
					//first part add in order
					for (unsigned s = routeStart; s < x; s++) {
						newRoute.push_back(solution[s]);
					}
					//middle part add in reverse order (swaping)
					for (unsigned s = y; s >= x; s--) {
						newRoute.push_back(solution[s]);
					}
					//middle part add in reverse order (swaping)
					for (unsigned s = y + 1; s < i; s++) {
						newRoute.push_back(solution[s]);
					}

					double actCost = solutionCost(newRoute);
					if (actCost < bestCost) {
						//change vehicle route
						for (unsigned s = routeStart; s < i; s++)
							solution[s] = newRoute[s - routeStart];
						bestCost = actCost;
						goto IMPROVED_START_AGAIN;
					}
				}
			}

			routeStart=i+1;
			i++;
		}

	}

}

/*** End of file: ACO.cpp ***/

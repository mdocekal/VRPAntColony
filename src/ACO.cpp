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
	numOfCandidates=vrp.getCustomers().size()/4;
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

	for(Vertex& v: vertices)v.candidates.clear();

	double visibilityMin=std::numeric_limits<double>::infinity();
	//select max
	for (int i = 0; i < static_cast<int>(vertices.size()) - 1; i++) {
		for (int j = i + 1; j < static_cast<int>(vertices.size()); j++) {

			Arc a;

			a.distance=vertices[i].distToVertex(vertices[j]);

			a.v.insert(&vertices[i]);
			a.v.insert(&vertices[j]);

			//lets calc visibility



			#ifndef VIS_DISTANCE
			a.visibility=vertices[i].distToVertex(vertices[0])+vertices[0].distToVertex(vertices[j])
				-g*a.distance
				+f*std::abs(vertices[i].distToVertex(vertices[0])-vertices[0].distToVertex(vertices[j]));
			#endif

			#ifdef VIS_DISTANCE
			a.visibility=1/a.distance;
			#endif

			if(visibilityMin>a.visibility) visibilityMin=a.visibility;


			//create the arc
			arcs.push_back(a);

			//set arc to vertices
			vertices[i].candidates.push_back(&arcs.back());
			vertices[j].candidates.push_back(&arcs.back());


		}
	}

	//shift visibility
	if(visibilityMin<=0){
		visibilityMin=(-visibilityMin)+1;
		for(Arc& a : arcs){
			a.visibility=std::pow(visibilityMin+a.visibility, beta);
		}
	}

	#ifndef NO_CANDIDATES

	//ok now we have all arcs and their vertices
	//now its time to sort the arcs in vertices according to distances
	//and create list of vertices to visit

	for (Vertex& v : vertices) {

		if (v.c->type == EnityType::DEPOT)
			continue;
		Arc* toDepot = nullptr;
		for (Arc* a : v.candidates) {

			for (const Vertex* actV : a->v) {
				if (actV->c->type == EnityType::DEPOT) {
					toDepot = a;
					break;
				}
			}
			if (toDepot != nullptr)
				break;
		}

		std::sort(v.candidates.begin(), v.candidates.end(),
				[](const Arc*const a, const Arc*const b) -> bool
				{	return a->distance < b->distance;});

		//filter only the best ones as candidates
		unsigned selectN =
				numOfCandidates <= v.candidates.size() ?
						numOfCandidates : v.candidates.size();
		if (selectN < v.candidates.size()) {//if is equal than depot must be in and also there is no need for resizing
			v.candidates.resize(selectN + 1); //+1 because of reserve for depot
			bool containsDepot = false;
			//check if depot is in candidate list
			for (const Arc* a : v.candidates) {
				for (const Vertex* actV : a->v) {
					if (actV->c->type == EnityType::DEPOT) {
						containsDepot = true;
						goto CONTAINS_DEPOT;
					}
				}
			}
			CONTAINS_DEPOT: if (!containsDepot) {
				//depot is not in candidates
				v.candidates.pop_back();
				//add depot
				v.candidates.push_back(toDepot);
			}
		}

	}
	#endif


}

void ACO::solve(const unsigned iterations){
	for(Arc& a : arcs) a.pheromone=100;
	bestSoFar.first=std::numeric_limits<double>::infinity();
	bestSoFar.second.clear();

	//iter
	for(unsigned i=0; i<iterations; i++ ){
		//init iter

		std::vector<std::vector<const Vertex*>> iterSolutions;

		std::vector<std::pair<double,unsigned>> sortedSolutions; //cost, solution index
		//create new solution for each ant
		for(Ant& a: ants){
			iterSolutions.push_back(a.genSolution());//every ant creates its solution
			/*
			unsigned num=0;
			for(auto v: iterSolutions.back()){
				if(v->c->type!=EnityType::DEPOT){
					num++;
				}
			}
			if(num!=vrp.getCustomers().size())
				std::cout << "\nERROR HERE: " <<num << std::endl;*/

			#ifndef NO_TWO_OPT
			//lets try to improve solution with 2-opt heuristic
			twoOpt(iterSolutions.back());
			#endif
			sortedSolutions.push_back(std::make_pair(solutionCost(iterSolutions.back()),sortedSolutions.size()));
		}
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

		if(bestSoFar.first>sortedSolutions[0].first){
			//we searched new best
			bestSoFar=std::make_pair(sortedSolutions[0].first, iterSolutions[sortedSolutions[0].second]);
		}

		//update pheromones

		//evaporation
		for(Arc& a: arcs) a.pheromone=ro*a.pheromone;

		//increase pheromones of visited arcs
		for (unsigned mi = 0; mi < sortedSolutions.size() - 1; mi++) {//we don't want the last one
			double pDelta = (sortedSolutions.size() - (mi + 1))
					/ sortedSolutions[mi].first;
			for (unsigned vi = 0;
					vi < iterSolutions[sortedSolutions[mi].second].size() - 1;
					vi++) {
				//select arc for update
				Arc* a =selectArc(*(iterSolutions[sortedSolutions[mi].second][vi]),
						*(iterSolutions[sortedSolutions[mi].second][vi+1])
						);

				a->pheromone += pDelta;

			}

		}

		//increase pheromones for arcs that belongs to best solution so far
		double pDeltaBest=sortedSolutions.size()/bestSoFar.first;

		for (unsigned vi = 0;vi < bestSoFar.second.size() - 1;vi++) {
			//select arc for update
			Arc* a =selectArc(*(bestSoFar.second[vi]),*(bestSoFar.second[vi+1]));
			a->pheromone += pDeltaBest;

		}
		std::cout << i << ". ITER best so far: " << bestSoFar.first-vrp.getCustomers().size()*vrp.getDropTime() <<std::endl;
		/*
		double length=0;
		double carry=0;
		const Vertex * before=nullptr;



		for(auto v: bestSoFar.second){
			if(before!=nullptr) length+=before->distToVertex(*v);
			if(v->c->type==EnityType::DEPOT){
				if(before!=nullptr) std::cout << "-> (" << before->distToVertex(*v) << ") -> "<<v->c->id;
				std::cout <<std::endl;
				std::cout << "\t Route time: " << length << std::endl;
				std::cout << "\t Carry: " << carry << std::endl;
				length=0;
				carry=0;
			}else{

				std::cout << "-> (" << before->distToVertex(*v) << ") -> ";
				std::cout << v->c->id;
				carry+=v->c->quantity;
				length+=vrp.getDropTime();
			}
			before=v;
		}
		std::cout << "len (without drop time): " << bestSoFar.first-vrp.getCustomers().size()*vrp.getDropTime() << std::endl;

		std::cout << "NUM OF CUSTOMERS " << vrp.getCustomers().size() << std::endl;
		std::cout << "CAPACITY " << vrp.getVehicleCapacity() << std::endl;
		std::cout << "MAX TIME " << vrp.getMaxRouteTime() << std::endl;
		std::cout << "DROP TIME " << vrp.getDropTime() << std::endl;

		*/

	}

	bestSoFar.first=bestSoFar.first-vrp.getCustomers().size()*vrp.getDropTime();
}

Ant::Ant(const Vertex* iV, ACO* aco):initVertex(iV), parentACO(aco), dist(0,1) {}

void Ant::returnToDepot(){
	while (route.size() > 0 && route.back()->distToDepot() + time > parentACO->getVrp().getMaxRouteTime()) {
		//we must return because distance to depot exceeds route length limit
		if (route.size() > 1) {
			time -= parentACO->getVrp().getDropTime();
			time -= route.back()->distToVertex(*(route[route.size() - 2]));
			filledCapacity -= route.back()->c->quantity;
		}
		tabu.erase(route.back());
		route.pop_back();
	}
	if (route.size() > 0) {
		//add depot
		route.push_back(&(parentACO->getVertices()[0]));

		//we are in depot so we can reset these
		filledCapacity = 0;
		time = 0;
	} else {
		throw std::runtime_error(
				"Maximum route time is too small. Vehicles can not visit some customers.");
	}
}

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

	START_AGAIN:
	while(tabu.size()<parentACO->getVrp().getCustomers().size()){//if we need to visit some customer
		//find next visit
		const Vertex* nextVertex = nextVisit();

		if (nextVertex == nullptr) {
			//can not find feasible vertex
			//return to the depot
			//it finishes one vehicle route and starts new for another vehicle
			returnToDepot();
		} else {
			//next vertex was find
			//update capacity and time
			filledCapacity += nextVertex->c->quantity;
			//find distance
			time += nextVertex->distToVertex(*route.back());
			//add drop time
			time += parentACO->getVrp().getDropTime();


			route.push_back(nextVertex);
			//add it to tabu
			tabu.insert(nextVertex);

		}
	}
	if(route.back()->c->type!=EnityType::DEPOT){
		//ok we visited all customers but we need to return to depot
		returnToDepot();
		if(tabu.size()<parentACO->getVrp().getCustomers().size()){
			//ok we lost some costumers
			goto START_AGAIN;
		}
	}

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

							if(a->distance+time+parentACO->getVrp().getDropTime()
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
					//middle part add in reverse order (swapping)
					for (unsigned s = y; s >= x; s--) {
						newRoute.push_back(solution[s]);
					}
					//middle part add in reverse order (swapping)
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
		}

	}

}

/*** End of file: ACO.cpp ***/

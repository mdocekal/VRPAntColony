/**
 * Project: VRPAntColony
 * @file main.cpp
 * @date 5. 5. 2018
 * @author xdocek09
 * @brief This program tries to solve vehicle routing problem with ant colony method.
 */
#include <iostream>
#include <fstream>
#include <exception>
#include "VRP.h"
#include "ACO.h"

/**
 * Entry point of the program.
 *
 * @param[in] argc
 * 	Number of arguments.
 * @param[in] argv
 * 	Arguments.
 * @return Exits codes.
 */
int main(int argc, char* argv[]){
	try{
		if(argc!=2){
			std::cerr << "This program is expecting one argument with file containing problem to solve." << std::endl;
			return 1;
		}

		std::ifstream file(argv[1]);
		if(!file){
			std::cerr << "Can not open "<< argv[1] << " for reading."<< std::endl;
			return 2;
		}

		//read problem
		VRP vrp(file);
		file.close();
		if(vrp.getCustomers().size()==0){
			std::cerr << "No customers to visit." << std::endl;
			return 3;
		}

		//init solver
		ACO aco(vrp);
		aco.solve(2*vrp.getCustomers().size()); //according to paper 2n iteration

		auto s=aco.getBestSoFar();
		for(auto v: s.second){
			std::cout << v->c->id << "\t";
		}
		std::cout << "\n" << s.first << std::endl;

	}catch (const std::exception& e) {
		std::cerr << "Error:\n\t"<< e.what() << std::endl;
		return 3;
	}catch(...){
		std::cerr << "Unknown error."<< std::endl;
		return 4;
	}
}




/*** End of file: main.cpp ***/

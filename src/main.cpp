#include<stdio.h>

#include"GreedyConstruction.h"

int main(int argc, char** argv) {
	GreedyConstruction *con = new GreedyConstruction();
	
	
	con->run();
	delete con;

	return 0;
}
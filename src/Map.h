#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

struct RGB_Map {
	Eigen::MatrixXd visR, visG, visB;
};


struct GridPoint {
	int x;
	int y;
};

struct Coord {
	double lat;
	double lon;
};

class Map {
public:
	Map(){};
	// default constructor for the class. Initializes member variables only.
	int numRows;
	Eigen::MatrixXd LOSpercent;
	Eigen::MatrixXd slopeMap;
	RGB_Map img;
	Map(Eigen::MatrixXd LOSpercent) { 
		this->LOSpercent = LOSpercent;  
		numRows = LOSpercent.rows();
	}

	double getLOSpercent(int i, int j) { return LOSpercent(i, j); }

	

	GridPoint landSite;
	std::vector<GridPoint> vantagePoints;
	double distToLander;
	double percentVantagePoints;
	double lengthVantagePath;

};
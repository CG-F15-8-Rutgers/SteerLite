//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#define HEURISTIC_WEIGHT 1
#define DIAGONAL_COST 1

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}
	
	double AStarPlanner::heuristic(int startIndex, int endIndex) {
		//If method is true, use Euclidean, else use Manhattan
		bool method = true;
		unsigned int startx,startz,endx,endz;
		gSpatialDatabase->getGridCoordinatesFromIndex(startIndex,startx,startz);
		gSpatialDatabase->getGridCoordinatesFromIndex(endIndex,endx,endz);
		if (method) {
			return ((double) sqrt((startx-endx)*(startx-endx) + (startz-endz)*(startz-endz)));
		}
		else {
			return ( (abs((double) startx-endx)+abs((double) startz-endz)));
		}
	}
	
	void AStarPlanner::expand(int currentNode, int goalIndex, std::set<int>& openset, std::set<int> closedset, std::map<int,double>& gscore, std::map<int,double>& fscore, std::map<int,int>& camefrom) {
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(currentNode, x, z);
		for (int i = MAX(x-1,0); i<MIN(x+2,gSpatialDatabase->getNumCellsX()); i += GRID_STEP) {
			for (int j = MAX(z-1,0); j<MIN(z+2,gSpatialDatabase->getNumCellsZ()); j += GRID_STEP) {
				int neighbor = gSpatialDatabase->getCellIndexFromGridCoords(i,j);
				if (canBeTraversed(neighbor) && closedset.count(neighbor)==0) {
					double tempg;
					if ((i==x) || (j==z)) {
						tempg = gscore[currentNode]+(DIAGONAL_COST*gSpatialDatabase->getTraversalCost(neighbor));
					}
					else {
						tempg = gscore[currentNode]+gSpatialDatabase->getTraversalCost(neighbor);
					}
					if (tempg < gscore[neighbor]) {
						gscore[neighbor] = tempg;
						fscore[neighbor] = gscore[neighbor]+HEURISTIC_WEIGHT*heuristic(neighbor,goalIndex);
						if(openset.count(neighbor)==1) {
							openset.erase(openset.find(neighbor));
						}
						openset.insert(neighbor);
						camefrom[neighbor] = currentNode;
					}
				}
			}
		}
	}
	
	bool AStarPlanner::retrace_path(std::vector<Util::Point>& agent_path, int currentNode, int startIndex, std::map<int,int>& camefrom) {
		int temp = currentNode;
		std::vector<Util::Point> reverse;
		reverse.push_back(getPointFromGridIndex(temp));
		while (temp != startIndex) {
			temp = camefrom[temp];
			reverse.push_back(getPointFromGridIndex(temp));
		}
		for (int i = reverse.size()-1; i>=0; --i) {
			agent_path.push_back(reverse.at(i));
		}
		std::cout<<"\nPath length: "<<reverse.size()<<'\n';
		return true;
	}
	
	int AStarPlanner::getCurrentNode(std::set<int> openset, std::map<int,double> fscore, std::map<int,double> gscore) {
		std::set<int>::iterator it;
		double temp = INFINITY;
		//If bigger is true, larger g scores have precedence, else smaller scores have precedence
		bool bigger = true;
		for (std::set<int>::iterator i = openset.begin(); i != openset.end(); ++i) {
			if (fscore[(*i)] < temp) {
				temp = fscore[(*i)];
				it = i;
			}
			else if (fscore[(*i)] == temp) {
				if (bigger) {
					if (gscore[(*it)] < gscore[(*i)]) {
						it = i;
					}
				}
				else {
					if (gscore[(*it)] > gscore[(*i)]) {
						it = i;
					}
				}
			}
		}
		return (*it);
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		
		//Create open and closed sets
		std::set<int> openset;
		std::set<int> closedset;
		
		//create map of f and g scores
		std::map<int, double> gscore;
		std::map<int, double> fscore;
		for(int i = 0; i < gSpatialDatabase->getNumCellsX(); ++i){
			for(int j = 0; j < gSpatialDatabase->getNumCellsZ(); ++j){
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i,j);
				gscore[index] = INFINITY;
				fscore[index] = INFINITY;
			}
		}
		
		//create map of path
		std::map<int, int> camefrom;
		
		//initialize starting node
		int startIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		int goalIndex = gSpatialDatabase->getCellIndexFromLocation(goal);
		gscore[startIndex] = 0.0;
		fscore[startIndex] = gscore[startIndex] + HEURISTIC_WEIGHT*heuristic(startIndex,goalIndex);
		openset.insert(startIndex);
		
		while (!openset.empty()) {
			//take node with lowest f in open set, move from openset to closedset
			int currentNode = getCurrentNode(openset, fscore, gscore);
			closedset.insert(currentNode);
			openset.erase(openset.find(currentNode));
			
			if (currentNode == goalIndex) {
				return retrace_path(agent_path, currentNode, startIndex, camefrom);
			}
			
			//expand node, adjust f,g scores of neighbors and add to open set (unless in closed set).
			expand(currentNode,goalIndex,openset,closedset,gscore,fscore,camefrom);
		}
		//TODO
		std::cout<<"\nIn A*";

		return false;
	}
	
}
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


	std::vector<Util::Point> AStarPlanner::reconstruct(AStarPlannerNode _Node, Util::Point start){
		std::vector<Util::Point> reversePath;
		std::vector<Util::Point> path;
		//std::cout<<"reached reconstruct\n";
		while (getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(_Node.point)) != getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(start))){
			//std::cout<<_Node.point.x<<','<<_Node.point.y<<','<<_Node.point.z<<'\n';
			//std::cout<<reversePath.size()<<'\n';
			reversePath.push_back(_Node.point);
			_Node = (*_Node.parent);
		}
		reversePath.push_back(start);
		std::cout<<"out of while\n";
		for(int i = reversePath.size() - 1; i >= 0; --i){
			path.push_back(reversePath.at(i));
		}
		std::cout<<"out\n";
		return path;
	}
	
	double AStarPlanner::heuristic(Util::Point a, Util::Point b){
		double f;
		bool method = true; //if true, use Euclidean, if false use Manhattan
		if(method){
			f = sqrt((a.x - b.x)*(a.x - b.x) + (a.z - b.z)*(a.z - b.z));
		}
		else{
			f = abs(a.x - b.x) + abs(a.z -b.z);
		}
		return f;
	}
	
	

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		//std::cout<<start.x<<','<<start.z<<'\n';
		//std::cout<<goal.x<<','<<goal.z<<'\n';
		gSpatialDatabase = _gSpatialDatabase;
		std::vector<AStarPlannerNode> checked;
		std::vector<AStarPlannerNode> frontier;
		int x_max_range = gSpatialDatabase->getNumCellsX();
		int z_max_range = gSpatialDatabase->getNumCellsZ();
		std::map<int, double> gscore;
		std::map<int, double> fscore;
		for(int i = 0; i < x_max_range; ++i){
			for(int j = 0; j < z_max_range; ++j){
				int WhatAmIDoingWithMyLife = gSpatialDatabase->getCellIndexFromGridCoords(i,j);
				gscore[WhatAmIDoingWithMyLife] = INFINITY;
				fscore[WhatAmIDoingWithMyLife] = INFINITY;
			}
		}
		int startID = gSpatialDatabase->getCellIndexFromLocation(start);
		//Initialize start point to f=heuristic, g=0
		gscore[startID] = 0;
		fscore[startID] = gscore[startID] + heuristic(start,goal);
		AStarPlannerNode startNode(start, gscore[startID], fscore[startID], nullptr);
		frontier.push_back(startNode);
		AStarPlannerNode currentNode = startNode;
		
		while(frontier.empty() == false){
			std::vector<AStarPlannerNode>::iterator it = frontier.begin();
			double temp = frontier.at(0).f;
			//Loop to find frontier node with smallest f
			for(std::vector<AStarPlannerNode>::iterator i = frontier.begin(); i != frontier.end(); ++i){
				if((*i).f <= temp){
					temp = (*i).f;
					it = i;
				}
			}
			//Eliminate from frontier set, add to checked set
			currentNode = (*it);
			checked.push_back(currentNode);
			frontier.erase(it);
			currentNode=checked.at(checked.size()-1);
			//std::cout<<currentNode.point.x<<','<<currentNode.point.z<<'\n';
			if(getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(currentNode.point)) == getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(goal))){
				//std::cout<<"reached goal\n";
				checked.at(checked.size()-1).point = goal;
				agent_path = reconstruct(checked.at(checked.size()-1), start);
				return true;
			}
			int currentID = gSpatialDatabase->getCellIndexFromLocation(currentNode.point);
			unsigned int currentX;
			unsigned int currentZ;
			gSpatialDatabase->getGridCoordinatesFromIndex(currentID,currentX,currentZ);
			
			int x_range_min = MAX(currentX-GRID_STEP, 0);
			int x_range_max = MIN(currentX+GRID_STEP, gSpatialDatabase->getNumCellsX()-1);
			int z_range_min = MAX(currentZ-GRID_STEP, 0);
			int z_range_max = MIN(currentZ+GRID_STEP, gSpatialDatabase->getNumCellsZ()-1);
			
			int tempID;
			Util::Point tempPoint;
			//Loop through each neighbor, add to frontier with pointer to currentNode
			for(int i = x_range_min; i <= x_range_max; ++i) {
				for(int j = z_range_min; j <= z_range_max; ++j) {
					//Continue if the node we are expanding
					if (!(i == currentX && j == currentZ)) {
						tempID = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
						tempPoint = getPointFromGridIndex(tempID);
						bool contains = false;
						//Check to see if the node is already in checked
						for (int k = 0; k < checked.size(); ++k) {
							if (checked.at(k).point == tempPoint) {
								contains = true;
							}
						}
						//If not, and if it can be traversed, calculate g,f scores and add to frontier
						if(canBeTraversed(tempID) && !contains){
							double tempGScore = gscore[currentID] +  gSpatialDatabase->getTraversalCost(tempID);
							if(tempGScore <= gscore[tempID]){
								gscore[tempID] = tempGScore;
								fscore[tempID] = gscore[tempID] + heuristic(tempPoint,goal);
								AStarPlannerNode neighborNode(tempPoint, gscore[tempID], fscore[tempID], &checked.at(checked.size()-1));
								//Erase duplicates
								for (std::vector<AStarPlannerNode>::iterator k = frontier.begin(); k != frontier.end(); ++k) {
									if ((*k).point == tempPoint) {
										frontier.erase(k);
									}
								}
								frontier.push_back(neighborNode);
							}
						}
					}
				}
			}
			
			
		}
		return false;
	}
}
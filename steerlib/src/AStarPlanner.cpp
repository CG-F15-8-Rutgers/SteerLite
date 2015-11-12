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
		//AStarPlannerNode *p = _Node.parent;
		while((*_Node.parent).point != getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(start))){
			//std::cout<<_Node.point.x<<','<<_Node.point.y<<','<<_Node.point.z<<'\n';
			reversePath.push_back(_Node.point);
			//p = _Node.parent;
			_Node = (*_Node.parent);
		}
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
		std::cout<<start.x<<','<<start.z<<'\n';
		std::cout<<goal.x<<','<<goal.z<<'\n';
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
		gscore[startID] = 0;
		fscore[startID] = gscore[startID] + heuristic(start,goal);
		AStarPlannerNode startNode(getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(start)), gscore[startID], fscore[startID], NULL);
		frontier.push_back(startNode);
		AStarPlannerNode currentNode = startNode;
		
		while(frontier.empty() == false){
			std::vector<AStarPlannerNode>::iterator it = frontier.begin();
			double temp = frontier.at(0).f;
			for(std::vector<AStarPlannerNode>::iterator i = frontier.begin(); i != frontier.end(); ++i){
				if((*i).f <= temp){
					temp = (*i).f;
					it = i;
				}
			}
			currentNode = (*it);
			checked.push_back(currentNode);
			if(checked.back() == startNode){
			//std::cout<<checked.back().point.x<<','<<checked.back().point.z<<'\n';
			}
			frontier.erase(it);
			currentNode=checked.back();
			if(currentNode.point == getPointFromGridIndex(gSpatialDatabase->getCellIndexFromLocation(goal))){
				agent_path = reconstruct(checked.back(), start);
				return true;
			}
			int currentID = gSpatialDatabase->getCellIndexFromLocation(currentNode.point);
			unsigned int currentX;
			unsigned int currentZ;
			gSpatialDatabase->getGridCoordinatesFromIndex(currentID,currentX,currentZ);
			int tempID;
			Util::Point tempPoint;
			for(int i = currentX-1; i <= currentX+1; ++i) {
				for(int j = currentZ-1; j <= currentZ+1; ++j) {
					if (!(i == currentX && j == currentZ)) {
						tempID = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
						tempPoint = getPointFromGridIndex(tempID);
						bool contains = false;
						for (int k = 0; k < checked.size(); ++k) {
							if (checked.at(k).point == tempPoint) {
								contains = true;
							}
						}
						if(canBeTraversed(tempID) && !contains){
							double tempGScore = gscore[currentID] + gSpatialDatabase->getTraversalCost(tempID);
							if(tempGScore < gscore[tempID]){
								gscore[tempID] = tempGScore;
								fscore[tempID] = gscore[tempID] + heuristic(tempPoint,goal);
								AStarPlannerNode neighborNode(tempPoint, gscore[tempID], fscore[tempID], &checked.back());
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
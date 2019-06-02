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

	int AStarPlanner::getIndexFromPoint(Util::Point p)
	{
		return gSpatialDatabase->getCellIndexFromLocation(p);
	}

	void neighbor_index_list(int current_cell, std::vector<int> &neighbor_list, SteerLib::SpatialDataBaseInterface *_gSpatialDatabase)
	{
		int dx[] = { -1, 0, 1, -1, 1, -1, 0, 1 }, dz[] = { -1, -1, -1, 0, 0, 1, 1, 1 }, i;
		unsigned int x_cur, z_cur;
		neighbor_list.clear();
		_gSpatialDatabase->getGridCoordinatesFromIndex(current_cell, x_cur, z_cur);
		for (i = 0; i < 8; i++)
		{
			int x_next = x_cur + dx[i];
			int z_next = z_cur + dz[i];
			if (x_next >= 0 && x_next < _gSpatialDatabase->getNumCellsX() && z_next >= 0 && z_next < _gSpatialDatabase->getNumCellsZ())
			{
				neighbor_list.push_back(_gSpatialDatabase->getCellIndexFromGridCoords(x_next, z_next));
			}
		}
	}

	// get the cost from current_cell to next_cell
	double cost_between(int current_cell, int next_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		unsigned int x_cur, z_cur, x_next, z_next;
		_gSpatialDatabase->getGridCoordinatesFromIndex(current_cell, x_cur, z_cur);
		_gSpatialDatabase->getGridCoordinatesFromIndex(next_cell, x_next, z_next);
		if ((MAX(x_cur, x_next)- MIN(x_cur, x_next)) + (MAX(z_cur, z_next)- MIN(z_cur, z_next)) == 1)
			return 1;
		else
			return sqrt(2);
	}

	// check if an agent can go to a cell
	bool can_go_to(int current_cell, int next_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner)
	{
		return (!_gSpatialDatabase->hasAnyItems(next_cell) && (planner->canBeTraversed(next_cell)));
	}

	
	double euclidean_distance(Util::Point &current, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		double dx = fabs(current.x - goal.x), dz = fabs(current.z - goal.z);
		return sqrt(dx * dx + dz * dz);
	}

	double manhattan_distance(Util::Point &current, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		double dx = fabs(current.x - goal.x), dz = fabs(current.z - goal.z);
		return dx + dz;
	}

	double calculate_hvalue(int heuristic_index, int current_cell, int goal_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		Util::Point current_point, goal_point;
		_gSpatialDatabase->getLocationFromIndex(current_cell, current_point);
		_gSpatialDatabase->getLocationFromIndex(goal_cell, goal_point);
		if (heuristic_index == 1)
			return euclidean_distance(current_point, goal_point, _gSpatialDatabase);
		else if (heuristic_index == 2)
			return manhattan_distance(current_point, goal_point, _gSpatialDatabase);
	}

	double calculate_fvalue(double gvalue, double w, int heuristic_index, int current_cell, int goal_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		double fvalue = gvalue + w * calculate_hvalue(heuristic_index, current_cell, goal_cell, _gSpatialDatabase);
		return fvalue;
	}

	void final_path(std::vector<Util::Point> &agent_path, std::unordered_map<int, int> come_from_list, int start_cell, int goal_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		int current_cell = goal_cell, i;
		std::vector<Util::Point> reversed_list;
		reversed_list.clear();
		while (1)
		{
			Util::Point current_location;
			_gSpatialDatabase->getLocationFromIndex(current_cell, current_location);
			reversed_list.push_back(current_location);
			if (current_cell == start_cell)
				break;
			current_cell = come_from_list[current_cell];
		}
		agent_path.clear();
		for (i = reversed_list.size() - 1; i >= 0; i--)
			agent_path.push_back(reversed_list[i]);
	}

	bool sequential_astar(std::vector<Util::Point>& agent_path, Util::Point &start, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, std::vector<double> &weight, std::vector<int> &heuristic, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_taken)
	{
		
		int heuristic_n = heuristic.size(), i, j;

		std::unordered_map<int, int> *come_from = new std::unordered_map<int, int>[heuristic_n];
		Astar_minheap *open_list = new Astar_minheap[heuristic_n];
		std::unordered_map<int, double> *f_val = new std::unordered_map<int, double>[heuristic_n], *g_val = new std::unordered_map<int, double>[heuristic_n], *h_val = new std::unordered_map<int, double>[heuristic_n];
		std::set<int> *closed_list = new std::set<int>[heuristic_n];
		int start_cell, goal_cell;
		bool path_found = false;

		auto start_time = std::chrono::steady_clock::now();
		start_cell = _gSpatialDatabase->getCellIndexFromLocation(start);
		goal_cell = _gSpatialDatabase->getCellIndexFromLocation(goal);

		node_expanded = 0;
		node_generated = 0;
		for (i = 0; i < heuristic_n; i++)
		{
			double f_start, g_start, h_start;
			(come_from[i]).clear();
			(open_list[i]).init();
			(f_val[i]).clear();
			(g_val[i]).clear();
			(h_val[i]).clear();
			(closed_list[i]).clear();
			node_generated++;
			g_start = 0;
			h_start = calculate_hvalue(heuristic[i], start_cell, goal_cell, _gSpatialDatabase);
			f_start = calculate_fvalue(g_start, weight[i], heuristic[i], start_cell, goal_cell, _gSpatialDatabase);
			g_val[i][start_cell] = g_start, h_val[i][start_cell] = h_start, f_val[i][start_cell] = f_start;
			(open_list[i]).insert(f_start, g_start, h_start, start_cell);
		}

		while (!((open_list[0]).empty()))
		{
			for (i = 0; i < heuristic_n; i++)
			{
				std::vector<int> neighbor_list;
				double f0_cur, g0_cur, h0_cur;
				double fi_cur, gi_cur, hi_cur;
				double f_chosen, g_chosen, h_chosen;
				int current_cell_0, current_cell_i, current_chosen, chosen_heurisic;
				(open_list[0]).root(f0_cur, g0_cur, h0_cur, current_cell_0);
				(open_list[i]).root(fi_cur, gi_cur, hi_cur, current_cell_i);
				if (heuristic_n == 1 || fi_cur <= f0_cur)
				{
					chosen_heurisic = i;
					f_chosen = fi_cur, g_chosen = gi_cur, h_chosen = hi_cur;
					current_chosen = current_cell_i;
				}
				else
				{
					chosen_heurisic = 0;
					f_chosen = f0_cur, g_chosen = g0_cur, h_chosen = h0_cur;
					current_chosen = current_cell_0;
				}
				(open_list[chosen_heurisic]).pop();
				(closed_list[chosen_heurisic]).insert(current_chosen);
				node_expanded++;
				if (current_chosen == goal_cell)
				{
					final_path(agent_path, come_from[chosen_heurisic], start_cell, goal_cell, _gSpatialDatabase);
					path_found = true;
					path_cost = g_chosen;
					path_length = agent_path.size();
					break;
				}
				neighbor_index_list(current_chosen, neighbor_list, _gSpatialDatabase);
				for (j = 0; j < neighbor_list.size(); j++)
					if (can_go_to(current_chosen, neighbor_list[j], _gSpatialDatabase, planner) && (closed_list[chosen_heurisic]).find(neighbor_list[j]) == (closed_list[chosen_heurisic]).end())
					{
						int next_cell = neighbor_list[j];
						double g_new = g_chosen + cost_between(current_chosen, next_cell, _gSpatialDatabase);
						if ((g_val[chosen_heurisic]).find(next_cell) == (g_val[chosen_heurisic]).end() || g_new < g_val[chosen_heurisic][next_cell])
						{
							double g_next = g_new;
							double h_next = calculate_hvalue(heuristic[chosen_heurisic], next_cell, goal_cell, _gSpatialDatabase);
							double f_next = calculate_fvalue(g_next, weight[chosen_heurisic], heuristic[chosen_heurisic], next_cell, goal_cell, _gSpatialDatabase);
							if ((g_val[chosen_heurisic]).find(next_cell) == (g_val[chosen_heurisic]).end())
								node_generated++;
							come_from[chosen_heurisic][next_cell] = current_chosen;
							g_val[chosen_heurisic][next_cell] = g_next;
							h_val[chosen_heurisic][next_cell] = h_next;
							f_val[chosen_heurisic][next_cell] = f_next;
							if (!((open_list[chosen_heurisic]).index_exists(next_cell)))
								(open_list[chosen_heurisic]).insert(f_next, g_next, h_next, next_cell);
							else
								(open_list[chosen_heurisic]).update(f_next, g_next, h_next, next_cell);
						}
					}
			}
			if (path_found)
				break;
		}

		auto end_time = std::chrono::steady_clock::now();
		auto diff_time = end_time - start_time;
		time_taken = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count())/ 1000000000;
		std::cout << "Time Taken: " << time_taken;
		std::cout << "Nodes expanded: " << node_expanded;
		std::cout << "Nodes generated: " << node_generated;
		std::cout << "Path length: " << path_length;
		std::cout << "Path cost: " << path_cost;
		
		delete[] come_from;
		come_from = NULL;
		delete[] open_list;
		open_list = NULL;
		delete[] f_val;
		f_val = NULL;
		delete[] g_val;
		g_val = NULL;
		delete[] h_val;
		h_val = NULL;
		delete[] closed_list;
		closed_list = NULL;


		return path_found;
	}

	bool weighted_astar(std::vector<Util::Point>& agent_path, Util::Point &start, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, double weight, int heuristic_index, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs)
	{
		// init
		auto start_time = std::chrono::steady_clock::now();
		std::unordered_map<int, int> come_from;
		Astar_minheap open_list;
		std::unordered_map<int, double> f_val, g_val, h_val;
		std::set<int> closed_list;
		int start_point, goal_point;
		bool path_found = false;

		start_point = _gSpatialDatabase->getCellIndexFromLocation(start);
		goal_point = _gSpatialDatabase->getCellIndexFromLocation(goal);

		come_from.clear();
		open_list.init();
		f_val.clear();
		g_val.clear();
		h_val.clear();
		closed_list.clear();
		node_expanded = 0;
		node_generated = 1;

		g_val[start_point] = 0;
		h_val[start_point] = calculate_hvalue(heuristic_index, start_point, goal_point, _gSpatialDatabase);
		f_val[start_point] = calculate_fvalue(g_val[start_point], weight, heuristic_index, start_point, goal_point, _gSpatialDatabase);

		open_list.insert(f_val[start_point], g_val[start_point], h_val[start_point], start_point);

		while (!open_list.empty())
		{
			std::vector<int> neighbor_list;
			int current_cell, i;
			double f_cur, g_cur, h_cur;
			open_list.root(f_cur, g_cur, h_cur, current_cell);
			open_list.pop();
			closed_list.insert(current_cell);
			node_expanded++;
			if (current_cell == goal_point)
			{
				final_path(agent_path, come_from, start_point, goal_point, _gSpatialDatabase);
				path_found = true;
				path_cost = g_cur;
				path_length = agent_path.size();
				break;
			}
			neighbor_index_list(current_cell, neighbor_list, _gSpatialDatabase);
			for (i = 0; i < neighbor_list.size(); i++)
				if (can_go_to(current_cell, neighbor_list[i], _gSpatialDatabase, planner) && closed_list.find(neighbor_list[i]) == closed_list.end())
				{
					int next_cell = neighbor_list[i];
					double g_new = g_cur + cost_between(current_cell, next_cell, _gSpatialDatabase);
					if (g_val.find(next_cell) == g_val.end() || g_new < g_val[next_cell])
					{
						double f_next, g_next, h_next;
						g_next = g_new;
						h_next = calculate_hvalue(heuristic_index, next_cell, goal_point, _gSpatialDatabase);
						f_next = calculate_fvalue(g_next, weight, heuristic_index, next_cell, goal_point, _gSpatialDatabase);
						if (g_val.find(next_cell) == g_val.end())
							node_generated++;
						come_from[next_cell] = current_cell;
						g_val[next_cell] = g_next;
						f_val[next_cell] = f_next;
						h_val[next_cell] = h_next;
						if (!open_list.index_exists(next_cell))
							open_list.insert(f_next, g_next, h_next, next_cell);
						else
							open_list.update(f_next, g_next, h_next, next_cell);
					}
				}
		}

		auto end_time = std::chrono::steady_clock::now();
		auto diff_time = end_time - start_time;
		time_secs = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count());
		time_secs /= 1000000000;
		std::cout << "Time Taken: " << time_secs;
		std::cout << "Nodes expanded: " << node_expanded;
		std::cout << "Nodes generated: " << node_generated;
		std::cout << "Path length: " << path_length;
		std::cout << "Path cost: " << path_cost;
		return path_found;
	}

	bool improve(Astar_minheap &open_list, std::set<int> &incons_cell, std::unordered_map<int, double> &incons_f, std::unordered_map<int, double> &incons_g, std::unordered_map<int, double> &incons_h, std::unordered_map<int, int> &incons_come_from, std::set<int> &closed_list, std::unordered_map<int, int> &come_from_list, std::unordered_map<int, double> &f_val, std::unordered_map<int, double> &g_val, std::unordered_map<int, double> &h_val, double weight, int heuristic_index, int start_cell, int goal_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, std::vector<Util::Point> &path_reported, int &node_expanded, int &node_generated)
	{
		int i;
		bool path_found = false;
		while (!(open_list.empty()))
		{
			double f_cur, g_cur, h_cur;
			int current_cell;
			open_list.root(f_cur, g_cur, h_cur, current_cell);
			if (f_val.find(goal_cell) != f_val.end() && f_val[goal_cell] < f_cur)
				break;
			open_list.pop();
			closed_list.insert(current_cell);
			node_expanded++;
			std::vector<int> neighbor_list;
			neighbor_index_list(current_cell, neighbor_list, _gSpatialDatabase);
			for (i = 0; i < neighbor_list.size(); i++)
				if (can_go_to(current_cell, neighbor_list[i], _gSpatialDatabase, planner))
				{
					int next_cell = neighbor_list[i];
					double g_new = g_cur + cost_between(current_cell, next_cell, _gSpatialDatabase);
					if (g_val.find(next_cell) == g_val.end() || g_new < g_val[next_cell])
					{
						double g_next = g_new;
						double h_next = calculate_hvalue(heuristic_index, next_cell, goal_cell, _gSpatialDatabase);
						double f_next = calculate_fvalue(g_next, weight, heuristic_index, next_cell, goal_cell, _gSpatialDatabase);
						if (closed_list.find(next_cell) == closed_list.end())
						{
							come_from_list[next_cell] = current_cell;
							f_val[next_cell] = f_next;
							g_val[next_cell] = g_next;
							h_val[next_cell] = h_next;
							if (!open_list.index_exists(next_cell))
							{
								node_generated++;
								open_list.insert(f_next, g_next, h_next, next_cell);
							}
							else
								open_list.update(f_next, g_next, h_next, next_cell);
						}
						else
						{
							if (incons_cell.find(next_cell) == incons_cell.end() || g_next < incons_g[next_cell])
							{
								incons_cell.insert(next_cell);
								incons_f[next_cell] = f_next;
								incons_g[next_cell] = g_next;
								incons_h[next_cell] = h_next;
								incons_come_from[next_cell] = current_cell;
							}
						}
					}
				}

		}
		if (come_from_list.find(goal_cell) != come_from_list.end())
		{
			path_found = true;
			final_path(path_reported, come_from_list, start_cell, goal_cell, _gSpatialDatabase);
		}
		return path_found;
	}

	void incons_to_open(Astar_minheap &open_list, std::unordered_map<int, double> &f_val, std::unordered_map<int, double> &g_val, std::unordered_map<int, double> &h_val, std::unordered_map<int, int> &come_from_list, std::set<int> &incons_cell, std::unordered_map<int, double> &incons_f, std::unordered_map<int, double> &incons_g, std::unordered_map<int, double> &incons_h, std::unordered_map<int, int> &incons_come_from, SteerLib::SpatialDataBaseInterface *_gSpatialDatabase, double weight, int heuristic_index, int goal_cell)
	{
		std::set<int> cell_in_open;
		cell_in_open.clear();

		while (!(open_list.empty()))
		{
			int cell_root;
			double f_root, g_root, h_root;
			open_list.root(f_root, g_root, h_root, cell_root);
			open_list.pop();
			cell_in_open.insert(cell_root);
			g_val[cell_root] = g_root;
			h_val[cell_root] = calculate_hvalue(heuristic_index, cell_root, goal_cell, _gSpatialDatabase);
			f_val[cell_root] = calculate_fvalue(g_root, weight, heuristic_index, cell_root, goal_cell, _gSpatialDatabase);
		}

		for (auto cell_checked : incons_cell)
		{
			come_from_list[cell_checked] = incons_come_from[cell_checked];
			g_val[cell_checked] = incons_g[cell_checked];
			h_val[cell_checked] = calculate_hvalue(heuristic_index, cell_checked, goal_cell, _gSpatialDatabase);
			f_val[cell_checked] = calculate_fvalue(g_val[cell_checked], weight, heuristic_index, cell_checked, goal_cell, _gSpatialDatabase);
			if (cell_in_open.find(cell_checked) == cell_in_open.end())
				cell_in_open.insert(cell_checked);
		}

		for (auto cell_checked : cell_in_open)
		{
			open_list.insert(f_val[cell_checked], g_val[cell_checked], h_val[cell_checked], cell_checked);
		}

		incons_cell.clear();
		incons_f.clear();
		incons_g.clear();
		incons_h.clear();
		incons_come_from.clear();
	}

	bool araStar(std::vector<Util::Point>& agent_path, Util::Point &start, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, double init_weight, double weight_decay, int heuristic_index, double time_limit, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs)
	{
		// init
		auto start_time = std::chrono::steady_clock::now();
		std::unordered_map<int, int> come_from, incons_come_from;
		Astar_minheap open_list;
		std::unordered_map<int, double> f_val, g_val, h_val;
		std::unordered_map<int, double> incons_f, incons_g, incons_h;
		std::set<int> closed_list, incons_cell;
		int start_cell, goal_cell;
		bool path_found = false;
		double weight = init_weight;

		start_cell = _gSpatialDatabase->getCellIndexFromLocation(start);
		goal_cell = _gSpatialDatabase->getCellIndexFromLocation(goal);

		come_from.clear();
		open_list.init();
		f_val.clear();
		g_val.clear();
		h_val.clear();
		closed_list.clear();
		incons_come_from.clear();
		incons_f.clear();
		incons_g.clear();
		incons_h.clear();
		incons_cell.clear();
		node_expanded = 0;
		node_generated = 1;

		g_val[start_cell] = 0;
		h_val[start_cell] = calculate_hvalue(heuristic_index, start_cell, goal_cell, _gSpatialDatabase);
		f_val[start_cell] = calculate_fvalue(g_val[start_cell], weight, heuristic_index, start_cell, goal_cell, _gSpatialDatabase);

		open_list.insert(f_val[start_cell], g_val[start_cell], h_val[start_cell], start_cell);
		path_found = improve(open_list, incons_cell, incons_f, incons_g, incons_h, incons_come_from, closed_list, come_from, f_val, g_val, h_val, weight, heuristic_index, start_cell, goal_cell, _gSpatialDatabase, planner, agent_path, node_expanded, node_generated);
		do
		{
			auto end_time = std::chrono::steady_clock::now();
			auto diff_time = end_time - start_time;
			time_secs = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count());
			time_secs /= 1000000000;
			if (time_secs >= time_limit || weight < 1)
				break;
			weight -= weight_decay;
			if (weight < 1)
				weight = 1;
			incons_to_open(open_list, f_val, g_val, h_val, come_from, incons_cell, incons_f, incons_g, incons_h, incons_come_from, _gSpatialDatabase, weight, heuristic_index, goal_cell);
			closed_list.clear();
			path_found = improve(open_list, incons_cell, incons_f, incons_g, incons_h, incons_come_from, closed_list, come_from, f_val, g_val, h_val, weight, heuristic_index, start_cell, goal_cell, _gSpatialDatabase, planner, agent_path, node_expanded, node_generated);
		} while (weight > 1);

		if (path_found)
		{
			path_length = agent_path.size();
			path_cost = g_val[goal_cell];
		}

		return path_found;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		
		double path_cost, time_secs;
		int path_length, node_expanded, node_generated, i;
		bool path_found;
		std::vector<Util::Point> plan_output;
		plan_output.clear();

		// weighted A*
		/*
		path_found = weighted_astar(plan_output, start, goal, _gSpatialDatabase, this, 5, 1, path_cost, path_length, node_expanded, node_generated, time_secs);
		if (!append_to_path)
		{
			agent_path.clear();
		}
		for (i = 0; i < plan_output.size(); i++)
			agent_path.push_back(plan_output[i]);
		*/
		// sequential A*
		
		std::vector<int> heuristic_index = {2, 1};
		std::vector<double> weight = {1, 1.1};
		path_found = sequential_astar(plan_output, start, goal, _gSpatialDatabase, this, weight, heuristic_index, path_cost, path_length, node_expanded, node_generated, time_secs);
		if (!append_to_path)
		{
			agent_path.clear();
		}
		for (i = 0; i < plan_output.size(); i++)
			agent_path.push_back(plan_output[i]);
		
		
		// ARA*
		
		/*path_found = araStar(plan_output, start, goal, _gSpatialDatabase, this, 300, 3, 1, 2, path_cost, path_length, node_expanded, node_generated, time_secs);
		printf("debug: plan done within %lf\n", time_secs);
		if (!append_to_path)
		{
			agent_path.clear();
		}
		for (i = 0; i < plan_output.size(); i++)
			agent_path.push_back(plan_output[i]);
		*/
		
		
		return path_found;
	}
}
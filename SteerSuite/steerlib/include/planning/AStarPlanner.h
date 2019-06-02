//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"
#include <unordered_map>
#include <chrono>


namespace SteerLib
{
	class Astar_minheap
	{
	public:

		//initialize the heap
		void init()
		{
			fvalue_heap.push_back(-1);
			gvalue_heap.push_back(-1);
			hvalue_heap.push_back(-1);
			indexnumber_heap.push_back(-1);
		}

		//clear the heap
		void clear()
		{
			fvalue_heap.clear();
			gvalue_heap.clear();
			hvalue_heap.clear();
			indexnumber_heap.clear();
			index_node.clear();
		}

		

		Astar_minheap()
		{
			clear();
			init();

		}

		~Astar_minheap()
		{
			clear();
		}

		//returns the values stored in the root node of the heap
		bool root(double &fvalue, double &gvalue, double &hvalue, int &index)
		{
			if (empty())
				return false;
			fvalue = fvalue_heap[1];
			gvalue = gvalue_heap[1];
			hvalue = hvalue_heap[1];
			index = indexnumber_heap[1];
			return true;
		}

		// return true when first node has smaller cost thatn the second node
		static bool less(double fvalue1, double gvalue1, double fvalue2, double gvalue2)
		{
			if (fvalue1 < fvalue2)
				return true;
			else if (fvalue1 > fvalue2)
				return false;
			else
				return (gvalue1 < gvalue2);
		}

		// maintain the min heap
		int maintain_minheap(int node)
		{
			if (node >= indexnumber_heap.size())
				return -1;
			int temp = node;
			int n = fvalue_heap.size();
			
			while (temp > 1 && less(fvalue_heap[temp], gvalue_heap[temp], fvalue_heap[temp >> 1], gvalue_heap[temp >> 1]))
			{
				swap(temp, (temp >> 1));
				temp = (temp >> 1);
			}
			
			while ((temp << 1) < n)
			{
				int left_child = (temp << 1);
				int right_child = 1 + (temp << 1);
				int best_child;
				if (right_child >= n || less(fvalue_heap[left_child], gvalue_heap[left_child], fvalue_heap[right_child], gvalue_heap[right_child]))
					best_child = left_child;
				else
					best_child = right_child;
				if (less(fvalue_heap[temp], gvalue_heap[temp], fvalue_heap[best_child], gvalue_heap[best_child]))
					break;
				swap(temp, best_child);
				temp = best_child;
			}
			return temp;
		}

		// return true if the heap is empty
		bool empty()
		{
			return (fvalue_heap.size() < 2);
		}

		// return true when cell with index exists in the heap
		bool index_exists(int index)
		{
			if (index_node.find(index) == index_node.end())
				return false;
			return true;
		}

		// inserts a new node in the heap
		bool insert(double fvalue, double gvalue, double hvalue, int index)
		{
			fvalue_heap.push_back(fvalue);
			gvalue_heap.push_back(gvalue);
			hvalue_heap.push_back(hvalue);
			indexnumber_heap.push_back(index);
			index_node[index] = indexnumber_heap.size() - 1;
			maintain_minheap(index_node[index]);
			return true;
		}

		//deletes the last node of the heap
		bool pop()
		{
			if (empty())
				return false;
			int n = fvalue_heap.size() - 1;
			int index = indexnumber_heap[n];
			swap(1, n);
			fvalue_heap.pop_back();
			gvalue_heap.pop_back();
			hvalue_heap.pop_back();
			indexnumber_heap.pop_back();
			index_node.erase(index);
			maintain_minheap(1);
			return true;
		}


		bool update(double fvalue, double gvalue, double hvalue, int index)
		{
			if (!index_exists(index))
				return false;
			int node = index_node[index];
			fvalue_heap[node] = fvalue;
			gvalue_heap[node] = gvalue;
			hvalue_heap[node] = hvalue;
			maintain_minheap(node);
			return true;
		}

		bool remove_node(int index)
		{
			if (!index_exists(index))
				return false;
			int node = index_node[index];
			int n = indexnumber_heap.size() - 1;
			swap(node, n);
			fvalue_heap.pop_back();
			gvalue_heap.pop_back();
			hvalue_heap.pop_back();
			indexnumber_heap.pop_back();
			index_node.erase(index);
			maintain_minheap(node);
			return true;
		}

	private:
		std::vector<double> fvalue_heap, gvalue_heap, hvalue_heap;
		std::vector<int> indexnumber_heap;
		std::unordered_map<int, int> index_node;

		void swap(int node1, int node2)
		{
			double temp = fvalue_heap[node1];
			fvalue_heap[node1] = fvalue_heap[node2];
			fvalue_heap[node2] = temp;
			temp = gvalue_heap[node1];
			gvalue_heap[node1] = gvalue_heap[node2];
			gvalue_heap[node2] = temp;
			temp = hvalue_heap[node1];
			hvalue_heap[node1] = hvalue_heap[node2];
			hvalue_heap[node2] = temp;
			int index1 = indexnumber_heap[node1];
			int index2 = indexnumber_heap[node2];
			index_node[index1] = node2;
			index_node[index2] = node1;
			indexnumber_heap[node1] = index2;
			indexnumber_heap[node2] = index1;
		}

	};

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}
			bool operator<(AStarPlannerNode other) const
		    {
		        return this->f < other.f;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
		        return this->f > other.f;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }

	};

	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			int getIndexFromPoint(Util::Point p);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/
			
			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);
			//int getIndexFromPoint(Util::Point p);
		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	};


}


#endif

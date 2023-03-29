/*
RRT-Connect path planning algorithm for grid based map
Author: Hossein Rezaeifar
Copyright: Copyright (c) 2023, Hossein Rezaeifar
License: MIT
Date: March 2023
*/

#ifndef RRT_CONNECT_OOP_RRTCONNECT_H
#define RRT_CONNECT_OOP_RRTCONNECT_H

#include <bits/stdc++.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <tuple>
#include <random>
#include <limits>
#include <algorithm>

using namespace std;

// Define the size of the grid cell
const double CELL_SIZE = 1.0;

// Define the radius of the robot
const double ROBOT_RADIUS = 0.5;

// Define the maximum number of iterations for RRT
const int MAX_ITER = 10000;

// Define the maximum distance in RRT branch
const double MAX_DIS = 2;

// Define the data structure for a 2D point
struct Point {
    double x;
    double y;
};

// Define the data structure for a grid cell
struct Cell {
    bool obstacle;
    Point center;
};

// Define the data structure for a node in the RRT tree
struct Node {
    Point point;
    int parent;
};

class RRTConnect
{

public:
    int ROW, COL;
    vector<vector<Cell>> grid;
    Point start, goal;

    RRTConnect(vector<vector<Cell>> mgrid, Point mstart, Point mgoal);

    double distance(const Point& p1, const Point& p2);
    double Angle(const Point& p1, const Point& p2);
    bool withinBounds(const Point& p);
    bool withinObstacle(const Point& p, const vector<vector<Cell>>& grid);
    vector<vector<int>> bresenham(double x0, double y0, double x1, double y1);
    bool clearPath(const Point& p1, const Point& p2, const vector<vector<Cell>>& grid);
    Point generateRandomPoint();
    int nearestNodeIndex(const vector<Node>& tree, const Point& p);
    Node createNewNode(const Point& p, int parentIndex);
    tuple<bool,int> test_goal(const Node& p1, const vector<Node>& node_list, double tolerance);
    vector<Node> rrtConnect(const Point& start, const Point& goal, const vector<vector<Cell>>& grid);
    void initmap();
    void trace(vector<Node>& path);

};

#endif //RRT_CONNECT_OOP_RRTCONNECT_H


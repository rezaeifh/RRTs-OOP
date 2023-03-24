/*
Rapidly-Exploring Random Trees (RRT) path planning algorithm for grid based map
Author: Hossein Rezaeifar
Copyright: Copyright (c) 2023, Hossein Rezaeifar
License: MIT
Date: March 2023
*/

#ifndef RRT_RRT_H
#define RRT_RRT_H

#include <bits/stdc++.h>
#include <iostream>
#include <cmath>
#include <vector>
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

class RRT
{

public:
    int ROW, COL;
    vector<vector<Cell>> grid;
    Point start, goal;

    RRT(vector<vector<Cell>> mgrid, Point mstart, Point mgoal);

    double distance(const Point& p1, const Point& p2);
    bool withinBounds(const Point& p);
    bool withinObstacle(const Point& p, const vector<vector<Cell>>& grid);
    vector<vector<int>> bresenham(double x0, double y0, double x1, double y1);
    bool clearPath(const Point& p1, const Point& p2, const vector<vector<Cell>>& grid);
    Point generateRandomPoint();
    int nearestNodeIndex(const vector<Node>& tree, const Point& p);
    Node createNewNode(const Point& p, int parentIndex);
    vector<Node> rrtpath(const Point& start, const Point& goal, const vector<vector<Cell>>& grid);
    void initmap();
    void trace(const vector<Node>& path);

};

#endif //RRT_RRT_H

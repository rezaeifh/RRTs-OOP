/*
Rapidly-Exploring Random Trees (RRT) path planning algorithm for grid based map
Author: Hossein Rezaeifar
Copyright: Copyright (c) 2023, Hossein Rezaeifar
License: MIT
Date: March 2023
*/

#include <bits/stdc++.h>
#include "RRT.h"

using namespace std;

// Class constructor
RRT::RRT(vector<vector<Cell>> mgrid, Point mstart, Point mgoal){

    grid = mgrid;
    start = mstart;
    goal = mgoal;
    ROW = grid.size();
    COL = grid[0].size();

}

// Define a function to calculate the distance between two points
double RRT::distance(const Point& p1, const Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

// Define a function to check if a point is within the bounds of the workspace
bool RRT::withinBounds(const Point& p) {
    return p.x >= 0 && p.x < (ROW-2) * CELL_SIZE && p.y >= 0 && p.y < (COL-2) * CELL_SIZE;
}

// Define a function to check if a point is within an obstacle
bool RRT::withinObstacle(const Point& p, const vector<vector<Cell>>& grid) {
    for (int i = 0 ; i < ROW; i++){
        for (int j = 0; j < COL; j++){
            if (grid[i][j].obstacle && distance(p, grid[i][j].center) <= ROBOT_RADIUS) {
                return true;
            }
        }
    }
    return false;
}

// Implementation of Bresenham's line drawing algorithm: For example, the coordinates of a line from (-1, -4) to (3, 2), are:
// [(-1, -4), (0, -3), (0, -2), (1, -1), (2, 0), (2, 1), (3, 2)]
vector<vector<int>> RRT::bresenham(double x0, double y0, double x1, double y1) {

    int xsign, ysign, xx, xy, yx, yy, D, y, xp0, xp1, yp0, yp1;
    vector<vector<int>> bres;
    for (int i = 0; i < ROW - 2; i++) {
        for (int j = 0; j < COL - 2; j++) {
            if (x0 > i && x0 < i + 1) xp0 = i + 1;
            if (x1 > i && x1 < i + 1) xp1 = i + 1;
            if (y0 > j && y0 < j + 1) yp0 = j + 1;
            if (y1 > j && y1 < j + 1) yp1 = j + 1;
        }
    }

    int dx = abs(xp1 - xp0);
    int dy = abs(yp1 - yp0);

    if (dx > 0) xsign = 1;
    else xsign = -1;

    if (dy > 0) ysign = 1;
    else ysign = -1;

    if (dx > dy) {
        xx = xsign;
        xy = 0;
        yx = 0;
        yy = ysign;
    } else {
        int hol = dy;
        dy = dx;
        dx = hol;

        xx = 0;
        xy = ysign;
        yx = xsign;
        yy = 0;
    }

    D = 2 * dy - dx;
    y = 0;

    for (int x = 0; x <= dx; x++) {
        bres.push_back({xp0 + x * xx + y * yx, yp0 + x * xy + y * yy});

        if (D >= 0) {
            y += 1;
            D -= 2 * dx;
            D += 2 * dy;
        }
    }

    return bres;
}

// Define a function to check if there is a clear path between two points
bool RRT::clearPath(const Point& p1, const Point& p2, const vector<vector<Cell>>& grid) {
    vector<vector<int>> breslis = bresenham(p1.x, p1.y, p2.x, p2.y);

    if (withinBounds(p2)){

        for (vector<int> poi : breslis){
            if (grid[poi[0]][poi[1]].obstacle) return false;
        }
        return true;
    }

    else return false;
}

// Define a function to generate a random point within the bounds of the workspace
Point RRT::generateRandomPoint() {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> dis1(0.0, (ROW-2) * CELL_SIZE);
    uniform_real_distribution<double> dis2(0.0, (COL-2) * CELL_SIZE);
    double x = dis1(gen);
    double y = dis2(gen);
    return {x, y};
}

// Define a function to find the index of the nearest node in the RRT tree
int RRT::nearestNodeIndex(const vector<Node>& tree, const Point& p) {
    int nearestIndex = 0;
    double minDist = numeric_limits<double>::max();
    for (int i = 0; i < tree.size(); i++) {
        double d = distance(tree[i].point, p);
        if (d < minDist) {
            nearestIndex = i;
            minDist = d;
        }
    }
    return nearestIndex;
}

// Define a function to create a new node
Node RRT::createNewNode(const Point& p, int parentIndex) {
    return {p, parentIndex};
}

// Define the RRT algorithm with obstacle avoidance
vector<Node> RRT::rrtpath(const Point& start, const Point& goal, const vector<vector<Cell>>& grid) {

    //check if the start point is inside the map
    if (withinBounds(start) == false) {
        cout << "Start point is outside the map" << endl;
        return {};
    }

    //check if the goal point is inside the map
    if (withinBounds(goal) == false) {
        cout << "Goal point is outside the map" << endl;
        return {};
    }

    //check if the start point is on the obstacle
    if (withinObstacle(start, grid)) {
        cout << "Start point is on the obstacle" << endl;
        return {};
    }

    //check if the start point is on the obstacle
    if (withinObstacle(goal, grid)) {
        cout << "Goal point is on the obstacle" << endl;
        return {};
    }

    vector<Node> tree;
    tree.push_back({start, -1}); // Add the start node to the tree

    for (int i = 0; i < MAX_ITER; i++) {
        Point randPoint = generateRandomPoint(); // Generate a random point
        int nearestIndex = nearestNodeIndex(tree, randPoint); // Find the nearest node in the tree

        Point nearestPoint = tree[nearestIndex].point;
        Point newPoint;
        if (clearPath(nearestPoint, randPoint, grid)) { // Check if there is a clear path to the random point
            if (distance(nearestPoint, randPoint) < CELL_SIZE) { // If the random point is very close to the nearest node, set the new point to be the random point
                newPoint = randPoint;
            } else { // Otherwise, set the new point to be the point on the path that is one grid cell away from the nearest node
                double d = distance(nearestPoint, randPoint);
                double dx = (randPoint.x - nearestPoint.x) / d;
                double dy = (randPoint.y - nearestPoint.y) / d;
                newPoint = {nearestPoint.x + CELL_SIZE * dx, nearestPoint.y + CELL_SIZE * dy};
            }
            if (!withinObstacle(newPoint, grid)) { // Check if the new point is within an obstacle
                int newNodeIndex = tree.size(); // Add the new node to the tree
                tree.push_back(createNewNode(newPoint, nearestIndex));
                if (distance(newPoint, goal) < CELL_SIZE/2) { // If the new node is close enough to the goal, return the path
                    vector<Node> path;
                    int currentIndex = newNodeIndex;
                    while (currentIndex != -1) {
                        path.push_back(tree[currentIndex]);
                        currentIndex = tree[currentIndex].parent;
                    }
                    reverse(path.begin(), path.end());
                    return path;
                }
            }
        }
    }
    return {}; // If no path is found, return an empty vector
}

// Define a function for presenting the scenario
void RRT::initmap(){

    cout << endl;
    cout << "Map of the scenario: (S is the Startpoint, G is the Goalpoint and X is the obstacle)" << endl;

    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            if (i == 0 || j == 0 || i == ROW - 1 || j == COL - 1) {
                grid[i][j].obstacle = false;
            }
        }
    }

    // Print the grid with obstacles
    for (int i = 0; i < COL; i++){
        for (int j =0; j < ROW; j++){
            if (i == 0 || j == 0 || j == ROW-1 || i == COL-1) cout << "X ";
            else if (grid[j][COL-i-1].obstacle) cout << "X ";
            else if ( j == start.x+CELL_SIZE/2 && i == COL-2-start.y+CELL_SIZE/2) cout << "S ";
            else if ( j == goal.x+CELL_SIZE/2 && i == COL-2-goal.y+CELL_SIZE/2) cout << "G ";
            else cout << ". ";
        }

        cout << endl;
    }
    cout << endl;
}

// Define a function for representing the path
void RRT::trace(const vector<Node>& path){

    // Print the path
    if (path.empty()) {
        cout << "No path found!" << endl;
    } else {
        cout << "Path found:" << endl;
        for (const Node& node : path) {
            cout << "(" << node.point.x << ", " <<  node.point.y << ") ";
        }
        cout << endl;
    }
}

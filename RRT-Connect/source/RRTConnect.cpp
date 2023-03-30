/*
RRT-Connect path planning algorithm for grid based map
Author: Hossein Rezaeifar
Copyright: Copyright (c) 2023, Hossein Rezaeifar
License: MIT
Date: March 2023
*/

#include <bits/stdc++.h>
#include "RRTConnect.h"
#include <algorithm>

using namespace std;

// Class constructor
RRTConnect::RRTConnect(vector<vector<Cell>> mgrid, Point mstart, Point mgoal){

    grid = mgrid;
    start = mstart;
    goal = mgoal;
    ROW = grid.size();
    COL = grid[0].size();
}

// Define a function to calculate the distance between two points
double RRTConnect::distance(const Point& p1, const Point& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

double RRTConnect::Angle(const Point& p1, const Point& p2){
    return atan2(double(p2.y - p1.y), double(p2.x - p1.x));
}

// Define a function to check if a point is within the bounds of the workspace
bool RRTConnect::withinBounds(const Point& p) {
    return p.x >= 0 && p.x < (ROW-2) * CELL_SIZE && p.y >= 0 && p.y < (COL-2) * CELL_SIZE;
}

// Define a function to check if a point is within an obstacle
bool RRTConnect::withinObstacle(const Point& p, const vector<vector<Cell>>& grid) {
    for (int i = 0 ; i < ROW; i++){
        for (int j = 0; j < COL; j++){
            if (grid[i][j].obstacle && distance(p, grid[i][j].center) <= ROBOT_RADIUS) {
                return true;
            }
        }
    }
    return false;
}

// Bresenham algorithm
vector<vector<int>> RRTConnect::bresenham(double x0, double y0, double x1, double y1) {

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
bool RRTConnect::clearPath(const Point& p1, const Point& p2, const vector<vector<Cell>>& grid) {
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
Point RRTConnect::generateRandomPoint() {
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<double> dis1(0.0, (ROW-2) * CELL_SIZE);
    uniform_real_distribution<double> dis2(0.0, (COL-2) * CELL_SIZE);
    double x = dis1(gen);
    double y = dis2(gen);
    return {x, y};
}

// Define a function to find the index of the nearest node in the RRT tree
int RRTConnect::nearestNodeIndex(const vector<Node>& tree, const Point& p) {
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
Node RRTConnect::createNewNode(const Point& p, int parentIndex) {
    return {p, parentIndex};
}

// Define a function to find if two tree are close to each other
tuple<bool,int> RRTConnect::test_goal(const Node& p1, const vector<Node>& node_list, double tolerance){
    for (int i = 0; i < node_list.size(); i++) {
        double dis = distance(p1.point, node_list[i].point);

        if ((tolerance > dis) && clearPath(p1.point,node_list[i].point, grid)){
            return make_tuple(true, i);
        }
    }

    return make_tuple(false, 0);
}

// Define the RRT-Connect algorithm with obstacle avoidance
vector<Node> RRTConnect::rrtConnect(const Point& start, const Point& goal, const vector<vector<Cell>>& grid) {

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

    // Define parameters
    int end_t =0;
    int end_i =0;
    int last_i, last_t;

    vector<Node> tree1, tree2, path;
    tree1.push_back({start, -1}); // Add the start node to tree1
    tree2.push_back({goal, -1}); // Add the goal node to tree2

    for (int i = 0; i < MAX_ITER; i++) {
        Point randPoint1 = generateRandomPoint(); // Generate a random point for tree1
        Point randPoint2 = generateRandomPoint(); // Generate a random point for tree2
        int nearestIndex1 = nearestNodeIndex(tree1, randPoint1); // Find the nearest node in tree1
        int nearestIndex2 = nearestNodeIndex(tree2, randPoint2); // Find the nearest node in tree2
        Point nearestPoint1 = tree1[nearestIndex1].point;
        Point nearestPoint2 = tree2[nearestIndex2].point;
        Point newPoint1, newPoint2;

        // tree1
        if (clearPath(nearestPoint1, randPoint1,
                      grid)) { // Check if there is a clear path to the random point from tree1
            if (distance(nearestPoint1, randPoint1) <
                MAX_DIS) { // If the random point is very close to the nearest node in tree1, set the new point to be the random point
                newPoint1 = randPoint1;
            } else { // Otherwise, set the new point to be the point on the path that is one grid cell away from the nearest node in tree1

                double theta = Angle (nearestPoint1, randPoint1);
                newPoint1 = {nearestPoint1.x + MAX_DIS * cos(theta), nearestPoint1.y + MAX_DIS * sin(theta)};
            }
            if (!withinObstacle(newPoint1, grid)) { // Check if the new point is within an obstacle
                int newNodeIndex1 = tree1.size(); // Add the new node to tree1
                tree1.push_back(createNewNode(newPoint1, nearestIndex1));
            }
        }

        tuple<bool, int> test_i = test_goal(tree1.back(), tree2, MAX_DIS);

        // Check if the node is close to another tree
        if (get<0>(test_i)) {

            end_t = 0;
            end_i = 1;
            last_t = get<1>(test_i);
            break;
        }

        // tree2
        if (clearPath(nearestPoint2, randPoint2,
                      grid)) { // Check if there is a clear path to the random point from tree1

            if (distance(nearestPoint2, randPoint2) <
                MAX_DIS) { // If the random point is very close to the nearest node in tree1, set the new point to be the random point

                newPoint2 = randPoint2;
            } else { // Otherwise, set the new point to be the point on the path that is one grid cell away from the nearest node in tree1
                double theta = Angle (nearestPoint2, randPoint2);
                newPoint2 = {nearestPoint2.x + MAX_DIS * cos(theta), nearestPoint2.y + MAX_DIS * sin(theta)};
            }

            if (!withinObstacle(newPoint2, grid)) { // Check if the new point is within an obstacle
                int newNodeIndex1 = tree2.size(); // Add the new node to tree1
                tree2.push_back(createNewNode(newPoint2, nearestIndex2));
            }
        }

        tuple<bool, int> test_t = test_goal(tree2.back(), tree1, MAX_DIS);

        // Check if the node is close to another tree
        if (get<0>(test_t)) {

            end_t = 1;
            end_i = 0;
            last_i = get<1>(test_t);
            break;
        }
    }

    Node node;

    // Reconstruct path by working backwards from target
    if (end_t == 1 && last_i < MAX_ITER) {

        node = tree2.back();
        while (node.parent != -1){
            path.push_back(node);
            node = tree2[node.parent];
        }
        path.push_back(node);
        reverse(path.begin(),path.end());

        node = tree1.back();

        while (node.parent != -1){
            path.push_back(node);
            node = tree1[node.parent];
        }
        path.push_back(node);
        reverse(path.begin(),path.end());
    }

    else if (end_i == 1 && last_t < MAX_ITER){

        node = tree2[last_t];

        while (node.parent != -1){
            path.push_back(node);
            node = tree2[node.parent];
        }
        path.push_back(node);
        reverse(path.begin(),path.end());

        node = tree1.back();

        while (node.parent != -1){
            path.push_back(node);
            node = tree1[node.parent];
        }
        path.push_back(node);
        reverse(path.begin(),path.end());
    }
    else{

        path = vector<Node>();
    }

    return path;
}

// Define a function to present the initial map
void RRTConnect::initmap(){

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

// Define a function to present the path
void RRTConnect::trace(vector<Node>& path){

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
#include <bits/stdc++.h>
#include <vector>
#include "source/RRT.h"

using namespace std;

int main() {

    // Required variables
    int ROW, COL, n_o, x_o, y_o, in_gs, h;
    vector<vector<int>> obs;
    Point start;
    Point goal;

    // User defines the number of rows and columns
    cout << "Please type the number of rows: ";
    cin >> ROW;
    cout << "Please type the number of columns: ";
    cin >> COL;

    // User determines the location of the obstacle
    cout << "Please determine the number of cells acts as obstacles: ";
    cin >> n_o;
    cout << "Please enter the location of each obstacle:" << endl;

    for (int i = 0; i < n_o; i++) {

        cout << "x" << i+1 << ": ";
        cin >> x_o;
        cout << "y" << i+1 << ": ";
        cin >> y_o;
        obs.push_back({x_o,y_o});
    }

    // User determine the Start Cell
    cout << "Please determine the location of the start Cell with integers {Sx, Sy}: "<< endl;
    for (int i = 0; i < 2; i++) {

        cin >> in_gs;
        if (i == 0) start.x = in_gs - CELL_SIZE/2 ;
        else start.y = in_gs - CELL_SIZE/2;
    }

    // Determine Goal Cell
    cout << "Please determine the location of the goal Cell with integers {Gx, Gy}: " << endl;
    for (int i = 0; i < 2; i++) {

        cin >> in_gs;
        if (i == 0) goal.x = in_gs - CELL_SIZE/2 ;
        else goal.y = in_gs - CELL_SIZE/2;
    }

    cout << endl;
    //cout << "The start point would be: (" << start.y << ", " << ROW-start.x << ")" << endl;
    //cout << "The goal point would be: (" << goal.y << ", " << ROW-goal.x << ")" << endl;

    cout << "The start point would be: (" << start.x << ", " << start.y << ")" << endl;
    cout << "The goal point would be: (" << goal.x << ", " << goal.y << ")" << endl;

    // Define the grid
    vector<vector<Cell>> grid(COL+2,vector<Cell>(ROW+2));
    for (int i = 0; i < COL+2; i++) {
        for (int j = 0; j < ROW+2; j++) {
            Point center = {i * CELL_SIZE - CELL_SIZE / 2, j * CELL_SIZE - CELL_SIZE / 2};
            grid[i][j].center = center;
            grid[i][j].obstacle = false;
            if (i == 0 || j == 0 || i == COL+1 || j == ROW+1) {
                grid[i][j].obstacle = true;
            }
        }
    }

    for (int k = 0; k < n_o; k++) {
        grid[obs[k][0]][obs[k][1]].obstacle = true;
    }


    // Create RRT object
    RRT train(grid, start, goal);
    // RRT
    vector<Node> path = train.rrtpath( start, goal, grid);
    //Print initial map
    train.initmap();
    // Print output
    train.trace( path);


    return 0;
}

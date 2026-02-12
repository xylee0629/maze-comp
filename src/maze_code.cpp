#include <iostream>
#include <stack>
#include <algorithm>
#include "API.h"

const int mazeSize = 8;

const uint8_t north = 0x01;
const uint8_t east = 0x02;
const uint8_t south = 0x03;
const uint8_t west = 0x04;
enum Direction {robotNorth = 0, robotEast = 1, robotSouth = 2, robotWest = 3};

struct Map{
	int value;
	uint8_t wall;
};

struct Robot{
    int robotX, robotY;
    enum Direction dir;
};
		
Map maze[mazeSize][mazeSize];
//-> set robot coordinates to 0, 0, direction is north
Robot robot = {0, 0, robotNorth};

//1. Initialise maze walls (all empty)
//-> set all maze[][].values to 255
void initialiseMap()
{
    for (int i = 0; i < mazeSize; i++)
    {
        for (int j = 0; j < mazeSize; j++)
        {
            maze[i][j].wall = 0;
            maze[i][j].value = 255;
        }
    }
}

//2. update walls (track wall on specific cell)
void wallCheck(struct Map maze, struct Robot robot)
{
// check for robot orientation. any time a turn is performed, change the robot direction. (cardinal robot facing north = 0, robot facing east = 1, south = 2, west = 3)
	// Check for wall direction
    switch(robot.dir){
        case robotNorth:
            maze[robot.robotX][robot.robotY].wall = 
    }
	cardinal = wall
	if the robot is facing north, north has wall (same for all directions). 
	(do i just block out the adjacent wall too?)

	// left wall: sensorValue[0] && sensorValue[1] = 0. 
	(cardinal = 3) % 4 = wall 
	if robot facing north, west has wall. 
	if robot facing east, north has wall.
	if robot facing south, east has wall.
	if robot facing west,  south has wall.

	// right wall: sensorValue[4] && sensorValue[5] = 0.
	(cardinal + 1) % 4 = wall
}
	
3. update cell value 




2. if current cell is not destination, check all adjacent cell values and current cell value 
if current cell value = 1 + min(adjacent cell value)
	update distance (perform floodfill)

void minDistance()
{
	int minDistance = 255;
	// if the current cell does not have a north wall, and current yAxis is less than 7 (map[0][0] to 7,7)
		compare between minDistance and wall value 
	(repeat for all walls)
	return minDistance
}

if
	


void floodfill(int wall, int xAxis, int yAxis)
{
  
}


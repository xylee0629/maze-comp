#include <iostream>
#include <vector>
#include <stack>
#include <algorithm>
#include <cstdint>


const int MAZE_SIZE = 8;

// Direction Enums (match your request)
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Bitmasks for Wall Representation
// North=Bit0, East=Bit1, South=Bit2, West=Bit3
const uint8_t MASK_NORTH = 0x01; // 0001
const uint8_t MASK_EAST  = 0x02; // 0010
const uint8_t MASK_SOUTH = 0x04; // 0100
const uint8_t MASK_WEST  = 0x08; // 1000

struct Coord {
    int x, y;
};

struct MazeCell {
    int distance;
    uint8_t walls; // 4-bit wall data
};


class Micromouse {
public:
    MazeCell maze[MAZE_SIZE][MAZE_SIZE];
    int robotX, robotY;
    Direction robotDir;
    
    // Helper: Check if a coordinate is within grid bounds
    bool isValid(int x, int y) {
        return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
    }

public:
    Micromouse() {
        robotX = 0;
        robotY = 0;
        robotDir = NORTH;
        initializeMaze();
    }

    // Initialize maze with optimistic distances (assuming no walls)
    // Destination is usually the center 2x2 square: (7,7), (7,8), (8,7), (8,8)
    void initializeMaze() {
        for (int x = 0; x < MAZE_SIZE; x++) {
            for (int y = 0; y < MAZE_SIZE; y++) {
                maze[x][y].walls = 0; // Assume no walls initially
                
                // Calculate Manhattan distance to the closest center cell
                // Center is roughly 7.5, 7.5
                int distX = std::min(abs(x - 7), abs(x - 8));
                int distY = std::min(abs(y - 7), abs(y - 8));
                maze[x][y].distance = distX + distY;
            }
        }
    }

    // ---------------------------------------------------------
    // 1. Wall Detection & Updates
    // ---------------------------------------------------------

    // Update walls for specific cell AND its shared neighbor
    void setWall(int x, int y, Direction dir) {
        if (!isValid(x, y)) return;

        switch (dir) {
            case NORTH:
                maze[x][y].walls |= MASK_NORTH;
                if (isValid(x, y + 1)) maze[x][y + 1].walls |= MASK_SOUTH;
                break;
            case EAST:
                maze[x][y].walls |= MASK_EAST;
                if (isValid(x + 1, y)) maze[x + 1][y].walls |= MASK_WEST;
                break;
            case SOUTH:
                maze[x][y].walls |= MASK_SOUTH;
                if (isValid(x, y - 1)) maze[x][y - 1].walls |= MASK_NORTH;
                break;
            case WEST:
                maze[x][y].walls |= MASK_WEST;
                if (isValid(x - 1, y)) maze[x - 1][y].walls |= MASK_EAST;
                break;
        }
    }

    // Call this function after reading your IR sensors
    void updateWallsFromSensors(bool leftSensor, bool frontSensor, bool rightSensor) {
        if (frontSensor) {
            setWall(robotX, robotY, robotDir);
        }
        if (leftSensor) {
            // (dir + 3) % 4 is mathematically equivalent to (dir - 1) wrapped around
            setWall(robotX, robotY, static_cast<Direction>((robotDir + 3) % 4));
        }
        if (rightSensor) {
            setWall(robotX, robotY, static_cast<Direction>((robotDir + 1) % 4));
        }
    }

    // ---------------------------------------------------------
    // 2. Flood Fill Algorithm (Stack-Based)
    // ---------------------------------------------------------

    // Helper to find min distance among OPEN neighbors
    int getMinOpenNeighborDistance(int x, int y) {
        int minD = 255; // Max value placeholder

        // Check North
        if (!(maze[x][y].walls & MASK_NORTH) && isValid(x, y + 1)) 
            minD = std::min(minD, maze[x][y + 1].distance);
        
        // Check East
        if (!(maze[x][y].walls & MASK_EAST) && isValid(x + 1, y)) 
            minD = std::min(minD, maze[x + 1][y].distance);
        
        // Check South
        if (!(maze[x][y].walls & MASK_SOUTH) && isValid(x, y - 1)) 
            minD = std::min(minD, maze[x][y - 1].distance);

        // Check West
        if (!(maze[x][y].walls & MASK_WEST) && isValid(x - 1, y)) 
            minD = std::min(minD, maze[x - 1][y].distance);

        return minD;
    }

    void floodFill() {
        std::stack<Coord> s;
        
        // Push current cell onto stack
        s.push({robotX, robotY});

        while (!s.empty()) {
            Coord c = s.top();
            s.pop();

            // Goal cells (distance 0) should never change
            if (maze[c.x][c.y].distance == 0) continue;

            int minNeighbor = getMinOpenNeighborDistance(c.x, c.y);

            // If current cell is NOT (1 + min_neighbor), update it!
            if (maze[c.x][c.y].distance != 1 + minNeighbor) {
                maze[c.x][c.y].distance = 1 + minNeighbor;

                // Push all OPEN neighbors to stack to re-check them
                if (!(maze[c.x][c.y].walls & MASK_NORTH) && isValid(c.x, c.y + 1)) 
                    s.push({c.x, c.y + 1});
                if (!(maze[c.x][c.y].walls & MASK_EAST) && isValid(c.x + 1, c.y)) 
                    s.push({c.x + 1, c.y});
                if (!(maze[c.x][c.y].walls & MASK_SOUTH) && isValid(c.x, c.y - 1)) 
                    s.push({c.x, c.y - 1});
                if (!(maze[c.x][c.y].walls & MASK_WEST) && isValid(c.x - 1, c.y)) 
                    s.push({c.x - 1, c.y});
            }
        }
    }

    // ---------------------------------------------------------
    // 3. Movement Logic (Next Step)
    // ---------------------------------------------------------

    void decideNextMove() {
        int currentDist = maze[robotX][robotY].distance;
        int bestDist = 255;
        Direction bestDir = robotDir; // Default to staying put (or 180 turn)

        // Array to easily iterate directions: N, E, S, W
        int dx[4] = {0, 1, 0, -1};
        int dy[4] = {1, 0, -1, 0};
        uint8_t masks[4] = {MASK_NORTH, MASK_EAST, MASK_SOUTH, MASK_WEST};

        // 1. Scan all neighbors to find the absolute minimum distance
        for(int i = 0; i < 4; i++) {
            // If no wall exists in this direction
            if (!(maze[robotX][robotY].walls & masks[i])) {
                int nx = robotX + dx[i];
                int ny = robotY + dy[i];

                if (isValid(nx, ny)) {
                    if (maze[nx][ny].distance < bestDist) {
                        bestDist = maze[nx][ny].distance;
                    }
                }
            }
        }

        // 2. Tie-Breaker: Prefer going STRAIGHT if that neighbor has the best distance
        int forwardIndex = robotDir;
        int nx = robotX + dx[forwardIndex];
        int ny = robotY + dy[forwardIndex];
        
        bool canGoStraight = !(maze[robotX][robotY].walls & masks[forwardIndex]);
        
        // If we can go straight and the cell in front is the best option, DO IT.
        if (canGoStraight && isValid(nx, ny) && maze[nx][ny].distance == bestDist) {
            bestDir = (Direction)forwardIndex;
        } else {
            // Otherwise, pick the first neighbor that matches the best distance
            for(int i = 0; i < 4; i++) {
                if (!(maze[robotX][robotY].walls & masks[i])) {
                     int neighborX = robotX + dx[i];
                     int neighborY = robotY + dy[i];
                     if (isValid(neighborX, neighborY) && maze[neighborX][neighborY].distance == bestDist) {
                         bestDir = (Direction)i;
                         break; 
                     }
                }
            }
        }

        // 3. Execute Move (Simulation)
        moveTo(bestDir);
    }

    void moveTo(Direction dir) {
        // Here is where you would send commands to motors (e.g. turnLeft(), moveForward())
        
        // Simulating the coordinate change
        if (dir == NORTH) robotY++;
        else if (dir == EAST) robotX++;
        else if (dir == SOUTH) robotY--;
        else if (dir == WEST) robotX--;

        robotDir = dir;
        
        std::cout << "Moved to (" << robotX << ", " << robotY << ") Facing: " << robotDir << std::endl;
    }
    
    // Check if we reached the center
    bool isFinished() {
        return maze[robotX][robotY].distance == 0;
    }
    
    // Getter for current position for the simulation loop
    int getX() { return robotX; }
    int getY() { return robotY; }
};

// ==========================================
// Main Loop (Simulation)
// ==========================================

int main() {
    Micromouse mouse;
    
    int steps = 0;
    while (!mouse.isFinished() && steps < 1000) {
        std::cout << "Step " << steps << ": At (" << mouse.getX() << "," << mouse.getY() << ")\n";

        // 1. READ SENSORS
        // In simulation, we fake this. 
        // Real code: bool left = readLeftIR(); bool front = readFrontIR(); ...
        bool leftWall = false;
        bool frontWall = false; 
        bool rightWall = false;
        
        // Example: Pretend there is a wall at (0,1) blocking North movement
        if (mouse.getX() == 0 && mouse.getY() == 0) {
            frontWall = true; // Block North
            rightWall = false; // East is open
        }

        // 2. UPDATE WALLS
        mouse.updateWallsFromSensors(leftWall, frontWall, rightWall);

        // 3. FLOOD FILL
        mouse.floodFill();

        // 4. DECIDE & MOVE
        mouse.decideNextMove();
        
        steps++;
        std::cout << "--------------------\n";
    }

    std::cout << "Destination Reached!" << std::endl;
    return 0;
}
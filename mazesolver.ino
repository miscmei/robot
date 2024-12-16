#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *br = AFMS.getMotor(1);
Adafruit_DCMotor *fr = AFMS.getMotor(2);
Adafruit_DCMotor *bl = AFMS.getMotor(3);
Adafruit_DCMotor *fl = AFMS.getMotor(4);

//The distance sensors:
int le = 7; //left echo
int ltr = 8; // left trigger
int re = 9; //right echo
int rtr = 10; //right trigger
int be = 11; //back echo
int btr = 12; //back trigger
int fe = 5; //front echo
int ftr = 6; // back trigger
long duration; //used for sensor distance calculations
int distance; //used for sensor distance calculations
float runningTotal; // for getting rid of noise
int N = 10; // for getting rid of noise
int button = 13; // 

//Detecting Walls:
bool isWalls[4]; //at current cell, keeps track of surrounding walls in the order front, right, back, left
float lt = 4; //the target distance from left sensor to left wall if centered
float rt = 4; //the target distance from right sensor to right wall if centered
float bt = 2; //the target distance from back sensor to back wall if centered
int ft = 2; //the target distance from front sensor to front wall if centered
int lmax = 11; //the maximum distance from left sensor a left wall can be if there exists a left wall (any farther means no wall)
int rmax = 11; //the maximum distance from right sensor a right wall can be if there exists a right wall (any farther means no wall)
int bmax = 11; //the maximum distance from back sensor a back wall can be if there exists a back wall (any farther means no wall)
int ffmax = 11; //the maximum distance from front sensor a front wall can be if there exists a front wall (any farther means no wall)

//Floodfill:
int queue[300]; //make a queue of size |board| ^2 ... two pointer array
int left_pointer = 0; //pop from the queue
int right_pointer = 0; //push onto the queue
int current_min_neightbor; //value of the minimum 

struct cell { //a cell structure to hold:
    int dist; //the distance of the cell from the center
    bool nWall; //the presence of a north wall (direction 0)
    bool eWall; //the presence of an east wall (direction 1)
    bool sWall; //the presence of a south wall (direction 2)
    bool wWall; //the presence of a west wall (direction 3)
    bool visited; //whether or not that cell has been visited
    int cellNum; //the numerical label for that cell
    int i; //the X coordinate of that cell on the board
    int j; //the Y coordinate of that cell on the board
};

cell board[5][5]; // 5x5 grid of cells

//Keeping track of the robot:
int positionX = 0; //current X position..stop when 2,2
int positionY = 0; //current Y position...stop when 2,2
int orientation = 0; //direction the robot is facing...0 = forward, 1 = right, 2 = back, 3 = left 

void setup() {
  AFMS.begin();

  //setting speeds for all motors
  fl->setSpeed(50);
  fr->setSpeed(50);
  bl->setSpeed(50);
  br->setSpeed(50);

  //initiailizing the board of cells
  int cellNum = 0; //the first cell number
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      if(i == 2 && j == 2) {
        board[i][j].dist = 0; // middle cell
      } else if ((i == 1 && j == 2) || (i == 2 && j == 1) || (i == 3 && j == 2) || (i == 2 && j == 3)) {
        board[i][j].dist= 1; //cells that are a distance of 1 from the center
      } else if ((i == 0 && j == 0) || (i == 0 && j == 4) || (i == 4 && j == 0) || (i == 4 && j == 4)) {
        board[i][j].dist = 4; //the corner cells
      } else if ((i == 0 && j == 1) || (i == 0 && j == 3) || (i == 1 && j == 0) || (i == 0 && j == 4) || (i == 3 && j == 0) || (i == 3 && j == 4) || (i == 4 && j == 1) || (i == 4 && j == 3)) {
        board[i][j].dist = 3; //cells that are a distance of 3 from the center
      } else {
        board[i][j].dist = 2; //cells that are a distance of 2 from the center
      }

      board[i][j].nWall = false; //initialize presence of north wall to be false for all cells
      board[i][j].eWall = false; //initialize presence of east wall to be false for all cells
      board[i][j].sWall = false; //initialize presence of south wall to be false for all cells
      board[i][j].wWall = false; //initialize presence of west wall to be false for all cells
      board[i][j].visited = false; //initialize all cells as unvisted
      board[i][j].cellNum = cellNum; //initialize the cell name to the current cellNum
      board[i][j].i = i; //initialize the X position as the current i
      board[i][j].j = j; //initialize the Y position as the current j
    
      //establish walls along the edges of the board:
      if (i==0) {
        board[i][j].sWall = true;
      } 
      if (j==0) {
        board[i][j].wWall = true;
      }
      if (i==4) {
        board[i][j].nWall = true;
      }
      if (j==4) {
        board[i][j].eWall = true;
      }

      cellNum +=1; //increment cellNum
    }
  }

  pinMode(ltr, OUTPUT); // Sets the triggerPin as an Output
  pinMode(le, INPUT); // Sets the echoPin as an Input
  pinMode(rtr, OUTPUT); // Sets the triggerPin as an Output
  pinMode(re, INPUT); // Sets the echoPin as an Input
  pinMode(btr, OUTPUT); // Sets the triggerPin as an Output
  pinMode(be, INPUT); // Sets the echoPin as an Input
  pinMode(ftr, OUTPUT); // Sets the triggerPin as an Output
  pinMode(fe, INPUT); // Sets the echoPin as an Input
  pinMode(button, INPUT); // Sets button in as input
  Serial.begin(9600); // Starts the serial communication

}


//distance testing:
// void loop() {
//   moveForwardOne();
//   turnRight();
//   moveForwardOne();
//   moveBackOne();
//   turnLeft();
//   moveBackOne();
//   while(true){
//   stop();
//   }
// }


void loop() {
  while ((positionX == 2) && (positionY == 2)) { //if the robot is at the center
    turnLeft(); //celebrate victory by spinning in circles
    // button has been pressed, resetting position to run maze again
    if (digitalRead(button) == HIGH) {
      positionX = 0;
      positionY = 0;
      delay(3000);
    }
  }
  //if the robot has not made it to the center:

  checkWalls(); //check current cell's surrounding walls
  int move_direction = findMinNeighbor(positionX, positionY); //determines which direction the robot should move to go to the cell with minimum distance from center

  //move to the smallest number until you can't move anymore:
  if (current_min_neightbor < board[positionX][positionY].dist) { //if there is a minimum neighbor found
    if (orientation == move_direction) { //if the direction the robot wants to move is the same as the current orientation
      moveForwardOne(); //move forward one cell
    }
    else if (orientation-1 == move_direction) { //if the direction the robot wants to move is one less than the orientation
      turnLeft(); //turn left 
      moveForwardOne(); //move forward one cell
    }
    else if ((move_direction == 3) && (orientation-1 == -1)) { //if the robot wants to move in direction 3 and the orientation-1 is -1
      turnLeft(); //turn left
      moveForwardOne(); //move forward one cell
    }
    else if (orientation+1 == move_direction) { //if the direction the robot wants to move is one more than the orientation
      turnRight();  //turn right
      moveForwardOne(); //move forward one cell
    }
    else if ((move_direction == 0) && (orientation+1 == 4)) { //if the robot wants to move in direction 0 and the orientation+1 is 4
      turnRight(); //turn right
      moveForwardOne(); //move forward one cell
    }
    else {
      moveBackOne(); //move backward one cell
    }
  }

  else if (current_min_neightbor >= board[positionX][positionY].dist) { //if the robot's current cell's distance is smaller than all it's accessible neighbors
    board[positionX][positionY].dist += 1; //increase the current cell's distance to the center by one
    // floodfill();
  }

}


// if the robot's current cell's distance is smaller than all it's accessible neighbors run the rest of the floodfill algorithm
void floodfill() {
  stop(); //stop moving
  push(board[positionX][positionY].cellNum); //push the current position
  while (left_pointer < right_pointer) { //while there are items in the queue
        int current_cell_num = pop(); //pop the first cell of the queue and set is at the current_cell
        cell current_cell = getCell(current_cell_num);
        // if (current_cell.visited == true) { //if we know what walls surround the cell
          int dir = findMinNeighbor(current_cell.i, current_cell.j); //find min neightbor of that cell
          if (current_cell.dist <= current_min_neightbor) { //if curent cell's value is <= minimum of neighbors
            current_cell.dist = current_min_neightbor + 1; //set current_cell's value to min + 1
            //add accessible neighbors to the queue:
            if (board[current_cell.i][current_cell.j].nWall == false) { 
              push(board[current_cell.i + 1][current_cell.j].cellNum);
            }
            if (board[current_cell.i][current_cell.j].eWall == false) { 
              push(board[current_cell.i][current_cell.j + 1].cellNum); 
            }
            if (board[current_cell.i][current_cell.j].sWall == false) { 
              push(board[current_cell.i - 1][current_cell.j].cellNum); 
            }
            if (board[current_cell.i][current_cell.j].wWall == false) { 
              push(board[current_cell.i][current_cell.j - 1].cellNum); 
            }
          } else { //if curent cell's value is > minimum of neighbors
            continue; //do nothing
          }
  }
}


//given a numerical name of a cell, return the cell object
cell getCell(int num){
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
        if (board[i][j].cellNum == num) { //search through all of the cells in the 5x5 board until one matches the num
          return board[i][j]; //return that cell object
        }
    }
  }
}


//determines where the walls are in the current box (in relation to the gird, not the robot ortientation)
void checkWalls() {
  int ls= calcDistance(ltr, le); //get the distance to the wall on the left
  int rs= calcDistance(rtr, re); //get the distance to the wall on the right
  int fs= calcDistance(ftr, fe); //get the distance to the wall in front
  int bs= calcDistance(btr, be); //get the distance to the wall in back

  if (orientation == 0) { //if the robot is facing forward
    if (fs < ffmax) { //if the sensor is picking up a wall
      isWalls[0] = true; //there is a wall in the forward direction
    } else { //if the sensor is not picking up a wall
      isWalls[0]= false; //there is not a wall in the forward direction
    }

    if (rs < rmax) { //if the sensor is picking up a wall
      isWalls[1] = true; //there is a wall in the right direction
    } else { //if the sensor is not picking up a wall
      isWalls[1] = false; //there is not a wall in the right direction
    }

    if (bs < bmax) { //if the sensor is picking up a wall
      isWalls[2] = true; //there is a wall in the back direction
    } else { //if the sensor is not picking up a wall
      isWalls[2] = false; //there is not a wall in the back direction
    }

    if (ls < lmax) { //if the sensor is picking up a wall
      isWalls[3] = true; //there is a wall in the left direction
    } else { //if the sensor is not picking up a wall
      isWalls[3] = false; //there is not a wall in the left direction
    }
  }

  if (orientation == 1) { //if the robot is facing right
    if (fs < ffmax) { //if the sensor is picking up a wall
      isWalls[1] = true; //there is a wall in the right direction
    } else { //if the sensor is not picking up a wall
      isWalls[1]= false; //there is not a wall in the right direction
    }

    if (rs < rmax) { //if the sensor is picking up a wall
      isWalls[2] = true; //there is a wall in the back direction
    } else { //if the sensor is not picking up a wall
      isWalls[2] = false; //there is not a wall in the back direction
    }

    if (bs < bmax) { //if the sensor is picking up a wall
      isWalls[3] = true; //there is a wall in the left direction
    } else { //if the sensor is not picking up a wall
      isWalls[3] = false; //there is not a wall in the left direction
    }

    if (ls < lmax) { //if the sensor is picking up a wall
      isWalls[0] = true; //there is a wall in the forward direction
    } else { //if the sensor is not picking up a wall
      isWalls[0] = false; //there is not a wall in the forward direction
    }
  }

  if (orientation == 2) { //if the robot is facing backwards
    if (fs < ffmax) { //if the sensor is picking up a wall
      isWalls[2] = true; //there is a wall in the back direction
    } else { //if the sensor is not picking up a wall
      isWalls[2]= false; //there is not a wall in the back direction
    }

    if (rs < rmax) { //if the sensor is picking up a wall
      isWalls[3] = true; //there is a wall in the left direction
    } else { //if the sensor is not picking up a wall
      isWalls[3] = false; //there is not a wall in the left direction
    }

    if (bs < bmax) { //if the sensor is picking up a wall
      isWalls[0] = true; //there is a wall in the forward direction
    } else { //if the sensor is not picking up a wall
      isWalls[0] = false; //there is not a wall in the forward direction
    }

    if (ls < lmax) { //if the sensor is picking up a wall
      isWalls[1] = true; //there is a wall in the right direction
    } else { //if the sensor is not picking up a wall
      isWalls[1] = false; //there is not a wall in the right direction
    }
  }

  if (orientation == 3) { //if the robot is facing left
    if (fs < ffmax) { //if the sensor is picking up a wall
      isWalls[3] = true; //there is a wall in the left direction
    } else { //if the sensor is not picking up a wall
      isWalls[3]= false; //there is not a wall in the left direction
    }

    if (rs < rmax) { //if the sensor is picking up a wall
      isWalls[0] = true; //there is a wall in the forward direction
    } else { //if the sensor is not picking up a wall
      isWalls[0] = false; //there is not a wall in the forward direction
    }

    if (bs < bmax) { //if the sensor is picking up a wall
      isWalls[1] = true; //there is a wall in the right direction
    } else { //if the sensor is not picking up a wall
      isWalls[1] = false; //there is not a wall in the right direction
    }

    if (ls < lmax) { //if the sensor is picking up a wall
      isWalls[2] = true; //there is a wall in the back direction
    } else { //if the sensor is not picking up a wall
      isWalls[2] = false; //there is not a wall in the back direction
    }
  }

  board[positionX][positionY].visited = true; //set that cell as visited
  board[positionX][positionY].nWall = isWalls[0]; //keep track of the presence of the north wall
  board[positionX][positionY].eWall = isWalls[1]; //keep track of the presence of the east wall
  board[positionX][positionY].sWall = isWalls[2]; //keep track of the presence of the south wall
  board[positionX][positionY].wWall = isWalls[3]; //keep track of the presence of the west wall

}

//determines the minimum neighbor out of the available cells
int findMinNeighbor(int X, int Y) {
  int minimum_neighbor = 100; //initialize to high value to be updated
  int direction; //initialize variable to hold what direction the minimum neighbor is in

  if (board[X][Y].nWall == false) { //it is possible to go in direction 0
          if (board[X+1][Y].dist < minimum_neighbor) { //if the board in front is less than the minimum
            minimum_neighbor = board[X+1][Y].dist; //the new minimum is the board in front
            direction = 0; //plan to move forward
          }
  }

  if (board[X][Y].eWall == false) { //it is possible to go in direction 1
          if (board[X][Y+1].dist < minimum_neighbor) { //if the board to the right is less than the minimum
            minimum_neighbor = board[X][Y+1].dist; //the new minimum is the board on the right
            direction = 1; //plan to move right
          }
  }

  if (board[X][Y].sWall == false) { //it is possible to go in direction 2
          if (board[X-1][Y].dist < minimum_neighbor) { //if the board in the back is less than the minimum
            minimum_neighbor = board[X-1][Y].dist; //the new minimum is the board in the back
            direction = 2; //plan to move backward
          }
  }

  if (board[X][Y].wWall == false) { //it is possible to go in direction 3
          if (board[X][Y-1].dist < minimum_neighbor) { //if the board to the left is less than the minimum
            minimum_neighbor = board[X][Y-1].dist; //the new minimum is the board on the left
            direction = 3; //plan to move left
          }
  }
  current_min_neightbor = minimum_neighbor; //update the global variable current_min_neightbor to have the value of the current cell's minimum accessible neighbor
  return direction; //return the direction of the minimum accessible neighbor       
  }



//this method doesn't work due to sensor reading inaccuracies, but was intended to center the robot in a cell
void center() {
  int ls= calcDistance(ltr, le); //get the distance to the wall on the left
  int rs= calcDistance(rtr, re); //get the distance to the wall on the right
  int fs= calcDistance(ftr, fe); //get the distance to the wall in front
  int bs= calcDistance(btr, be); //get the distance to the wall in back
  if (bs <bmax) { //there is a wall within a certain range for the back sensor
      if (bs > bt) {//if the distance is too far
        backward(); //move backwards
        while (bs > bt) {  //keep moving backwards while the distance is too far
            bs= calcDistance(btr, be); //update the distance reading
            delay(50);
        }
        stop (); //stop adjustment
      }
      if (bs < bt) {//if the distance is too close
        forward(); //move forwards
        while (bs < bt) {  //keep moving forwards while the distance is too close
            bs= calcDistance(btr, be); //update the distance reading
            delay(50);
        }
        stop (); //stop adjustment
      }
  }

  //check front wall
  if (fs < ffmax)  { //there is a wall within a certain range for the front sensor
      if (fs > ft) {//if the distance is too far
        forward(); //move forwards
        while (fs > ft) {  //keep moving forwards while the distance is too far
            fs= calcDistance(ftr, fe); //update the distance reading
            delay(50);
        }
        stop (); //stop adjustment
      }
      if (fs < ft) {//if the distance is too close
        backward(); //move forwards
        while (fs < ft) {  //keep moving backwards while the distance is too close
            fs= calcDistance(ftr, fe); //update the distance reading
            delay(50);
        }
        stop (); //stop adjustment
      }
  }

  //check left wall
  if (ls < lmax)  { //there is a wall within a certain range for the left sensor
      if (ls > lt) {//if the distance is too far
        left(); //move left
        while (ls > lt) { //keep moving left while the distance is too far
            ls= calcDistance(ltr, le); //update the distance reading
            delay(50);
        }
        stop (); //stop adjustment
      }
      if (ls < lt) {//if the distance is too close
        right(); //move forwards
        while (ls < lt) { //keep moving right while the distance is too close
            ls= calcDistance(ltr, le); //update the distance reading
            delay(50);
        }
        stop (); //stop adjustment
      }
  }

  //check right wall
  if (rs < rmax)  { //there is a wall within a certain range for the left sensor
      if (rs > rt) {//if the distance is too far
        right(); //move left
        while (rs > rt) { //keep moving right while the distance is too far
            rs= calcDistance(rtr, re); //update the distance reading
            delay(50);
        }
        stop (); //stop adjustment
      }
      if (rs < rt) {//if the distance is too close
        left(); //move forwards
        while (rs < rt) {  //keep moving left while the distance is too close
            rs= calcDistance(rtr, re); //update the distance reading
            delay(50);
        }
        stop (); //stop adjustment
      }
  }

}

//method to move forward one sqare in the maze
void moveForwardOne() {
  
  forward(); //move forward
  delay(1560); //keep moving forward for 1560 ms

  //update position on board
  if (orientation == 0) {
    positionX += 1; //moved forward one cell
  }
  else if (orientation == 1) {
    positionY +=1; //moved right one cell
  }
  else if (orientation == 2) {
    positionX -=1; //moved back one cell
  }
  else if (orientation == 3) {
    positionY -=1; //moved left one cell
  }



  //use sensors to check in the correct location and adjust accordingly
}
//method to move backward one sqare in the maze
void moveBackOne() {
  
  backward(); //move backward
  delay(1540); //keep moving backward to 1540 ms
  
  //update position on board
  if (orientation == 0) {
    positionX -= 1; //moved back one cell
  }
  else if (orientation == 1) {
    positionY -=1; //moved left one cell
  }
  else if (orientation == 2) {
    positionX +=1; //moved forward one cell
  }
  else if (orientation == 3) {
    positionY +=1; //moved right one cell
  }

}
//method to turn the robot 90 degrees right
void turnRight() {
  //set wheels to turn right
  fl->run(FORWARD);
  fr->run(BACKWARD);
  bl->run(FORWARD);
  br->run(BACKWARD);
  delay(1100); //keep turning right for 1100 ms

  //update the robot's orientation in the maze
  orientation += 1;
  if (orientation >= 4) {
    orientation = 0;
  }
}
//method to turn the robot 90 degrees left
void turnLeft() {
   //set wheels to turn left
  fl->run(BACKWARD);
  fr->run(FORWARD);
  bl->run(BACKWARD);
  br->run(FORWARD);
  delay(1100); //keep turning left for 1100 ms

  //update the robot's orientation in the maze
  orientation -= 1;
  if (orientation < 0) {
    orientation = 3;
  }
}

//method to pop one item from the queue at index left_pointer
int pop() {
  int val = queue[left_pointer]; //get the item from the queue
  left_pointer += 1; //increment the left_pointer
  return val;
}
// method to push one item onto the queue with value val and index right_pointer
void push(int cellNum) {
  queue[right_pointer] = cellNum; //update the queue to hold the cellNum
  right_pointer += 1; //increment the right_pointer
}
//method to set wheels to forward direction
void forward() {
  fl->run(FORWARD);
  fr->run(FORWARD);
  bl->run(FORWARD);
  br->run(FORWARD);
}
//method to set wheels to left direction
void left() {
  fl->run(FORWARD);
  fr->run(BACKWARD);
  bl->run(BACKWARD);
  br->run(FORWARD);
}
//mthod to set wheels to right direction
void right() {
  fl->run(BACKWARD);
  fr->run(FORWARD);
  bl->run(FORWARD);
  br->run(BACKWARD);
}
//method to set wheels to back direction
void backward() {
  fl->run(BACKWARD);
  fr->run(BACKWARD);
  bl->run(BACKWARD);
  br->run(BACKWARD);
}
//method to get all wheels to stop moving
void stop() {
  fl->run(RELEASE);
  fr->run(RELEASE);
  bl->run(RELEASE);
  br->run(RELEASE);
}
//method to turn sensor readings into a usable distance
int calcDistance(int trigNum, int echoNum){
 // define variable
 duration = 0;
 distance = 0;
 runningTotal = 0;
 for (int i=1; i <= N; i++) {
   // clears triggerPin
   digitalWrite(trigNum, LOW);
   delayMicroseconds(2);

   // Sets the triggerPin on HIGH state for 10 micro seconds
   digitalWrite(trigNum, HIGH);
   delayMicroseconds(10);
   digitalWrite(trigNum, LOW);

   // Reads the echoPin, returns the sound wave travel time in microseconds
   duration = pulseIn(echoNum, HIGH);
   runningTotal += duration;
 }
 duration = runningTotal / N;

 // Calculating the distance
 distance = duration * 0.034 / 2;
 return distance;
}

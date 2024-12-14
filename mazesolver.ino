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
int queue[450]; //make a queue of size |board| ^2 ... two pointer array
int left_pointer = 0; //pop from the queue
int right_pointer = 0; //push onto the queue
int current_min_neightbor; 

struct cell {
    int dist;
    bool nWall;
    bool eWall;
    bool sWall;
    bool wWall;
    bool visited;
    int cellNum;
    int i;
    int j;
};

// 5x5 grid of cells
cell board[5][5];

//Keeping track of the robot:
int positionX = 0; //current X position..stop when 2,2
int positionY = 0; //current Y position...stop when 2,2
int orientation = 0; //direction the robot is facing...0 = forward, 1 = right, 2 = back, 3 = left 
int floodFillX;
int floodFillY;

void setup() {
  AFMS.begin();
  fl->setSpeed(50);
  fr->setSpeed(50);
  bl->setSpeed(50);
  br->setSpeed(50);

  int cellNum = 0;
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      if(i == 2 && j == 2) {
        // middle
        board[i][j].dist = 0;
      } else if ((i == 1 && j == 2) || (i == 2 && j == 1) || (i == 3 && j == 2) || (i == 2 && j == 3)) {

        board[i][j].dist= 1;
      } else if ((i == 0 && j == 0) || (i == 0 && j == 0) || (i == 4 && j == 0) || (i == 4 && j == 4)) {
        // corners
        board[i][j].dist = 4;
      } else if ((i == 0 && j == 1) || (i == 0 && j == 3) || (i == 1 && j == 0) || (i == 0 && j == 4) || (i == 3 && j == 0) || (i == 3 && j == 4) || (i == 4 && j == 1) || (i == 4 && j == 3)) {
        board[i][j].dist = 3;
      } else {
        board[i][j].dist = 2;
      }

      board[i][j].nWall = false;
      board[i][j].eWall = false;
      board[i][j].sWall = false;
      board[i][j].wWall = false;
      board[i][j].visited = false;
      board[i][j].cellNum = cellNum;
      board[i][j].i = i;
      board[i][j].j = j;
      

      cellNum +=1;
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
  Serial.begin(9600); // Starts the serial communication

}

void loop() {

  // Serial.println(board[1][0].cellNum);

  while ((positionX == 2) && (positionY == 2)) { //if at the center
    turnLeft(); //celebrate victory by spinning in circles
  }

  checkWalls(); //check current cell's surrounding walls
  int move_direction = findMinNeighbor(positionX, positionY); //determines which direction the robot should move to go to the cell with minimum distance from center

  if (current_min_neightbor < board[positionX][positionY].dist) { //if there is a minimum neighbor found
    if (orientation == move_direction) {
      moveForwardOne();
    }
    else if (orientation-1 == move_direction) {
      turnLeft();
      moveForwardOne();
    }
    else if (orientation+1 == move_direction) {
      turnRight();
      moveForwardOne();
    }
    else {
      moveBackOne();
    }
  }

  //   //floodfill queue
  else { //if none of the surrounding cells are smaller
    push(board[positionX][positionY].cellNum); //push the current position
    while (left_pointer < right_pointer) { //while there are items in the queue
        int current_cell = pop(); //pop the first cell of the queue and set is at the current_cell


  //   //  turnRight();
  // }


}

//determines where the walls are in the current box (in relation to the gird, not the robot ortientation)
void checkWalls() {
  int ls= calcDistance(ltr, le);
  int rs= calcDistance(rtr, re);
  int fs= calcDistance(ftr, fe);
  int bs= calcDistance(btr, be);

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
  board[positionX][positionY].visited = true;
  board[positionX][positionY].nWall = isWalls[0];
  board[positionX][positionY].eWall = isWalls[1];
  board[positionX][positionY].sWall = isWalls[2];
  board[positionX][positionY].wWall = isWalls[3];

}

//determines the minimum neighbor out of the available cells
int findMinNeighbor(int X, int Y) {
  int minimum_neighbor = 100; //initialize to high value to be updated
  int direction;

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
  current_min_neightbor = minimum_neighbor;
  return direction;      
  }




  // for (int i = 0; i <= 3; i++) { //consider all 4 directions
      // if (isWalls[i] == false) { //it is possible to go in direction i

      //   if (i == 0 ) { //if the direction is forward
      //     if (board[positionX+1][positionY].dist < minimum_neighbor) { //if the board in front is less than the minimum
      //       minimum_neighbor = board[positionX+1][positionY].dist; //the new minimum is the board in front
      //       direction = 0; //plan to move forward
      //     }
      //   }

      //   if (i == 1 ) { //if the direction is right
      //     if (board[positionX][positionY+1].dist < minimum_neighbor) { //if the board to the right is less than the minimum
      //       minimum_neighbor = board[positionX][positionY+1].dist; //the new minimum is the board on the right
      //       direction = 1; //plan to move right
      //     }
      //   }

      //   if (i == 2 ) { //if the direction is backwards
      //     if (board[positionX-1][positionY].dist < minimum_neighbor) { //if the board in the back is less than the minimum
      //       minimum_neighbor = board[positionX-1][positionY].dist; //the new minimum is the board in the back
      //       direction = 2; //plan to move backward
      //     }
      //   }

      //   if (i == 3 ) { //if the direction is left
      //     if (board[positionX][positionY-1].dist < minimum_neighbor) { //if the board to the left is less than the minimum
      //       minimum_neighbor = board[positionX][positionY-1].dist; //the new minimum is the board on the left
      //       direction = 3; //plan to move left
      //     }
      //   }
        
      // }
  


  // current_min_neightbor = minimum_neighbor;
    
      //while the left_pointer != right_poiniter: (AKA queue not empty)
          // current_cell = pop first element from queue (and update left pointer)
          // find current_cell's MINIMUM accessible neighbors
          // If current cell’s value is ≤ minimum of its neighbors:
                // set current_cell’s value to minimum +1
                // add accessible neighbors to queue
          //else 
                // continue

  // return direction;
// }

//this is no longer valid:
void center() {
  //check which walls are there (only coonsider walls that are within a range)

  //check backwall
  int ls= calcDistance(ltr, le);
  int rs= calcDistance(rtr, re);
  int fs= calcDistance(ftr, fe);
  int bs= calcDistance(btr, be);
  if (bs <bmax) { //there is a wall within a certain range for the back sensor
      if (bs > bt) {//if the distance is too far
        backward(); //move backwards
        while (bs > bt) { 
            bs= calcDistance(btr, be);
            delay(50);
        }
        stop (); //stop adjustment
      }
      if (bs < bt) {//if the distance is too close
        forward(); //move forwards
        while (bs < bt) { 
            bs= calcDistance(btr, be);
            delay(50);
        }
        stop (); //stop adjustment
      }
  }

  //check front wall
  if (fs < ffmax)  { //there is a wall within a certain range for the front sensor
      if (fs > ft) {//if the distance is too far
        forward(); //move forwards
        while (fs > ft) { 
            fs= calcDistance(ftr, fe);
            delay(50);
        }
        stop (); //stop adjustment
      }
      if (fs < ft) {//if the distance is too close
        backward(); //move forwards
        while (fs < ft) { 
            fs= calcDistance(ftr, fe);
            delay(50);
        }
        stop (); //stop adjustment
      }
  }

  //check left wall
  if (ls < lmax)  { //there is a wall within a certain range for the left sensor
      if (ls > lt) {//if the distance is too far
        left(); //move left
        while (ls > lt) { 
            ls= calcDistance(ltr, le);
            delay(50);
        }
        stop (); //stop adjustment
      }
      if (ls < lt) {//if the distance is too close
        right(); //move forwards
        while (ls < lt) { 
            ls= calcDistance(ltr, le);
            delay(50);
        }
        stop (); //stop adjustment
      }
  }

  //check right wall
  if (rs < rmax)  { //there is a wall within a certain range for the left sensor
      if (rs > rt) {//if the distance is too far
        right(); //move left
        while (rs > rt) { 
            rs= calcDistance(rtr, re);
            delay(50);
        }
        stop (); //stop adjustment
      }
      if (rs < rt) {//if the distance is too close
        left(); //move forwards
        while (rs < rt) { 
            rs= calcDistance(rtr, re);
            delay(50);
        }
        stop (); //stop adjustment
      }
  }

}


//method to move forward one sqare in the maze
void moveForwardOne() {
  //move forward
  forward();
  delay(1540);

  //update position on board
  if (orientation == 0) {
    positionX += 1;
  }
  else if (orientation == 1) {
    positionY +=1;
  }
  else if (orientation == 2) {
    positionX -=1;
  }
  else if (orientation == 3) {
    positionY -=1;
  }



  //use sensors to check in the correct location and adjust accordingly
}
//method to move backward one sqare in the maze
void moveBackOne() {
  //move forward
  backward();
  delay(1540);
  //use sensors to check in the correct location and adjust accordingly

  if (orientation == 0) {
    positionX -= 1;
  }
  else if (orientation == 1) {
    positionY -=1;
  }
  else if (orientation == 2) {
    positionX +=1;
  }
  else if (orientation == 3) {
    positionY +=1;
  }

}

void turnRight() {
  fl->run(FORWARD);
  fr->run(BACKWARD);
  bl->run(FORWARD);
  br->run(BACKWARD);
  delay(1100);

  orientation += 1;
  if (orientation >= 4) {
    orientation = 0;
  }
}

void turnLeft() {
  fl->run(BACKWARD);
  fr->run(FORWARD);
  bl->run(BACKWARD);
  br->run(FORWARD);
  delay(1100);

  orientation -= 1;
  if (orientation < 0) {
    orientation = 4;
  }
}

//method to pop one item from the queue at index left_pointer
int pop() {
  int val = queue[left_pointer]; 
  left_pointer += 1;
  return val;
}
// method to push one item onto the queue with value val and index right_pointer
void push(int cellNum) {
  queue[right_pointer] = cellNum;
  right_pointer += 1;
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

void stop() {
  fl->run(RELEASE);
  fr->run(RELEASE);
  bl->run(RELEASE);
  br->run(RELEASE);
}

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

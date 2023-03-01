// File:          ChessBoardExplore.cpp
// Date:          2023/02/17
// Description:   Traverse the Chess Board and find the location of the King
// Author:        ThatGuy
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#include <iostream>
#include <vector>

#define GRIPPER_MOTOR_MAX_SPEED 0.1
#define TIMESTEP 16

// All the webots classes are defined in the "webots" namespace
using namespace webots;

void pickUpTheBox();
void turnRight();
void turnLeft();
void advanceTile(double s = 1.0);
void advanceTileBack(double s = 1.0);
void travelMaze();
bool checkPiece();
void dropTheBox();
void DFS();
void DFS_1();
void goToExit();

int initialRow = 1;
int initialColumn = 3;
//int chessboard[9][9] = { 0 };
int posCounter[4] = { 0 };
int pos = 0;

bool exitFound = false;

int exitDirectionRow = 0;
int exitDirectionColumn = 0;

std::vector<std::vector<int>> chessboard{};

Robot* robot;

Motor* rightMotor;
Motor* leftMotor;

Motor* gripperLift;

Motor* rightFinger;
Motor* leftFinger;

DistanceSensor* frontDS;
DistanceSensor* topDS;
DistanceSensor* frontIR;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  robot = new Robot();

  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();
  int timeStep = TIMESTEP;

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  rightMotor = robot->getMotor("rightWheel");
  leftMotor = robot->getMotor("leftWheel");

  gripperLift = robot->getMotor("lift motor");

  rightFinger = robot->getMotor("right finger motor");
  leftFinger = robot->getMotor("left finger motor");

  frontDS = robot->getDistanceSensor("frontDS");
  frontDS->enable(timeStep);

  topDS = robot->getDistanceSensor("topDS");
  topDS->enable(timeStep);

  frontIR = robot->getDistanceSensor("frontIR");
  frontIR->enable(timeStep);

  // create the chessboard with extended walls
  for (int i = 0; i < 12; i++) {
      std::vector<int> newVec{};
      for (int j = 0; j < 12; j++) {
          newVec.push_back(1);
      }
      chessboard.push_back(newVec);
  }

  for (int i = 1; i < 11; i++) {
      for (int j = 1; j < 11; j++)
          chessboard[i][j] = 0;
  }

  //advanceTile();
  //DFS_1();

  //std::cout << "DONE DONE AND DONE\n";

  //std::cout << "( " << exitDirectionRow << ", " << exitDirectionColumn << " )\n";

  //if (exitDirectionRow == 10 && exitDirectionColumn == 11)
  //    turnLeft();

  //goToExit();
  travelMaze();
  //goToExit();

  //travelMaze();
  //pickUpTheBox();
  //turnRight();
  //turnLeft();
  //advanceTile();
  //turnRight();
  //turnLeft();

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  //while (robot->step(timeStep) != -1) {

  //  // Read the sensors:
  //  // Enter here functions to read sensor data, like:
  //  //  double val = ds->getValue();
  //    double newFrontDSValue = frontDS->getValue();
  //  // Process sensor data here.
  //    //cout << prevFrontDSValue - newFrontDSValue << "\n";
  //  // Enter here functions to send actuator commands, like:
  //  //  motor->setPosition(10.0);
  //    cout << newFrontDSValue << "\n";

  //    if (newFrontDSValue < 13) {
  //        rightFinger->setPosition(0.06);
  //        leftFinger->setPosition(0.06);
  //        gripperLift->setPosition(0.01);
  //    }

  //    //prevFrontDSValue = newFrontDSValue;
  //};

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

void travelMaze() {
    pickUpTheBox();

    int advanceStraightCount = 0;
    bool kingFound = false;

    while (true) {
        turnRight();

        double newFrontDSValue = frontDS->getValue();

        // Check for obstacle on right
        if (newFrontDSValue > 800) {
            turnLeft();
        }
        else {
            std::cout << "OBSTACLE FOUND\n";
            
            newFrontDSValue = frontDS->getValue();
            std::cout << newFrontDSValue << "\n";

            // Go to the right chess piece
            int advanceRightCount = 0;
            while (newFrontDSValue > 120) {
                advanceTile();
                advanceRightCount++;

                newFrontDSValue = frontDS->getValue();
            }

            std::cout << "CHECKING THE PIECE\n";
            kingFound = checkPiece();
     
            // Come back to the main path
            for (int i = 0; i < advanceRightCount; i++) {
                advanceTileBack();
            }

            turnLeft();
        }

        if (kingFound) {
            advanceTileBack();
            advanceTile(0.6);
            dropTheBox();
            advanceTileBack(0.6);
            advanceStraightCount--;
            break;
        }

        newFrontDSValue = frontDS->getValue();
        if (newFrontDSValue > 100) {
            advanceTile();
            advanceStraightCount++;
        }
        else {
            break;
        }
    }

    // 180 Turn and go back to A7
    turnRight();
    turnRight();
    for (int i = 0; i < advanceStraightCount; i++) {
        advanceTile();
    }

    turnLeft();
    robot->step(TIMESTEP);
    bool canGoRight = frontDS->getValue() > 100;

    if (!kingFound && canGoRight) {

        advanceTile();
        advanceStraightCount = 1;

        while (true) {
            turnLeft();

            double newFrontDSValue = frontDS->getValue();

            // Check for obstacle on right
            if (newFrontDSValue > 950) {
                turnRight();
            }
            else {
                std::cout << "OBSTACLE FOUND\n";

                newFrontDSValue = frontDS->getValue();
                std::cout << newFrontDSValue << "\n";

                // Go to the right chess piece
                int advanceRightCount = 0;
                while (newFrontDSValue > 120) {
                    advanceTile();
                    advanceRightCount++;

                    newFrontDSValue = frontDS->getValue();
                }

                std::cout << "CHECKING THE PIECE\n";
                kingFound = checkPiece();

                // Come back to the main path
                for (int i = 0; i < advanceRightCount; i++) {
                    advanceTileBack();
                }

                turnRight();
            }

            if (kingFound) {
                advanceTileBack();
                advanceTile(0.6);
                dropTheBox();
                advanceTileBack(0.6);
                advanceStraightCount--;
                break;
            }

            newFrontDSValue = frontDS->getValue();
            if (newFrontDSValue > 100) {
                advanceTile();
                advanceStraightCount++;
            }
            else {
                break;
            }
        }

        // 180 Turn and go back to A7
        turnRight();
        turnRight();
        for (int i = 0; i < advanceStraightCount; i++) {
            advanceTile();
        }
    }
}

bool checkPiece() {
    double topDSValue = topDS->getValue();
    std::cout << "TOP DS: " << topDSValue << "\n";

    double frontIRValue = frontIR->getValue();
    std::cout << "FRONT IR: " << frontIRValue << "\n";

    if (topDSValue < 500 && frontIRValue < 500) {
        std::cout << "WHITE KING FOUND\n";

        return true;
    }

    return false;
}

void pickUpTheBox() {
    double elapsedTime = 0;

    rightMotor->setVelocity(1.0);
    leftMotor->setVelocity(1.0);

    gripperLift->setVelocity(0.03);
    rightFinger->setVelocity(0.1);
    leftFinger->setVelocity(0.1);

    rightMotor->setPosition(-19);
    leftMotor->setPosition(-19);

    gripperLift->setPosition(0.10);
    rightFinger->setPosition(0.1);
    leftFinger->setPosition(0.1);

    while (elapsedTime < 1180) {
        robot->step(TIMESTEP);

        double newFrontDSValue = frontDS->getValue();

        //cout << newFrontDSValue << "\n";

        if (newFrontDSValue < 6) {
            rightFinger->setPosition(0.06);
            leftFinger->setPosition(0.06);
            gripperLift->setPosition(0.01);
        }

        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);
}

void turnRight() {
    rightMotor->setPosition(INFINITY);  
    leftMotor->setPosition(INFINITY);
    rightMotor->setVelocity(1.0);
    leftMotor->setVelocity(-1.0);
   
    double elapsedTime = 0;

    while (elapsedTime < 810) {
        robot->step(TIMESTEP);

        //double newFrontDSValue = frontDS->getValue();

        //cout << newFrontDSValue << "\n";

        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);

    std::cout << "TURNED RIGHT\n";
}

void turnLeft() {
    rightMotor->setPosition(INFINITY);  
    leftMotor->setPosition(INFINITY);
    rightMotor->setVelocity(-1.0);
    leftMotor->setVelocity(1.0);
    
    double elapsedTime = 0;

    while (elapsedTime < 810) {
        robot->step(TIMESTEP);
        
        //double newFrontDSValue = frontDS->getValue();

        //cout << newFrontDSValue << "\n";

        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);

    std::cout << "TURNED LEFT\n";
}

void advanceTile(double s) {
    double elapsedTime = 0;

    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);

    rightMotor->setVelocity(-1.0);
    leftMotor->setVelocity(-1.0);

    while (elapsedTime < 1190 * s) {
        robot->step(TIMESTEP);
        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);
}

void advanceTileBack(double s) {
    double elapsedTime = 0;

    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);

    rightMotor->setVelocity(1.0);
    leftMotor->setVelocity(1.0);

    while (elapsedTime < 1190 * s) {
        robot->step(TIMESTEP);
        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);
}

void dropTheBox() {
    gripperLift->setVelocity(0.1);
    rightFinger->setVelocity(0.1);
    leftFinger->setVelocity(0.1);

    gripperLift->setPosition(0.13);
    rightFinger->setPosition(0.1);
    leftFinger->setPosition(0.1);
}

void DFS() {
    double newFrontDSValue = frontDS->getValue();

    if (newFrontDSValue < 100) {
        return;
    }

    advanceTile();
    DFS();
    turnRight();
    DFS();
    turnLeft();
    turnLeft();
    DFS();
    turnRight();
    advanceTileBack();
}

void DFS_1() {

    if (exitFound) return;

    //This is needed to get the frontDS value if this is not called before
    //robot->step(TIMESTEP);
    
    int currentRow = initialRow + posCounter[0] - posCounter[2];
    int currentColumn = initialColumn + posCounter[1] - posCounter[3];

    std::cout << "Current Position: (" << currentRow << ", " << currentColumn << ")\t" << chessboard[currentRow][currentColumn] << "\n";

    double newFrontDSValue = frontDS->getValue();
    std::cout << "frontDS: " << newFrontDSValue << "\n";

    posCounter[pos % 4]++;
    int nextRow = initialRow + posCounter[0] - posCounter[2];
    int nextColumn = initialColumn + posCounter[1] - posCounter[3];

    if ((currentColumn == 10 && currentRow == 10) || chessboard[nextRow][nextColumn] == 1 || newFrontDSValue < 100) {
        if ((currentColumn == 10 && currentRow == 10)) {
            std::cout << "first\n";
            std::cout << "Exit Found\n";
            exitFound = true;

            exitDirectionRow = nextRow;
            exitDirectionColumn = nextColumn;
        }

        else if (chessboard[nextRow][nextRow] == 1) std::cout << "second\n";
        else std::cout << "third\n";

        posCounter[pos % 4]--;

        return;
    }

    chessboard[currentRow][currentColumn] = 1;

    advanceTile();
    DFS_1();
    if (exitFound) return;
    
    turnRight();
    pos++;
    DFS_1();
    if (exitFound) return;
    
    turnLeft();
    turnLeft();
    pos+=2;
    DFS_1();
    if (exitFound) return;
    
    //turnRight();
    //advanceTileBack();

    // Going back
    turnLeft();
    advanceTile();
    turnRight();
    turnRight();
    pos -= 3;
    posCounter[pos % 4]--;
}

void goToExit() {
    // Goes to the exit from (1,3)
    DFS_1();

    std::cout << "DONE DONE AND DONE\n";

    std::cout << "( " << exitDirectionRow << ", " << exitDirectionColumn << " )\n";

    if (exitDirectionRow == 10 && exitDirectionColumn == 11)
        turnLeft();
}
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

#define GRIPPER_MOTOR_MAX_SPEED 0.1
#define TIMESTEP 16

// All the webots classes are defined in the "webots" namespace
using namespace webots;

void pickUpTheBox();
void turnRight();
void turnLeft();
void advanceTile();
void advanceTileBack();
void travelMaze();

Robot* robot;

Motor* rightMotor;
Motor* leftMotor;

Motor* gripperLift;

Motor* rightFinger;
Motor* leftFinger;

DistanceSensor* frontDS;

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


  travelMaze();
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

    while (true) {
        turnRight();

        double newFrontDSValue = frontDS->getValue();

        // Check for obstacle on right
        if (newFrontDSValue > 790) {
            turnLeft();
        }
        else {
            std::cout << "OBSTACLE FOUND\n";
            
            newFrontDSValue = frontDS->getValue();
            std::cout << newFrontDSValue << "\n";

            // Go to the right chess piece
            int advanceRightCount = 0;
            while (newFrontDSValue > 100) {
                advanceTile();
                advanceRightCount++;

                newFrontDSValue = frontDS->getValue();
            }

            std::cout << "CHECKING THE PIECE\n";
     
            // Come back to the main path
            for (int i = 0; i < advanceRightCount; i++) {
                advanceTileBack();
            }

            turnLeft();
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
    advanceTile();

    while (true) {
        turnLeft();

        double newFrontDSValue = frontDS->getValue();

        // Check for obstacle on right
        if (newFrontDSValue > 790) {
            turnRight();
        }
        else {
            std::cout << "OBSTACLE FOUND\n";

            newFrontDSValue = frontDS->getValue();
            std::cout << newFrontDSValue << "\n";

            // Go to the right chess piece
            int advanceRightCount = 0;
            while (newFrontDSValue > 100) {
                advanceTile();
                advanceRightCount++;

                newFrontDSValue = frontDS->getValue();
            }

            std::cout << "CHECKING THE PIECE\n";

            // Come back to the main path
            for (int i = 0; i < advanceRightCount; i++) {
                advanceTileBack();
            }

            turnRight();
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

void pickUpTheBox() {
    double elapsedTime = 0;

    rightMotor->setVelocity(1.0);
    leftMotor->setVelocity(1.0);

    gripperLift->setVelocity(0.1);
    rightFinger->setVelocity(0.1);
    leftFinger->setVelocity(0.1);

    rightMotor->setPosition(-19);
    leftMotor->setPosition(-19);

    gripperLift->setPosition(0.13);
    rightFinger->setPosition(0.1);
    leftFinger->setPosition(0.1);

    while (elapsedTime < 1180) {
        robot->step(TIMESTEP);

        double newFrontDSValue = frontDS->getValue();

        //cout << newFrontDSValue << "\n";

        if (newFrontDSValue < 13) {
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

void advanceTile() {
    double elapsedTime = 0;

    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);

    rightMotor->setVelocity(-1.0);
    leftMotor->setVelocity(-1.0);

    while (elapsedTime < 1190) {
        robot->step(TIMESTEP);
        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);
}

void advanceTileBack() {
    double elapsedTime = 0;

    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);

    rightMotor->setVelocity(1.0);
    leftMotor->setVelocity(1.0);

    while (elapsedTime < 1190) {
        robot->step(TIMESTEP);
        elapsedTime++;
    }

    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);
}



void lift(Motor* lift, double position) {
    lift->setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    lift->setPosition(position);
}

void moveFingers(Motor* leftFinger, Motor* rightFinger, double position) {
    leftFinger->setVelocity(GRIPPER_MOTOR_MAX_SPEED);
    rightFinger->setVelocity(GRIPPER_MOTOR_MAX_SPEED);

    leftFinger->setPosition(position);
    rightFinger->setPosition(position);
}

void moveForwards(Motor* leftMotor, Motor* rightMotor, double distance) {
    leftMotor->setVelocity(1.0);
    rightMotor->setVelocity(1.0);

    leftMotor->setPosition(distance);
    rightMotor->setPosition(distance);
}


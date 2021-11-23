#include <stdio.h>
#include <string.h>
#include <cmath>

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
//#include <webots/led.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

#define MAX_SPEED 6.28
#define timeStep 64

// Functions
double dist_to_goal(double *pos);
void doAlign(double *robotPosition, double *heading);
bool aligned(double *robotPosition, double *heading);
//////////////////////////////////////

  // new robot instance
  Robot *robot = new Robot();
  
  // GPS instance
  GPS *gps, *gpsT;
    
  
  // Distance sensors
  DistanceSensor *sensors[8];
    
  // get a handler to the motors and set target position to infinity (speed control)
  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  
  // Goal location
  double Goal[2] = {0.6, -0.7};

int main(int argc, char **argv) {
  
  // GPS locations
  gps = robot -> getGPS("gps");
  gps -> enable(timeStep);
  
  gpsT = robot -> getGPS("gpsT");
  gpsT -> enable(timeStep);
  
  // set and enable distance sensors
  for(int i=0; i<8; i++){
    char deviceName[4];
    sprintf(deviceName, "ps%d", i);
    sensors[i] = robot -> getDistanceSensor(deviceName);
    sensors[i] -> enable(timeStep);
  }
  
  // set up motors positions
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0); 

  bool CW = false, CCW = false;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
  double robotPosition[2] = {gps->getValues()[0], gps->getValues()[2]};
  double heading[2] = {gpsT->getValues()[0], gpsT->getValues()[2]};
  double ps[8];
  double perpendicular = 100.0;
  
  for(int i=0; i<8; i++)
    ps[i] = sensors[i] -> getValue();
    
  cout << "Distance to Goal" << endl;
  cout << dist_to_goal(robotPosition) << endl;
  cout << "#########################" << endl;
  
  if(dist_to_goal(robotPosition) < 0.01){
    //Goal reached
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);     
  }
  else if(ps[0] > 80.0 and ps[0] > ps[7]){
    CCW = true;
  }
  else if(ps[7] > 80.0){
    CW = true;
  }
  else if(CCW){
    if(ps[2] > perpendicular)
      CCW = false;
    else{
      leftMotor->setVelocity(-0.1 * MAX_SPEED);
      rightMotor->setVelocity(0.1 * MAX_SPEED); 
    }
  }
  else if(CW){
    if(ps[3] > perpendicular)
      CW = false;
    else{
      leftMotor->setVelocity(0.1 * MAX_SPEED);
      rightMotor->setVelocity(-0.1 * MAX_SPEED); 
    }
  }
  else if(ps[2] > perpendicular or ps[5] > perpendicular){
    leftMotor->setVelocity(0.1 * MAX_SPEED);
    rightMotor->setVelocity(0.1 * MAX_SPEED);
  }
  else{
    if(!aligned(robotPosition, heading)){
      doAlign(robotPosition, heading);
    }
    else{
      leftMotor->setVelocity(0.2 * MAX_SPEED);
      rightMotor->setVelocity(0.2 * MAX_SPEED); 
    }
  }
  
  
  
  if(CCW){
    if(ps[2] > perpendicular)
      CCW = false;
    else{
      leftMotor->setVelocity(-0.1 * MAX_SPEED);
      rightMotor->setVelocity(0.1 * MAX_SPEED); 
    }
  }
  if(CW){
    if(ps[5] > perpendicular)
      CW = false;
    else{
      leftMotor->setVelocity(0.1 * MAX_SPEED);
      rightMotor->setVelocity(-0.1 * MAX_SPEED); 
    }
  }
  
  };

  delete robot;
  return 0;
}


double dist_to_goal(double *pos){
  return sqrt((pos[0] - Goal[0])*(pos[0] - Goal[0]) + (pos[1] - Goal[1])*(pos[1] - Goal[1]));
  
}

bool aligned(double *robotPosition, double *heading){
  double epsilon = 0.001;
  double angle1 = atan2(Goal[1] - robotPosition[1],Goal[0] - robotPosition[0]);
  double angle2 = atan2(Goal[1] - heading[1],Goal[0] - heading[0]);
  
  double dist = dist_to_goal(robotPosition);
  double distH = dist_to_goal(heading);
  
  if(dist < 0.75) epsilon = 0.01;  
  if(dist < 0.16) epsilon = 0.1; 
  if(dist < 0.1) return true;
  
  if(fabs(angle1 - angle2) < epsilon and distH < dist)
    return true;
  else
    return false;

}

void doAlign(double *robotPosition, double *heading){
  double angle1 = 10.0*atan2(Goal[1] - robotPosition[1],Goal[0] - robotPosition[0]);
  double angle2 = 10.0*atan2(Goal[1] - heading[1],Goal[0] - heading[0]);

  if(angle1 - angle2 >= 0.0){
    leftMotor->setVelocity(-0.1 * MAX_SPEED);
    rightMotor->setVelocity(0.1 * MAX_SPEED);
  }
  else{
    leftMotor->setVelocity(0.1 * MAX_SPEED);
    rightMotor->setVelocity(-0.1 * MAX_SPEED); 
  }
  
}
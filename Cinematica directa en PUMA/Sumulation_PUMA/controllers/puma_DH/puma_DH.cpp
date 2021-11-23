#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Brake.hpp>

#include <vector>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

typedef vector<vector<double> > Matrix;
#define PI 3.14159265358979323846
//#define timeStep 64

Matrix DH(double,double,double,double); // Get 4x4 matrix using DH convention
Matrix AXB(Matrix,Matrix);             // Function to multiply two matrices

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  
  // Joint motors
  Motor *j1 = robot -> getMotor("joint1");
  Motor *j2 = robot -> getMotor("joint2");
  Motor *j3 = robot -> getMotor("joint3");
  Motor *j4 = robot -> getMotor("joint4");
  Motor *j5 = robot -> getMotor("joint5");
  Motor *j6 = robot -> getMotor("joint6");
  
  j1 -> setPosition(INFINITY);
  j2 -> setPosition(INFINITY);
  j3 -> setPosition(INFINITY);
  j4 -> setPosition(INFINITY);
  j5 -> setPosition(INFINITY);
  j6 -> setPosition(INFINITY);
  
  // Initial velocities
  j1 -> setVelocity(0.0);
  j2 -> setVelocity(0.0);
  j3 -> setVelocity(0.0);
  j4 -> setVelocity(0.0);
  j5 -> setVelocity(0.0);
  j6 -> setVelocity(0.0);
  
  
  // Gripper motors
  Motor *g1 = robot -> getMotor("joint_GripperL");
  Motor *g2 = robot -> getMotor("joint_GripperR");
  
  //g1 -> setPosition(INFINITY);
  //g2 -> setPosition(INFINITY);
  g1 -> setVelocity(0.0); //0.005
  g2 -> setVelocity(0.0); //0.005
  
  // GPS gripper position
  GPS *gps;
  gps = robot -> getGPS("gps");
  gps -> enable(timeStep);

  // Keyboard
  Keyboard kb;
  kb.enable(timeStep);
  
  int j = 1; // joint selection
  double vel = 0.0; // velocity selection
  
  // DH parameters
  double a[6] = {0.0, 
                 0.4318, 
                 0.0, 
                 0.0, 
                 0.0, 
                 0.0};
  double alpha [6] = {PI/2.0,
                      0.0,
                      -PI/2.0,
                      PI/2.0,
                      -PI/2.0,
                      0.0};
  double d[6] = {0.6604,
                 -0.2286,
                 0.072,
                 0.4318,
                 0.0,
                 0.17};
  double q[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; // current joint angles
  
  
  // DH transformation matrices
  Matrix A1, A2, A3, A4, A5, A6;
  Matrix T01, T02, T03, T04, T05, T06;
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
  
    int key = kb.getKey();
    if(key == 49)
      j = 1;
    else if(key == 50)
      j = 2;
    else if(key == 51)
      j = 3;
    else if(key == 52)
      j = 4;
    else if(key == 53)
      j = 5;
    else if(key == 54)
      j = 6;
    else if(key == 71)
      j = 7;
    else if(key == 315)
      vel = 0.2;
    else if(key == 317)
      vel = -0.2;
    else
      vel = 0.0;
    
    if(j == 7)
      cout << "Gripper";
    else
      cout << "Joint " << j; 
    cout << " selected" << endl << endl;
    
    switch(j){
      case 1:
        j1 -> setVelocity(vel);
        q[0] += vel * 1.0/timeStep;
        if(q[0] < -2.79) q[0] = -2.79;
        if(q[0] > 2.79) q[0] = 2.79;
        break;
      case 2:
        j2 -> setVelocity(vel);
        q[1] += vel * 1.0/timeStep;
        break;
      case 3:
        j3 -> setVelocity(vel);
        q[2] += vel * 1.0/timeStep;
        break;
      case 4:
        j4 -> setVelocity(vel);
        q[3] += vel * 1.0/timeStep;
        if(q[3] < -1.92) q[3] = -1.92;
        if(q[3] > 2.79) q[3] = 2.79;
        break;
      case 5:
        j5 -> setVelocity(vel);
        q[4] += vel * 1.0/timeStep;
        if(q[4] < -1.75) q[4] = -1.75;
        if(q[4] > 1.75) q[4] = 1.75;
        break;
      case 6:
        j6 -> setVelocity(vel);
        q[5] += vel * 1.0/timeStep;
        break;
      default:
        if(vel > 0.0){
          g1 -> setPosition(-0.004);
          g2 -> setPosition(-0.004);
          g1 -> setVelocity(0.05); 
          g2 -> setVelocity(0.05);
        }
        else if(vel < 0.0){
          g1 -> setPosition(0.01);
          g2 -> setPosition(0.01);
          g1 -> setVelocity(0.05);
          g2 -> setVelocity(0.05);
        }
        break;
    }
    
    
    // DH Transformation matrices
    A1 = DH(a[0], alpha[0], d[0], q[0]);
    A2 = DH(a[1], alpha[1], d[1], -q[1]);
    A3 = DH(a[2], alpha[2], d[2], -q[2]);
    A4 = DH(a[3], alpha[3], d[3], q[3]);
    A5 = DH(a[4], alpha[4], d[4], -q[4]);
    A6 = DH(a[5], alpha[5], d[5], q[5]);
    
    T01 = A1;
    T02 = AXB(T01,A2);
    T03 = AXB(T02,A3);
    T04 = AXB(T03,A4);
    T05 = AXB(T04,A5);
    T06 = AXB(T05,A6);
    
    
    cout << "Gripper position by GPS" << endl;
    cout << "X: " << gps -> getValues()[0] << endl;
    cout << "Y: " << gps -> getValues()[2] << endl;
    cout << "Z: " << gps -> getValues()[1] << endl<<endl;
    
    cout << "Gripper position by D-H" << endl;
    cout << "X: " << T06[0][3] << endl;
    cout << "Y: " << -T06[1][3] << endl;
    cout << "Z: " << T06[2][3] << endl;
    cout << "#####################" << endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}


Matrix DH(double a,double alpha,double d,double theta){
/*
*   Inputs
*       - double a,double alpha,double d,double theta
*
*   Outputs:
*       - Matrix :4x4 DH matrix
*/
    Matrix A;
    vector<double> row;

    for(int i=0; i<4; i++)
        row.push_back(0);

    for(int i=0; i<4; i++)
        A.push_back(row);

    A[0][0] = cos(theta);
    A[0][1] = -sin(theta) * cos(alpha);
    A[0][2] = sin(theta) * sin(alpha);
    A[0][3] = a * cos(theta);
    A[1][0] = sin(theta);
    A[1][1] = cos(theta) * cos(alpha);
    A[1][2] = -cos(theta) * sin(alpha);
    A[1][3] = a * sin(theta);
    A[2][1] = sin(alpha);
    A[2][2] = cos(alpha);
    A[2][3] = d;
    A[3][3] = 1;

    return A;
}


Matrix AXB(Matrix A,Matrix B){
/*
*   Inputs:
*       - Matrix A: Matrix A
*       - Matrix B: Matrix B
*
*    Outputs:
*       - Matrix : multiplication A*B
*/
    Matrix C;
    if(A[0].size() == B.size()){
        vector<double> row;

        for(int i=0; i<(int)A.size(); i++){
            for(int j=0; j<(int)B[0].size(); j++){
                double val = 0;
                for(int k=0; k<(int)A[0].size(); k++)
                    val += A[i][k] * B[k][j];
                row.push_back(val);
            }
            C.push_back(row);
            row.clear();
        }
    }
    else{
        cout << "Error: A and B are NOT compatible\nMultiplication can not be done\n";
    }

    return C;
}



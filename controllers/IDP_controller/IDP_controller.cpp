// File:          IDP_controller.cpp
#include <webots/Robot.hpp>

// Added a new include file
#include <webots/Motor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <cmath>

#define TIME_STEP 64
#define MAX_SPEED 10
#define PI 3.14159265
// All the webots classes are defined in the "webots" namespace
using namespace webots;

Robot *robot = new Robot();

//DistanceSensor *ds[4];
//char dsNames[4][40] = {"us_right","us_left","ir_left","ir_right"};

/*LightSensor *light_sensor = robot->getLightSensor("TEPT4400");
light_sensor->enable(TIME_STEP);
*/

// get the motor devices
Motor *leftMotor = robot->getMotor("Wheel_L");
Motor *rightMotor = robot->getMotor("Wheel_R");

// get the arm motor devices
Motor *leftArmMotor = robot->getMotor("Arm_L");
Motor *rightArmMotor = robot->getMotor("Arm_R");

void move_forwards() {
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(0.5 * MAX_SPEED);
   rightMotor->setVelocity(0.5 * MAX_SPEED);
}
void move_backwards() {
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(-0.5 * MAX_SPEED);
   rightMotor->setVelocity(-0.5 * MAX_SPEED);
}

void move_position(double x){
   // set the target position of the motors
   leftMotor->setPosition(x);
   rightMotor->setPosition(x);
}

void rotate_CW() {
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(0.5 * MAX_SPEED);
   rightMotor->setVelocity(-0.5 * MAX_SPEED);
}
void rotate_ACW() {
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(-0.5*MAX_SPEED);
   rightMotor->setVelocity(0.5*MAX_SPEED);
}
void open_arms() {
   // set the target position of the motors
   leftArmMotor->setPosition(0.5);
   rightArmMotor->setPosition(-0.5);
}
void close_arms() {
   // set the target position of the motors
   leftMotor->setPosition(0.0);
   rightMotor->setPosition(0.0);
}
/*
void scanOnSpot(){
    for (int i = 0; i < 4 ; i++){
    double dsValues[4];
    dsValues[i] = ds[i]->getValue();
    rotate_ACW();
    }
}
*/

/* //origional function that is wrong:
double get_bearing_in_degrees(const double *north) {
  //const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}
*/
double get_bearing_in_degrees(const double *north) { //fixed to match coordinate system
  //const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad) / M_PI * 180.0; // took away the -pi/2
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

void rotate_theta(double theta, double initial_bearing) { //function to be used in scan_on_spot
  double angle_rotated = 0.0;
  // set the target position, velocity of the motors
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.5 * MAX_SPEED);
  rightMotor->setVelocity(-0.5 * MAX_SPEED);
  while (robot->step(TIME_STEP) != -1){
  Compass *compass = robot->getCompass("compass");
  compass->enable(TIME_STEP);
  const double *north = compass->getValues();
  double bearing = get_bearing_in_degrees(north);
      if ((bearing - initial_bearing) >= 0.0) {
      angle_rotated = bearing - initial_bearing;
    }
    if ((bearing - initial_bearing) < 0.0) {
      angle_rotated = bearing + (360.0 - initial_bearing);
      } 
     //std::cout << angle_rotated << std::endl;
     //std::cout << theta << std::endl;
    if (angle_rotated > theta) {
       leftMotor->setVelocity(0.0 * MAX_SPEED);
       rightMotor->setVelocity(0.0 * MAX_SPEED);
       break;
    }
 } 
}
   
void rotate_until_bearing(double target_bearing, double initial_bearing) {
  double angle_rotated = 0.0;
  // set the target position, velocity of the motors
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  
  if (target_bearing > initial_bearing) {
    leftMotor->setVelocity(0.1 * MAX_SPEED);
    rightMotor->setVelocity(-0.1 * MAX_SPEED);
    while (robot->step(TIME_STEP) != -1){
      Compass *compass = robot->getCompass("compass");
      compass->enable(TIME_STEP);
      const double *north = compass->getValues();
      double bearing = get_bearing_in_degrees(north);
      //std::cout << bearing << std::endl;
      if (bearing > target_bearing) {
        leftMotor->setVelocity(0.0 * MAX_SPEED);
        rightMotor->setVelocity(0.0 * MAX_SPEED);
        break;
      }
    }
  }
    if (target_bearing < initial_bearing) {
    leftMotor->setVelocity(-0.1 * MAX_SPEED);
    rightMotor->setVelocity(0.1 * MAX_SPEED);
    while (robot->step(TIME_STEP) != -1){
      Compass *compass = robot->getCompass("compass");
      compass->enable(TIME_STEP);
      const double *north = compass->getValues();
      double bearing = get_bearing_in_degrees(north);
      //std::cout << bearing << std::endl;
      if (bearing < target_bearing) {
        leftMotor->setVelocity(0.0 * MAX_SPEED);
        rightMotor->setVelocity(0.0 * MAX_SPEED);
        break;
      }
    }
    }
  else {
  return;
  }
}


void return_to_initial_position(double current_position[3], double initial_position[3]) {
  double target_bearing = 0.0;
  if (current_position[2] < initial_position[2]) {
    target_bearing = 90.0 + (std::atan((current_position[0] - initial_position[0]) / (current_position[2] - initial_position[2])) * 180.0 / PI);
  std::cout << "condition 1" << std::endl;
  }
  if (current_position[2] > initial_position[2]) {
    target_bearing = 270.0 - (std::atan((current_position[0] - initial_position[0]) / (current_position[2] - initial_position[2])) * 180.0 / PI);
  std::cout << "condition 2" << std::endl;
  }
  if (current_position[2] == initial_position[2] && current_position[0] > initial_position[0]) {
    target_bearing = 180.0;
  std::cout << "condition 3" << std::endl;
    }
  if (current_position[2] == initial_position[2] && current_position[0] < initial_position[0]) {
    target_bearing = 0.0;
  std::cout << "condition 4" << std::endl;
    }
  if (current_position[2] == initial_position[2] && current_position[0] == initial_position[0]) {
    // need to exit function
    std::cout << "condition 5" << std::endl;
    }
  std::cout << target_bearing << std::endl;
  double distance_to_travel = std::sqrt(std::pow((current_position[0] - initial_position[0]), 2) + std::pow((current_position[2] - initial_position[2]), 2));
  //double wheel_angle_to_rotate = distance_to_travel / 0.02;
  double initial_bearing = get_bearing_in_degrees(current_position);
  rotate_until_bearing(target_bearing, initial_bearing);
  double distance_travelled = 0.0;
  
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  if (distance_travelled < distance_to_travel) {
    leftMotor->setVelocity(0.5*MAX_SPEED);
    rightMotor->setVelocity(0.5*MAX_SPEED);
    while (robot->step(TIME_STEP) != -1){
      GPS *gps = robot->getGPS("gps");
      gps->enable(TIME_STEP);
      const double *position_while_moving = gps->getValues();
      distance_travelled = std::sqrt(std::pow((current_position[0] - position_while_moving[0]), 2) + std::pow((current_position[2] - position_while_moving[2]), 2));
      //std::cout << bearing << std::endl;
      if (distance_travelled > distance_to_travel) {
        leftMotor->setVelocity(0.0 * MAX_SPEED);
        rightMotor->setVelocity(0.0 * MAX_SPEED);
        break;
      }
    }
  }
 return;
}

  

int main(int argc, char **argv) {
  
//Initialising GPS
GPS *gps = robot->getGPS("gps");
gps->enable(TIME_STEP);

//Initialising compass
Compass *compass = robot->getCompass("compass");
compass->enable(TIME_STEP);

  // position and orientation pointers
  const double *position = gps->getValues();
  //const double *north = compass->getValues();

//Retrieving device tags and enabling with refresh time step
 /* for(int i = 0; i < 4; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP); //TIME_STEP defines rate at which sensor is refreshed
  }
  */
  
  double initial_position_cache[3]; //initiate initial position
  int i = 0; //initiate time step counter
  bool has_rotated = false;
  bool has_returned = false;      
 while (robot->step(TIME_STEP) != -1){
  //const double *initial_position;
  
  if (i == 0){
    const double *north = compass->getValues();
    //initial_position = gps->getValues();
    initial_position_cache[0] = gps->getValues()[0];
    initial_position_cache[1] = gps->getValues()[1];
    initial_position_cache[2] = gps->getValues()[2];
    //std::cout <<  north[0] << ", "<< north[1] << ", "<< north[2] << std::endl;
    double initial_bearing = get_bearing_in_degrees(north);
    std::cout << initial_position_cache[0] <<',' << initial_position_cache[1] << ',' << initial_position_cache[2] << std::endl;
    rotate_until_bearing(160.0, initial_bearing); // input target bearing and current bearing
    has_rotated = true;
  }
  //std::cout << "start" << initial_position[0] <<',' << initial_position[1] << ',' << initial_position[2] << std::endl;
  //double initial_bearing = 0.0;
  if (robot->step(TIME_STEP) == -1) {
    break;
    }
  //std::cout << has_rotated << std::endl;
  //std::cout << "_____" << std::endl;
  //std::cout << has_returned << std::endl;
  if (has_rotated == true && has_returned == false) {
    double current_position_cache[3];
    current_position_cache[0] = gps->getValues()[0];
    current_position_cache[1] = gps->getValues()[1];
    current_position_cache[2] = gps->getValues()[2];
    const double *current_position = gps->getValues();
    std::cout << initial_position_cache[0] <<',' << initial_position_cache[1] << ',' << initial_position_cache[2] << std::endl;
    std::cout << current_position_cache[0] <<',' << current_position_cache[1] << ',' << current_position_cache[2] << std::endl;
    return_to_initial_position(current_position_cache, initial_position_cache);
    has_returned = true;
    //rotate_theta(185.0, initial_bearing);
   }
  
    // Main (algorithmic loop)
    //std::cout << bearing << std::endl;
    //std::cout <<  position[0] << ", "<< position[1] << ", "<< position[2] << std::endl;
    //std::cout <<  north[0] << ", "<< north[1] << ", "<< north[2] << std::endl;
    //std::cout <<  initial_position[0] << ", "<< initial_position[1] << ", "<< initial_position[2] << std::endl;
    //std::cout << i << std::endl;
    
   i+=1;
   
 }
    
 delete robot;
 return 0;
 }


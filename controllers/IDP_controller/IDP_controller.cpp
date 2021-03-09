#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/Emitter.hpp>

#define TIME_STEP 32
#define MAX_SPEED 10

// All the webots classes are defined in the "webots" namespace
using namespace webots;

//Creating new robot instance
Robot *robot = new Robot();

//Get devices
Motor *leftMotor = robot->getMotor("Wheel_L");
Motor *rightMotor = robot->getMotor("Wheel_R");
Motor *leftArmMotor = robot->getMotor("Arm_L");
Motor *rightArmMotor = robot->getMotor("Arm_R");
GPS *gps = robot->getGPS("gps");
Compass *compass = robot->getCompass("compass");
/*LightSensor *lightSensor = robot->getLightSensor("lightsensor");
lightSensor->enable(TIME_STEP);
*/

//Initialsing US and IR sensors
DistanceSensor *ds[3];
char dsNames[3][20] = {"us_right","us_left","ir"};

//Initiate vector to store sensor readings
std::vector<std::vector<double>> sensorValueScan;

void rotate(double theta){

// set the target position of the motors
  leftMotor->setPosition(theta);
  rightMotor->setPosition(-theta);
  
  // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(0.5 * MAX_SPEED);
   rightMotor->setVelocity(0.5 * MAX_SPEED);
}

void move_forwards() {

   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

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
   leftMotor->setVelocity(-0.3*MAX_SPEED);
   rightMotor->setVelocity(0.3*MAX_SPEED);
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

//Function to get sensor values for one time step, in distance units
std::vector<double> getSensorValues(double (*get_bearing_in_degrees)()){

    //Creating vector to store one row of sensor values for one time step
    std::vector<double> sensorValues(4);
    
    //Input distance sensor values as distance (involves dividing through by 57)
    for (int i = 0; i < 2; i++){
      sensorValues[i]=(ds[i]->getValue())/5700;
    }
    
    //Get IR sensor lookup table
    const double *lookUpTable = ds[2]->getLookupTable();
    int lookUpTableSize = ds[2]->getLookupTableSize();
    
    
    //Input IR sensor values as distance (involves interpolation)
 
    double distanceActual = 0;
    double lookUpValue=ds[2]->getValue();
    for(int j = 0; j < lookUpTableSize; j++){
      double voltageRef = lookUpTable[3*j+1];
      double distanceRef = lookUpTable[3*j];
      if(lookUpValue > voltageRef){
        distanceActual = lookUpTable[3*(j-1)]+(distanceRef-lookUpTable[3*(j-1)])*(lookUpValue-voltageRef);
        break;
      }
    }
    sensorValues[2] = distanceActual;
    
    //Last value of size 5 vector is bearing
    double bearing = get_bearing_in_degrees();
    sensorValues[3] = bearing;
    return sensorValues;
}

std::vector<double> getRawSensorValues(double (*get_bearing_in_degrees)()){

    //Creating vector to store one row of sensor values for one time step
    std::vector<double> sensorValues(4);
    
    //Input distance sensor values as distance (involves dividing through by 57)
    for (int i = 0; i < 3; i++){
      sensorValues[i]=ds[i]->getValue();
    }
    return sensorValues;
}
      
double get_bearing_in_degrees() {
    const double *north = compass->getValues();
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0){
      bearing = bearing + 360.0;
    }
    return bearing;
}

/*======================================================================
SCANNING FUNCTIONS
========================================================================*/

//Function to rotate by a fixed angle
void rotate_theta(double theta, double initial_bearing, bool &fin) { 

    double angle_rotated = 0.0;
    // set the target position, velocity of the motors
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.5 * MAX_SPEED);
    rightMotor->setVelocity(-0.5 * MAX_SPEED);
    
    while (robot->step(TIME_STEP) != -1){
        double bearing = get_bearing_in_degrees();
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
       fin = true;
       break;
    }
  } 
}

//Version of rotate_theta that also scans while doing so
void doScan(double theta, double initial_bearing, bool &fin) { 

    double angle_rotated = 0.0;
    // set the target position, velocity of the motors
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.5 * MAX_SPEED);
    rightMotor->setVelocity(-0.5 * MAX_SPEED);
    
    while (robot->step(TIME_STEP) != -1){
        double bearing = get_bearing_in_degrees();
        std::vector<double> Values = getSensorValues(get_bearing_in_degrees);
        sensorValueScan.push_back(Values);
        if ((bearing - initial_bearing) >= 0) {
        angle_rotated = bearing - initial_bearing;
        }
    
        if ((bearing - initial_bearing) < 0) {
          angle_rotated = bearing + (360 - initial_bearing);
        } 
        //std::cout << angle_rotated << std::endl;
        //std::cout << theta << std::endl;
    
        if (angle_rotated > theta) {
           leftMotor->setVelocity(0);
           rightMotor->setVelocity(0);
           fin = true;
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
        double bearing = get_bearing_in_degrees();
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
        double bearing = get_bearing_in_degrees();
        //std::cout << bearing << std::endl;
        
        if (bearing < target_bearing) {
          leftMotor->setVelocity(0.0 * MAX_SPEED);
          rightMotor->setVelocity(0.0 * MAX_SPEED);
          break;
          
        }
      }
    }
}

/*================================================================
BLOCK DETECTION 
==================================================================*/

//Takes uncleaned 2d vector and filters for blocks, returning array of their bearings
std::vector<double> getBlockBearings(){

      std::vector<double> blockBearings;
      
      //Calculating average returned IR sensor value
      int lookUpTableSize = ds[2]->getLookupTableSize();
      int numCounter = 0;
      double sumDistance = 0;
      for(int j = 0; j<lookUpTableSize;j++){
        sumDistance += sensorValueScan[j][2];
        if(sensorValueScan[j][2]!=0){
          numCounter++;
        }
      }
      double avgDistance = sumDistance/numCounter;
      
      
      //Alpha is a shortcode for the distance value on the jth row, 2nd column of sensorValueScam
      double alpha;
      //Conditional logic to extract bearings and then convert to GPS location
      for(int j = 2; j<lookUpTableSize;j++){
        alpha = sensorValueScan[j][2];
        /*Conditions for blocks to be picked out:
        1. Large jump between previous value
        2. Large jump between value two time steps previously
        3. Large difference between distance value recorded and average distance value
        calculated above
        */
        if( (alpha - sensorValueScan[j-1][2]) > 0.05 || 
        (avgDistance - alpha) > 0.3){
          blockBearings.push_back(sensorValueScan[j][3]);
        }
      }
      
      return blockBearings;
}
//Clean sensorValueScan to get block GPS coordinates. A copy of getBlockBearings with GPS added
std::vector<std::vector<double>> getBlockGPS(){

      std::vector<double> blockBearings;
      std::vector<double> blockDistances;
      std::vector<std::vector<double>> blockGPS;
      
      //Calculating average returned IR sensor value
      int lookUpTableSize = ds[2]->getLookupTableSize();
      int numCounter = 0;
      double sumDistance = 0;
      for(int j = 0; j<sensorValueScan.size();j++){
        sumDistance += sensorValueScan[j][2];
        if(sensorValueScan[j][2]!=0){
          numCounter++;
        }
      }
      double avgDistance = sumDistance/numCounter;
      
      //Alpha is a shortcode for the distance value on the jth row, 2nd column of sensorValueScam
      double alpha;
      //Conditional logic to extract bearings and then convert to GPS location
      for(int j = 1; j<sensorValueScan.size();j++){
        alpha = sensorValueScan[j][2];
        /*Conditions for blocks to be picked out:
        1. Large jump between previous value
        2. Large difference between distance value recorded and average distance value
        calculated above
        */
        if( (alpha-sensorValueScan[j-1][2]) > 0.05 || 
        (avgDistance - alpha) > 0.3){
          blockBearings.push_back(sensorValueScan[j][3]);
          blockDistances.push_back(alpha);
        }
      }
      
      //Turning bearings and distances into new GPS readings
      const double *position = gps->getValues();
      for (auto i : blockBearings){
        blockGPS[i][0] = position[0]+(blockDistances[i]+0.12)*cos(blockBearings[i]*M_PI/180);
        blockGPS[i][1] = position[1];
        blockGPS[i][2] = position[2]+(blockDistances[i]+0.12)*sin(blockBearings[i]*M_PI/180);
      }
      return blockGPS;
}

/*================================================================
MAIN FUNCTION
==================================================================*/
 
int main(int argc, char **argv) {
    
    //Enabling navigation
    gps->enable(TIME_STEP);
    compass->enable(TIME_STEP);
  
    //Retrieving device tags and enabling with refresh time step
    for(int i = 0; i < 3; i++){
      ds[i] = robot->getDistanceSensor(dsNames[i]);
      ds[i]->enable(TIME_STEP); //TIME_STEP defines rate at which sensor is refreshed
    }
    
   int i = 0;
   
   //fin 1 tells you if the bot has finished turning
   //It is set to 1 inside 'rotate_theta'
   //Reset this whenever you are ready to start scanning again
   bool fin1 = false;
   
   //fin2 tells you if the bot has finished the going to a block and picking it up
   bool fin2 = false;
   
   //MAIN WHILE LOOP WHERE STUFF HAPPENS
   while (robot->step(TIME_STEP) != -1){
       
       //Code snippet to get sensor values as bot rotates in circle
       /*std::vector<double> Values = getRawSensorValues(get_bearing_in_degrees);
       sensorValueScan.push_back(Values);
       rotate_ACW();*/
       
       //Initial scan
       if(fin1==false){
          double currentBearing = get_bearing_in_degrees();
          doScan(355, currentBearing, fin1);
       }
      
      if(i==0 && fin2==false){
         std::vector<std::vector<double>> blockGPS = getBlockGPS();
         for(auto& row:blockGPS){
           for(auto& col:row){
              std::cout << col << ",";
           }
           std::cout << "\n";
         }
         fin2 == true;
      }
      
      //Collecting one block
      /*if(fin1==true && fin2==false){
        std::vector<double> blockBearings = getBlockBearings();
        rotate_until_bearing(blockBearings[0],get_bearing_in_degrees());
        open_arms();
        std::vector<std::vector<double>> blockGPS = getBlockGPS();
        
        do{
          const double *currentLocation = gps->getValues();
          double delta_x = fabs(blockGPS[0][0]-currentLocation[0]);
          double delta_z = fabs(blockGPS[0][2]-currentLocation[2]);
          double distance = sqrt(delta_x*delta_x + delta_z*delta_z);
          
          leftMotor->setPosition(INFINITY);
          rightMotor->setPosition(INFINITY);
          leftMotor->setVelocity(0.5 * MAX_SPEED);
          rightMotor->setVelocity(0.5 * MAX_SPEED);
          
          if(distance < 0.15){
            leftMotor->setVelocity(0);
            rightMotor->setVelocity(0);
            break;
          }
        }while(robot->step(TIME_STEP) != -1);
        
        //bool blockColour = readColour();
        //Some code here to store colour
        close_arms();
        fin2 = 1;
       }*/

/*=========================================
CODE SNIPPET TO OUTPUT SENSOR VALUE 2D ARRAY
===========================================*/
      
      /*//Outputting sensorValueScan 
      for(int j = 0; j<4; j++){
        std::cout << sensorValueScan[i][j]<< ",";
      }
       
      std::cout << "\n";*/

/*============================================
CODE SNIPPET TO OUTPUT AVERAGE DISTANCE
============================================*/

      //Output average distance
      /*int lookUpTableSize = ds[2]->getLookupTableSize();
        if(i==30){
        int numCounter = 0;
        double sumDistance = 0;
        for(int j = 0; j<lookUpTableSize;j++){
          sumDistance += sensorValueScan[j][2];
          if(sensorValueScan[j][2]!=0){
            numCounter++;
          }
        }
        double avgDistance = sumDistance/numCounter;
        std::cout << "DISTANCE AVERAGE:" << avgDistance;
      }*/
      
      i++;
      
      if(i==200){
        break;
       }
       
/*=============================================================
CODE SNIPPET FOR OUTPUTTING LOOKUP TABLE VALUES
===============================================================
*/
       /*if(i==0){
         const double *lookUpTable = ds[2]->getLookupTable();
         int lookUpTableSize = ds[2]->getLookupTableSize();
         for(int j=0; j<lookUpTableSize;j++){
           for(int k=0; k<3;k++){
             std::cout << lookUpTable[3*j+k] << ",";
           }
           std::cout << "\n";
          }
         i++;
       }*/

/*========================================================
CODE SNIPPET FOR OUTPUTTING GPS AND COMPASS VALUES
==========================================================
*/
     
      /*std::cout << initial_bearing << "," << final_bearing << "\n";
      
      double initial_position[3];
      if (i == 0){
      initial_position[0] = gps->getValues()[0];
      initial_position[1] = gps->getValues()[1];
      initial_position[2] = gps->getValues()[2];
     const double initial_north = compass->getValues();
    };
      
      double bearing = get_bearing_in_degrees(north);
      //std::cout << bearing << std::endl;
      //std::cout <<  position[0] << ", "<< position[1] << ", "<< position[2] << std::endl;
      //std::cout <<  north[0] << ", "<< north[1] << ", "<< north[2] << std::endl;
      //std::cout <<  initial_position[0] << ", "<< initial_position[1] << ", "<< initial_position[2] << std::endl;
      //std::cout << i << std::endl;
      //i++;
      */
   }
      
   delete robot;
   return 0;
 }
 